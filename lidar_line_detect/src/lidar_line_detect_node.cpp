#define PCL_NO_PRECOMPILE

#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>

#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>


#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp>

#include <opencv2/core.hpp>

struct PointXYZIr {
    PCL_ADD_POINT4D
    ; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t ring; ///< laser ring number
    float yaw; ///< yaw angle of the point
    float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
}EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZIr,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (uint16_t, ring, ring)
        (float, yaw, yaw))

struct lineWithLabel {
    pcl::PointCloud<PointXYZIr> line_pts;
    char labelZ;
    char labelY;
};

struct ringLine {
    int ring_index;
    Eigen::Vector3d a;
    Eigen::Vector3d b;
};

class lidarLineDetect {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_plane_pub;
    ros::Publisher cloud_edges_pub;
    ros::Publisher line_1_pub;
    ros::Publisher line_2_pub;
    ros::Publisher line_3_pub;
    ros::Publisher line_4_pub;

    ros::Publisher projected_line_1_pub;
    ros::Publisher projected_line_2_pub;
    ros::Publisher projected_line_3_pub;
    ros::Publisher projected_line_4_pub;

    std::string node_name;

    // Passthrough filter parameters
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    int min_pts;

    // Plane detection RANSAC param
    double ransac_threshold_fine;
    double ransac_threshold_coarse;
    double ransac_threshold_line;
    double dot_prod_low_val;
    double dot_prod_high_val;
    int min_points_per_line;
    int min_points_per_ring;

    int no_of_rings;
    std::string target_config_file_path;
    double side_len;

    std_msgs::Header header_info;

    std::vector<lineWithLabel> lls;
    std::vector<ringLine> rLs;

    bool lines_published;

    Eigen::Vector4d plane_coeff;

public:
    lidarLineDetect(ros::NodeHandle n) {
        nh = n;
        node_name = ros::this_node::getName();
        cloud_sub = nh.subscribe("/cloud_in", 1, &lidarLineDetect::callback, this);
        cloud_plane_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_plane_out", 1);
        cloud_edges_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_planeedge_out", 1);
        line_1_pub = nh.advertise<sensor_msgs::PointCloud2>("/line1_out", 1);
        line_2_pub = nh.advertise<sensor_msgs::PointCloud2>("/line2_out", 2);
        line_3_pub = nh.advertise<sensor_msgs::PointCloud2>("/line3_out", 3);
        line_4_pub = nh.advertise<sensor_msgs::PointCloud2>("/line4_out", 4);
        projected_line_1_pub = nh.advertise<sensor_msgs::PointCloud2>("/projected_line1_out", 1);
        projected_line_2_pub = nh.advertise<sensor_msgs::PointCloud2>("/projected_line2_out", 1);
        projected_line_3_pub = nh.advertise<sensor_msgs::PointCloud2>("/projected_line3_out", 1);
        projected_line_4_pub = nh.advertise<sensor_msgs::PointCloud2>("/projected_line4_out", 1);

        x_min = readParam<double>(nh, "x_min");
        x_max = readParam<double>(nh, "x_max");
        y_min = readParam<double>(nh, "y_min");
        y_max = readParam<double>(nh, "y_max");
        z_min = readParam<double>(nh, "z_min");
        z_max = readParam<double>(nh, "z_max");
        min_pts = readParam<int>(nh, "min_pts");
        min_points_per_ring = readParam<int>(nh, "min_points_per_ring");
        ransac_threshold_fine = readParam<double>(nh, "ransac_threshold_fine");
        ransac_threshold_coarse = readParam<double>(nh, "ransac_threshold_coarse");
        ransac_threshold_line = readParam<double>(nh, "ransac_threshold_line");
        dot_prod_low_val = readParam<double>(nh, "dot_prod_low_val");
        dot_prod_high_val = readParam<double>(nh, "dot_prod_high_val");
        min_points_per_line = readParam<int>(nh, "min_points_per_line");

        no_of_rings = readParam<int>(nh, "no_of_rings");
        target_config_file_path =
                readParam<std::string>(nh, "target_config_file_path");
        cv::FileStorage fs_target_config(target_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_target_config.isOpened());
        fs_target_config["side_len"] >> side_len;
        ROS_INFO_STREAM("[" << node_name << "]: " << side_len);
        lines_published = false;
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("[" << node_name << "]: " << " Loaded " << name << ": " << ans);
        } else {
            ROS_ERROR_STREAM("[" << node_name << "]: " << " Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    pcl::PointCloud<PointXYZIr> removeOutliers(pcl::PointCloud<PointXYZIr> planar_pts) {
        pcl::PointCloud<PointXYZIr> planar_pts_filtered;
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(planar_pts, centroid);
        double X_mp = centroid(0);
        double Y_mp = centroid(1);
        double Z_mp = centroid(2);
        double max_dist = side_len/sqrt(2);
        for (int i = 0; i < planar_pts.points.size(); i++) {
            double dX = X_mp-planar_pts.points[i].x;
            double dY = Y_mp-planar_pts.points[i].y;
            double dZ = Z_mp-planar_pts.points[i].z;
            double distance = sqrt(dX*dX + dY*dY + dZ*dZ);
            if (distance <= max_dist) {
                PointXYZIr pt;
                pt.x = planar_pts.points[i].x;
                pt.y = planar_pts.points[i].y;
                pt.z = planar_pts.points[i].z;
                pt.intensity = planar_pts.points[i].intensity;
                pt.ring = planar_pts.points[i].ring;
                planar_pts_filtered.points.push_back(pt);
            }
        }
        return planar_pts_filtered;
    }

    std::vector<std::vector<PointXYZIr> > getRings(pcl::PointCloud<PointXYZIr> cloud_in){
        std::vector<std::vector<PointXYZIr> > rings(no_of_rings);
        for(int i = 0; i < cloud_in.points.size(); i++) {
            ROS_ASSERT(cloud_in.points[i].ring < no_of_rings);
            cloud_in.points[i].yaw = atan2(cloud_in.points[i].y, cloud_in.points[i].x);
            rings[cloud_in.points[i].ring].push_back(cloud_in.points[i]);
        }
        return rings;
    }

    void publishCloud(pcl::PointCloud<PointXYZIr> cloud_in, ros::Publisher cloud_pub) {
        sensor_msgs::PointCloud2 cloud_out_ros;
        pcl::toROSMsg(cloud_in, cloud_out_ros);
        cloud_out_ros.header.stamp = header_info.stamp;
        cloud_out_ros.header.frame_id = header_info.frame_id;
        cloud_pub.publish(cloud_out_ros);
    }

    pcl::PointCloud<PointXYZIr> passThruFilterCld(pcl::PointCloud<PointXYZIr> cloud_in) {
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_in_ptr(new pcl::PointCloud<PointXYZIr>);
        *cloud_in_ptr = cloud_in;
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_filtered_x(new pcl::PointCloud<PointXYZIr>);
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_filtered_xy(new pcl::PointCloud<PointXYZIr>);
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_filtered_xyz(new pcl::PointCloud<PointXYZIr>);
        pcl::PointCloud<PointXYZIr>::Ptr
                plane(new pcl::PointCloud<PointXYZIr>);

        // Pass Thru Filters
        pcl::PassThrough<PointXYZIr> pass_x;
        pass_x.setInputCloud(cloud_in_ptr);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min, x_max);
        pass_x.filter(*cloud_filtered_x);
        pcl::PassThrough<PointXYZIr> pass_y;
        pass_y.setInputCloud(cloud_filtered_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min, y_max);
        pass_y.filter(*cloud_filtered_xy);
        pcl::PassThrough<PointXYZIr> pass_z;
        pass_y.setInputCloud(cloud_filtered_xy);
        pass_y.setFilterFieldName("z");
        pass_y.setFilterLimits(z_min, z_max);
        pass_y.filter(*cloud_filtered_xyz);

        return *cloud_filtered_xyz;
    }

    Eigen::Vector4d getTargetPlane(pcl::PointCloud<PointXYZIr> cloud_in) {
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_in_ptr(new pcl::PointCloud<PointXYZIr>);
        *cloud_in_ptr = cloud_in;
        pcl::PointCloud<PointXYZIr>::Ptr
                plane(new pcl::PointCloud<PointXYZIr>);
        // Plane Segmentation
        pcl::SampleConsensusModelPlane<PointXYZIr>::Ptr
                model_p(new pcl::SampleConsensusModelPlane<PointXYZIr>(cloud_in_ptr));
        pcl::RandomSampleConsensus<PointXYZIr> ransac(model_p);
        ransac.setDistanceThreshold(ransac_threshold_fine);
        ransac.computeModel();
        std::vector<int> inlier_indices;
        ransac.getInliers(inlier_indices);
        pcl::copyPointCloud<PointXYZIr>(*cloud_in_ptr, inlier_indices, *plane);
        publishCloud(*plane, cloud_plane_pub);

        Eigen::VectorXf plane_coeff_xf = ransac.model_coefficients_;
        Eigen::Vector4d plane_coeff(plane_coeff_xf(0), plane_coeff_xf(1),
                                    plane_coeff_xf(2), plane_coeff_xf(3));

        return plane_coeff;
    }

    pcl::PointCloud<PointXYZIr> getPlanarPoints(pcl::PointCloud<PointXYZIr> cloud_in,
                                                double rnsc_thres) {
        pcl::PointCloud<PointXYZIr>::Ptr cloud_in_ptr(new pcl::PointCloud<PointXYZIr>);
        *cloud_in_ptr = cloud_in;
        pcl::PointCloud<PointXYZIr> plane;
        pcl::SampleConsensusModelPlane<PointXYZIr>::Ptr model_p(new
                                                                        pcl::SampleConsensusModelPlane<PointXYZIr>(cloud_in_ptr));
        pcl::RandomSampleConsensus<PointXYZIr> ransac(model_p);
        ransac.setDistanceThreshold(rnsc_thres);
        bool model_computed = ransac.computeModel();
        std::vector<int> inlier_indices;
        if (model_computed) {
            Eigen::VectorXf model_coeffs = ransac.model_coefficients_;
            double n_x = fabs(model_coeffs(0));
            ransac.getInliers(inlier_indices);
            pcl::copyPointCloud<PointXYZIr>(*cloud_in_ptr,
                                            inlier_indices,
                                            plane);
        }
        return plane;
    }

    Eigen::VectorXf fit3DLine(pcl::PointCloud<PointXYZIr> points_3D) {
        pcl::PointCloud<PointXYZIr>::Ptr cloud_ptr(new pcl::PointCloud<PointXYZIr>);
        *cloud_ptr = points_3D;
        pcl::SampleConsensusModelLine<PointXYZIr>::Ptr model_l(new pcl::SampleConsensusModelLine<PointXYZIr>(cloud_ptr));
        pcl::RandomSampleConsensus<PointXYZIr> ransac_l(model_l);
        ransac_l.setDistanceThreshold(ransac_threshold_line);
        ransac_l.computeModel();
        std::vector<int> line_inliers;
        ransac_l.getInliers(line_inliers);
        Eigen::VectorXf line_coeff = ransac_l.model_coefficients_ ;
        return line_coeff;
    }

    pcl::PointCloud<PointXYZIr> getTargetEdges(std::vector<std::vector<PointXYZIr> > rings) {
        rLs.clear();
        pcl::PointCloud<PointXYZIr> edges_cloud;
        for(int i = 0; i < rings.size(); i++) {
            if(rings[i].size() >= min_points_per_ring) {
                std::vector<float> yaw_values;
                pcl::PointCloud<PointXYZIr> points_3D;
                for(int j = 0; j < rings[i].size(); j++) {
                    PointXYZIr pt = rings[i][j];
                    yaw_values.push_back(pt.yaw);
                    points_3D.points.push_back(pt);
                }
                Eigen::VectorXf line = fit3DLine(points_3D);
                Eigen::Vector3d A(line(0), line(1), line(2));
                Eigen::Vector3d B(line(3), line(4), line(5));
                ringLine rL;
                rL.ring_index = i;
                rL.a = A;
                rL.b = B;
                rLs.push_back(rL);
                long maxElementIndex = std::max_element(yaw_values.begin(), yaw_values.end()) - yaw_values.begin();
                float maxElement = *std::max_element(yaw_values.begin(), yaw_values.end());

                long minElementIndex = std::min_element(yaw_values.begin(), yaw_values.end()) - yaw_values.begin();
                float minElement = *std::min_element(yaw_values.begin(), yaw_values.end());                edges_cloud.points.push_back(rings[i][maxElementIndex]);
                edges_cloud.points.push_back(rings[i][minElementIndex]);
            }
        }
        return edges_cloud;
    }

    pcl::PointCloud<PointXYZIr> getPlaneEdges(pcl::PointCloud<PointXYZIr> cloud_in) {
        pcl::PointCloud<PointXYZIr>::Ptr
                plane(new pcl::PointCloud<PointXYZIr>);
        *plane = getPlanarPoints(cloud_in, ransac_threshold_coarse);
        pcl::PointCloud<PointXYZIr> edge_plane;
        if (plane->points.size() > 0) {
            pcl::PointCloud<PointXYZIr>::Ptr
                    plane_filtered(new pcl::PointCloud<PointXYZIr>);
            *plane_filtered = removeOutliers(*plane);
            std::vector<std::vector<PointXYZIr> > rings = getRings(*plane_filtered);
            pcl::PointCloud<PointXYZIr> edge_cloud = getTargetEdges(rings);
            edge_plane = getPlanarPoints(edge_cloud, ransac_threshold_fine);
            publishCloud(edge_plane, cloud_edges_pub);
        }
        return edge_plane;
    }

    void remove_inliers(const pcl::PointCloud<PointXYZIr> &cloud_in,
                        std::vector<int> inliers_indices,
                        pcl::PointCloud<PointXYZIr> &cloud_out) {

        std::vector<int> outliers_indicies;
        for (size_t i = 0; i < cloud_in.size(); i++)
        {
            if (find(inliers_indices.begin(), inliers_indices.end(), i) == inliers_indices.end())
            {
                outliers_indicies.push_back(i);
            }
        }
        pcl::copyPointCloud<PointXYZIr>(cloud_in, outliers_indicies, cloud_out);
    }

    Eigen::Vector3d getCentroid(pcl::PointCloud<PointXYZIr> cloud_in) {
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(cloud_in, centroid);
        return Eigen::Vector3d(centroid(0), centroid(1), centroid(2));
    }

    void labelLines(char axis) {
        ROS_ASSERT(lls.size() == 4);
        std::vector<std::pair<double, int>> dists_axis_mp;
        for(int i = 0; i < 4; i++) {
            Eigen::Vector3d centroid_i = getCentroid(lls[i].line_pts);
            if(axis == 'z')
                dists_axis_mp.push_back(std::make_pair(centroid_i.z(), i));
            else if(axis == 'y')
                dists_axis_mp.push_back(std::make_pair(centroid_i.y(), i));
            else
                ROS_ASSERT(axis == 'z' || axis == 'y');
        }
        ROS_ASSERT(dists_axis_mp.size() == 4);
        std::sort(dists_axis_mp.begin(),
                  dists_axis_mp.end(),
                  std::greater<std::pair<double, int>>());
        for (int i = 0; i < 4; i++) {
            if(axis == 'z') {
                if(i <= 1)
                    lls[dists_axis_mp[i].second].labelZ = 'b';
                else
                    lls[dists_axis_mp[i].second].labelZ = 't';
            } else if (axis == 'y') {
                if(i <= 1)
                    lls[dists_axis_mp[i].second].labelY = 'l';
                else
                    lls[dists_axis_mp[i].second].labelY = 'r';
            } else {
                ROS_ASSERT(axis == 'z' || axis == 'y');
            }
        }
    }

    pcl::PointCloud<PointXYZIr> getFilteredLine(pcl::PointCloud<PointXYZIr> line_in) {
        pcl::PointCloud<PointXYZIr>::Ptr cloud_ptr(new pcl::PointCloud<PointXYZIr>);
        pcl::PointCloud<PointXYZIr>::Ptr line_out(new pcl::PointCloud<PointXYZIr>);
        *cloud_ptr = line_in;
        pcl::SampleConsensusModelLine<PointXYZIr>::Ptr model_l(new pcl::SampleConsensusModelLine<PointXYZIr>(cloud_ptr));
        pcl::RandomSampleConsensus<PointXYZIr> ransac_l(model_l);
        ransac_l.setDistanceThreshold(ransac_threshold_line/2);
        ransac_l.computeModel();
        std::vector<int> line_inliers;
        ransac_l.getInliers(line_inliers);
        pcl::copyPointCloud<PointXYZIr>(*cloud_ptr, line_inliers, *line_out);
        return *line_out;
    }

    void publishLinesInOrder() {
        ros::Time time_stamp = header_info.stamp;
        std::string frame_id = header_info.frame_id;
        ROS_INFO_STREAM("[ " << node_name << " ] " << "Publishing LIDAR Lines");
        ROS_ASSERT(lls.size() == 4);
        sensor_msgs::PointCloud2 line1_ros, line2_ros, line3_ros, line4_ros;
        pcl::PointCloud<PointXYZIr> line1_plane_projected;
        pcl::PointCloud<PointXYZIr> line2_plane_projected;
        pcl::PointCloud<PointXYZIr> line3_plane_projected;
        pcl::PointCloud<PointXYZIr> line4_plane_projected;
        bool line1_flag = false;
        bool line2_flag = false;
        bool line3_flag = false;
        bool line4_flag = false;
        for(size_t i = 0; i < 4; i++) {
            char labelZ = lls[i].labelZ;
            char labelY = lls[i].labelY;
            if(labelZ == 'b' && labelY == 'l') {
                pcl::toROSMsg(lls[i].line_pts, line1_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line1_ros.header.stamp = time_stamp;
                    line1_ros.header.frame_id = frame_id;
                    line1_flag = true;
                    line1_plane_projected = projectOnPlane(plane_coeff, lls[i].line_pts);
                }
            }
            if(labelZ == 'b' && labelY == 'r') {
                pcl::toROSMsg(lls[i].line_pts, line2_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line2_ros.header.stamp = time_stamp;
                    line2_ros.header.frame_id = frame_id;
                    line2_flag = true;
                    line2_plane_projected = projectOnPlane(plane_coeff, lls[i].line_pts);
                }
            }
            if(labelZ == 't' && labelY == 'r') {
                pcl::toROSMsg(lls[i].line_pts, line3_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line3_ros.header.stamp = time_stamp;
                    line3_ros.header.frame_id = frame_id;
                    line3_flag = true;
                    line3_plane_projected = projectOnPlane(plane_coeff, lls[i].line_pts);
                }
            }
            if(labelZ == 't' && labelY == 'l') {
                pcl::toROSMsg(lls[i].line_pts, line4_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line4_ros.header.stamp = time_stamp;
                    line4_ros.header.frame_id = frame_id;
                    line4_flag = true;
                    line4_plane_projected = projectOnPlane(plane_coeff, lls[i].line_pts);
                }
            }
        }
        if(line1_flag && line2_flag && line3_flag && line4_flag) {

            line_1_pub.publish(line1_ros);
            pcl::PointCloud<PointXYZIr> line1_line_projected =
                    projectOnCorrespondingLine(line1_plane_projected);
            publishCloud(getFilteredLine(line1_line_projected), projected_line_1_pub);

            line_2_pub.publish(line2_ros);
            pcl::PointCloud<PointXYZIr> line2_line_projected =
                    projectOnCorrespondingLine(line2_plane_projected);
            publishCloud(getFilteredLine(line2_line_projected), projected_line_2_pub);

            line_3_pub.publish(line3_ros);
            pcl::PointCloud<PointXYZIr> line3_line_projected =
                    projectOnCorrespondingLine(line3_plane_projected);
            publishCloud(getFilteredLine(line3_line_projected), projected_line_3_pub);

            line_4_pub.publish(line4_ros);
            pcl::PointCloud<PointXYZIr> line4_line_projected =
                    projectOnCorrespondingLine(line4_plane_projected);
            publishCloud(getFilteredLine(line4_line_projected), projected_line_4_pub);

            lines_published = true;
        } else {
            lines_published = false;
        }
    }

    void detectLines(pcl::PointCloud<PointXYZIr> edge_cloud_in) {
        lls.clear();
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_msg_pcl(new pcl::PointCloud<PointXYZIr>);
        *cloud_msg_pcl = edge_cloud_in;
        int no_of_incoming_pts = cloud_msg_pcl->points.size();
        std::vector<pcl::PointCloud<PointXYZIr>> lines_pts;
        std::vector<Eigen::VectorXf> lines_eqns;
        for (int i = 0; i < 4; i++) {
            pcl::PointCloud<PointXYZIr>::Ptr cloud_ptr(cloud_msg_pcl);
            pcl::SampleConsensusModelLine<PointXYZIr>::Ptr model_l(new pcl::SampleConsensusModelLine<PointXYZIr>(cloud_ptr));
            pcl::RandomSampleConsensus<PointXYZIr> ransac_l(model_l);
            ransac_l.setDistanceThreshold(ransac_threshold_line);
            ransac_l.computeModel();
            std::vector<int> line_inliers;
            ransac_l.getInliers(line_inliers);
            if(!line_inliers.empty()) {
                pcl::PointCloud<PointXYZIr> line_i;
                pcl::copyPointCloud<PointXYZIr>(*cloud_msg_pcl,
                                                    line_inliers,
                                                    line_i);
                lines_pts.push_back(line_i);
                lines_eqns.push_back(ransac_l.model_coefficients_);
            } else {
                break;
            }
            pcl::PointCloud<PointXYZIr> plane_no_line;
            remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
            *cloud_msg_pcl = plane_no_line;
        }
        if (lines_pts.size() == 4) {
            pcl::PointCloud<PointXYZIr> line1_pcl = lines_pts[0];
            pcl::PointCloud<PointXYZIr> line2_pcl = lines_pts[1];
            pcl::PointCloud<PointXYZIr> line3_pcl = lines_pts[2];
            pcl::PointCloud<PointXYZIr> line4_pcl = lines_pts[3];
            ulong no_pts_l1 = line1_pcl.points.size();
            ulong no_pts_l2 = line2_pcl.points.size();
            ulong no_pts_l3 = line3_pcl.points.size();
            ulong no_pts_l4 = line4_pcl.points.size();
            Eigen::VectorXf line1_eqn = lines_eqns[0];
            Eigen::Vector3d line1_direction = Eigen::Vector3d(line1_eqn[3],
                                                              line1_eqn[4],
                                                              line1_eqn[5]);
            Eigen::VectorXf line2_eqn = lines_eqns[1];
            Eigen::Vector3d line2_direction = Eigen::Vector3d(line2_eqn[3],
                                                              line2_eqn[4],
                                                              line2_eqn[5]);
            Eigen::VectorXf line3_eqn = lines_eqns[2];
            Eigen::Vector3d line3_direction = Eigen::Vector3d(line3_eqn[3],
                                                              line3_eqn[4],
                                                              line3_eqn[5]);
            Eigen::VectorXf line4_eqn = lines_eqns[3];
            Eigen::Vector3d line4_direction = Eigen::Vector3d(line4_eqn[3],
                                                              line4_eqn[4],
                                                              line4_eqn[5]);

            double angle1 = fabs(line1_direction.dot(line2_direction));
            double angle2 = fabs(line1_direction.dot(line3_direction));
            double angle3 = fabs(line1_direction.dot(line4_direction));
            std::vector<double> angles_arr;
            angles_arr.push_back(angle1);
            angles_arr.push_back(angle2);
            angles_arr.push_back(angle3);
            int countRA = 0;
            int countP = 0;
            for(int m = 0; m < angles_arr.size(); m++) {
                if(angles_arr[m] < dot_prod_low_val)
                    countRA++;
                if(angles_arr[m] > dot_prod_high_val)
                    countP++;
            }
            bool count_condition = (no_pts_l1 > min_points_per_line &&
                                    no_pts_l2 > min_points_per_line &&
                                    no_pts_l3 > min_points_per_line &&
                                    no_pts_l4 > min_points_per_line);
            bool angle_conditon = (countRA == 2 && countP == 1);
            if (count_condition &&
                angle_conditon) {
                for (int l = 0; l < 4; l++) {
                    lineWithLabel ll;
                    ll.line_pts = lines_pts[l];
                    lls.push_back(ll);
                }
                labelLines('z');
                labelLines('y');
                publishLinesInOrder();
            }  else {
                lines_published = false;
            }
        }
    }

    pcl::PointCloud<PointXYZIr> projectOnPlane(Eigen::Vector4d plane_coff, pcl::PointCloud<PointXYZIr> cloud_pts) {
        pcl::PointCloud<PointXYZIr>::Ptr cloud_in (new pcl::PointCloud<PointXYZIr>);
        *cloud_in = cloud_pts;
        pcl::PointCloud<PointXYZIr> cloud_projected;

        // Model Coeffs
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = plane_coff(0);
        coefficients->values[1] = plane_coff(1);
        coefficients->values[2] = plane_coff(2);
        coefficients->values[3] = plane_coff(3);

        // Create the filtering object
        pcl::ProjectInliers<PointXYZIr> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud_in);
        proj.setModelCoefficients (coefficients);
        proj.filter (cloud_projected);

        return cloud_projected;
    }

    pcl::PointCloud<PointXYZIr> projectOnLine(Eigen::VectorXf line_coeffs, pcl::PointCloud<PointXYZIr> line_pt) {
        pcl::PointCloud<PointXYZIr>::Ptr cloud_in (new pcl::PointCloud<PointXYZIr>);
        *cloud_in = line_pt;
        pcl::PointCloud<PointXYZIr> cloud_projected;

        // Model Coeffs
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (6);
        coefficients->values[0] = line_coeffs(0);
        coefficients->values[1] = line_coeffs(1);
        coefficients->values[2] = line_coeffs(2);
        coefficients->values[3] = line_coeffs(3);
        coefficients->values[4] = line_coeffs(4);
        coefficients->values[5] = line_coeffs(5);

        // Create the filtering object
        pcl::ProjectInliers<PointXYZIr> proj;
        proj.setModelType (pcl::SACMODEL_LINE);
        proj.setInputCloud (cloud_in);
        proj.setModelCoefficients (coefficients);
        proj.filter (cloud_projected);

        return cloud_projected;
    }

    pcl::PointCloud<PointXYZIr> projectOnCorrespondingLine(pcl::PointCloud<PointXYZIr> cloud_pts) {
        pcl::PointCloud<PointXYZIr> cloud_out;
        int no_of_pts = cloud_pts.points.size();
        for(int i = 0; i < no_of_pts; i++) {
            int ring_no = cloud_pts.points[i].ring;
            Eigen::Vector3d A, B;
            bool match_found = false;
            for(int j = 0; j < rLs.size(); j++) {
                int index = rLs[j].ring_index;
                if (ring_no == index) {
                    A = rLs[j].a.transpose();
                    B = rLs[j].b.transpose();
                    match_found = true;
                    break;
                }
            }
            if (match_found) {
                Eigen::VectorXf line_coeffs(6);
                line_coeffs(0) = A(0);
                line_coeffs(1) = A(1);
                line_coeffs(2) = A(2);
                line_coeffs(3) = B(0);
                line_coeffs(4) = B(1);
                line_coeffs(5) = B(2);
                pcl::PointCloud<PointXYZIr> line_pt;
                line_pt.points.push_back(cloud_pts.points[i]);
                pcl::PointCloud<PointXYZIr> line_pt_projected = projectOnLine(line_coeffs, line_pt);
                cloud_out.points.push_back(line_pt_projected.points[0]);
            }
        }
        return cloud_out;
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        header_info.stamp = cloud_msg->header.stamp;
        header_info.frame_id = cloud_msg->header.frame_id;
        pcl::PointCloud<PointXYZIr> cloud_in_pcl;
        pcl::fromROSMsg(*cloud_msg, cloud_in_pcl);
        pcl::PointCloud<PointXYZIr> cloud_in_passthru_filtered = passThruFilterCld(cloud_in_pcl);
        plane_coeff = getTargetPlane(cloud_in_passthru_filtered);
        pcl::PointCloud<PointXYZIr> edge_plane = getPlaneEdges(cloud_in_passthru_filtered);
        detectLines(edge_plane);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_line_detect");
    ros::NodeHandle nh("~");
    lidarLineDetect lLD(nh);
    ros::spin();
}