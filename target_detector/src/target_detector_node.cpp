#define PCL_NO_PRECOMPILE

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>    // std::sort
#include <vector>       // std::vector

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <velodyne_pointcloud/point_types.h>

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
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

struct PointXYZIr
{
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

class targetDetector {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    // Passthrough filter parameters
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    int min_pts;
    // Plane detection RANSAC param
    double ransac_threshold_coarse;
    double ransac_threshold_fine;

    int view_no;
    bool remove_outlier;
    double side_len;
    std::string target_config_file_path;
    int no_of_rings;

    std::string node_name;
public:
    targetDetector(ros::NodeHandle nh_) {
        nh = nh_;
        node_name = ros::this_node::getName();
        cloud_sub = nh.subscribe("/cloud_in", 1,
                                 &targetDetector::callback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_out", 1);

        x_min = readParam<double>(nh, "x_min");
        x_max = readParam<double>(nh, "x_max");
        y_min = readParam<double>(nh, "y_min");
        y_max = readParam<double>(nh, "y_max");
        z_min = readParam<double>(nh, "z_min");
        z_max = readParam<double>(nh, "z_max");
        min_pts = readParam<int>(nh, "min_pts");
        ransac_threshold_coarse = readParam<double>(nh, "ransac_threshold_coarse");
        ransac_threshold_fine = readParam<double>(nh, "ransac_threshold_fine");
        view_no = 0;
        remove_outlier = readParam<bool>(nh, "remove_outlier");
        if (remove_outlier) {
            target_config_file_path =
                    readParam<std::string>(nh, "target_config_file_path");
            cv::FileStorage fs_target_config(target_config_file_path, cv::FileStorage::READ);
            ROS_ASSERT(fs_target_config.isOpened());
            fs_target_config["side_len"] >> side_len;
        }
        no_of_rings = readParam<int>(nh, "no_of_rings");
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("[" << node_name << "] " << " Loaded " << name << ": " << ans);
        } else {
            ROS_ERROR_STREAM("[" << node_name << "] " << " Failed to load " << name);
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
                pt.ring = planar_pts.points[i].ring;
                pt.intensity = planar_pts.points[i].intensity;
                planar_pts_filtered.points.push_back(pt);
            }
        }
        return planar_pts_filtered;
    }

    std::vector<std::vector<PointXYZIr> > getRings(pcl::PointCloud<PointXYZIr>::Ptr cloud_in){
        std::vector<std::vector<PointXYZIr> > rings(no_of_rings);
        for(int i = 0; i < cloud_in->points.size(); i++) {
            ROS_ASSERT(cloud_in->points[i].ring < no_of_rings);
            cloud_in->points[i].yaw = atan2(cloud_in->points[i].y, cloud_in->points[i].x);
            rings[cloud_in->points[i].ring].push_back(cloud_in->points[i]);
        }
        return rings;
    }

    pcl::PointCloud<PointXYZIr> getTargetEdges(std::vector<std::vector<PointXYZIr> > rings) {
        pcl::PointCloud<PointXYZIr> edges_cloud;
        for(int i = 0; i < rings.size(); i++) {
            if(rings[i].size() > 0) {
                std::vector<float> yaw_values;
                for(int j = 0; j < rings[i].size(); j++) {
                    PointXYZIr pt = rings[i][j];
                    yaw_values.push_back(pt.yaw);
                }
                long maxElementIndex = std::max_element(yaw_values.begin(), yaw_values.end()) - yaw_values.begin();
                float maxElement = *std::max_element(yaw_values.begin(), yaw_values.end());

                long minElementIndex = std::min_element(yaw_values.begin(), yaw_values.end()) - yaw_values.begin();
                float minElement = *std::min_element(yaw_values.begin(), yaw_values.end());

//                std::cout << "maxElementIndex:" << maxElementIndex << ", maxElement:" << maxElement*180/M_PI << std::endl;
//                std::cout << "minElementIndex:" << minElementIndex << ", minElement:" << minElement*180/M_PI << std::endl;
                edges_cloud.points.push_back(rings[i][maxElementIndex]);
                edges_cloud.points.push_back(rings[i][minElementIndex]);
            }
        }
        return edges_cloud;
    }

    pcl::PointCloud<PointXYZIr> getPlanarPoints(pcl::PointCloud<PointXYZIr> cloud_in, double rnsc_thres) {
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
            ROS_INFO_STREAM("[" << node_name << "] " << model_coeffs.transpose());
            double n_x = fabs(model_coeffs(0));
            ROS_INFO_STREAM("N_x: " << n_x);
            ransac.getInliers(inlier_indices);
            pcl::copyPointCloud<PointXYZIr>(*cloud_in_ptr,
                                                inlier_indices,
                                                plane);
        }
        return plane;
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        view_no++;
        pcl::PointCloud<PointXYZIr>::Ptr
                                cloud_in(new pcl::PointCloud<PointXYZIr>);
        pcl::fromROSMsg(*cloud_msg, *cloud_in);

        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_filtered_x(new pcl::PointCloud<PointXYZIr>);
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_filtered_xy(new pcl::PointCloud<PointXYZIr>);
        pcl::PointCloud<PointXYZIr>::Ptr
                cloud_filtered_xyz(new pcl::PointCloud<PointXYZIr>);

        // Pass through filters
        pcl::PassThrough<PointXYZIr> pass_x;
        pass_x.setInputCloud(cloud_in);
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

        pcl::PointCloud<PointXYZIr>::Ptr
                plane(new pcl::PointCloud<PointXYZIr>);
        *plane = getPlanarPoints(*cloud_filtered_xyz, ransac_threshold_coarse);
        if (plane->points.size() > 0) {
            pcl::PointCloud<PointXYZIr>::Ptr
                    plane_filtered(new pcl::PointCloud<PointXYZIr>);
            if(remove_outlier) {
                ROS_WARN_STREAM("[" << node_name << "] " << " Removing Outliers");
                *plane_filtered = removeOutliers(*plane);
            } else {
                ROS_WARN_STREAM("[" << node_name << "] " << " Not Removing Outliers");
                plane_filtered = plane;
            }

            std::vector<std::vector<PointXYZIr> > rings = getRings(plane_filtered);
            pcl::PointCloud<PointXYZIr> edge_cloud = getTargetEdges(rings);

            pcl::PointCloud<PointXYZIr>::Ptr
                    edge_plane(new pcl::PointCloud<PointXYZIr>);
            *edge_plane = getPlanarPoints(edge_cloud, ransac_threshold_fine);
            sensor_msgs::PointCloud2 cloud_out_ros;
            if(edge_plane->points.size() > min_pts) {
                pcl::toROSMsg(*edge_plane, cloud_out_ros);
                cloud_out_ros.header.stamp = cloud_msg->header.stamp;
                cloud_out_ros.header.frame_id = cloud_msg->header.frame_id;
                cloud_pub.publish(cloud_out_ros);
            }
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_detector_node");
    ros::NodeHandle nh("~");
    targetDetector cPD(nh);
    ros::spin();
}
