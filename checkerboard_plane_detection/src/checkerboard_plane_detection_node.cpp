#include <iostream>
#include <sstream>
#include <fstream>

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
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/extract_indices.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <std_msgs/Bool.h>

struct PointXYZIr
{
    PCL_ADD_POINT4D
    ; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t ring; ///< laser ring number
    float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
}EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZIr,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (uint16_t, ring, ring))

class chkrbrdPlaneDetector {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::Publisher cloud_pub_flag;

    // Passthrough filter parameters
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    int min_pts;
    // Plane detection RANSAC param
    double ransac_threshold;

    int view_no;
    std::ofstream points3d_file;
    bool remove_outlier;
    double side_len;
    std::string target_config_file_path;

public:
    chkrbrdPlaneDetector(ros::NodeHandle nh_) {
        nh = nh_;
        cloud_sub = nh.subscribe("/cloud_in", 1,
                                 &chkrbrdPlaneDetector::callback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_out", 1);
        cloud_pub_flag = nh.advertise<std_msgs::Bool>("/plane_out_flag", 1);
        x_min = readParam<double>(nh, "x_min");
        x_max = readParam<double>(nh, "x_max");
        y_min = readParam<double>(nh, "y_min");
        y_max = readParam<double>(nh, "y_max");
        z_min = readParam<double>(nh, "z_min");
        z_max = readParam<double>(nh, "z_max");
        min_pts = readParam<int>(nh, "min_pts");
        ransac_threshold = readParam<double>(nh, "ransac_threshold");
        view_no = 0;
        remove_outlier = readParam<bool>(nh, "remove_outlier");
        if (remove_outlier) {
            target_config_file_path =
                    readParam<std::string>(nh, "target_config_file_path");
            cv::FileStorage fs_target_config(target_config_file_path, cv::FileStorage::READ);
            ROS_ASSERT(fs_target_config.isOpened());
            fs_target_config["side_len"] >> side_len;
        }
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        } else {
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    pcl::PointCloud<pcl::PointXYZI> removeOutliers(pcl::PointCloud<pcl::PointXYZI> planar_pts) {
        pcl::PointCloud<pcl::PointXYZI> planar_pts_filtered;
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
                pcl::PointXYZI pt;
                pt.x = planar_pts.points[i].x;
                pt.y = planar_pts.points[i].y;
                pt.z = planar_pts.points[i].z;
                pt.intensity = planar_pts.points[i].intensity;
                planar_pts_filtered.points.push_back(pt);
            }
        }
        return planar_pts_filtered;
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        view_no++;
        pcl::PointCloud<pcl::PointXYZI>::Ptr
                                cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud_in);

        pcl::PointCloud<pcl::PointXYZI>::Ptr
                cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr
                cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr
                cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::PointCloud<pcl::PointXYZI>::Ptr
                plane(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr
                plane_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        // Pass through filters
        pcl::PassThrough<pcl::PointXYZI> pass_x;
        pass_x.setInputCloud(cloud_in);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min, x_max);
        pass_x.filter(*cloud_filtered_x);
        pcl::PassThrough<pcl::PointXYZI> pass_y;
        pass_y.setInputCloud(cloud_filtered_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min, y_max);
        pass_y.filter(*cloud_filtered_xy);
        pcl::PassThrough<pcl::PointXYZI> pass_z;
        pass_y.setInputCloud(cloud_filtered_xy);
        pass_y.setFilterFieldName("z");
        pass_y.setFilterLimits(z_min, z_max);
        pass_y.filter(*cloud_filtered_xyz);


        // Plane Segmentation
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr
                model_p(new
                                pcl::SampleConsensusModelPlane<pcl::PointXYZI>(cloud_filtered_xyz));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
        ransac.setDistanceThreshold(ransac_threshold);
        ransac.computeModel();
        std::vector<int> inlier_indices;
        ransac.getInliers(inlier_indices);
        pcl::copyPointCloud<pcl::PointXYZI>(*cloud_filtered_xyz,
                                            inlier_indices,
                                            *plane);
        if(remove_outlier) {
            ROS_WARN_STREAM("[Plane Detection]: Removing Outliers");
            *plane_filtered = removeOutliers(*plane);
        } else {
            ROS_WARN_STREAM("[Plane Detection]: Not Removing Outliers");
            plane_filtered = plane;
        }

//        ROS_INFO_STREAM("Model Coeffs: " << ransac.model_coefficients_);
//        ROS_INFO_STREAM("No of points on plane: " << plane->points.size());
//        Eigen::VectorXf plane_coeff = ransac.model_coefficients_;
//        std::cout << plane_coeff.transpose() << std::endl;
//        Eigen::Vector3f normal = Eigen::Vector3f(plane_coeff(0), plane_coeff(1), plane_coeff(2));
        // Publish detected plane
        sensor_msgs::PointCloud2 cloud_out_ros;
        ROS_WARN_STREAM("[Plane Detection]: " << plane_filtered->points.size());
        if(plane_filtered->points.size() > min_pts) {
            pcl::toROSMsg(*plane_filtered, cloud_out_ros);
            cloud_out_ros.header.stamp = cloud_msg->header.stamp;
            cloud_out_ros.header.frame_id = cloud_msg->header.frame_id;
            cloud_pub.publish(cloud_out_ros);
            std_msgs::Bool flag;
            flag.data = true;
            cloud_pub_flag.publish(flag);
        } else {
            std_msgs::Bool flag;
            flag.data = false;
            cloud_pub_flag.publish(flag);
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "checkerboard_plane_detection_node");
    ros::NodeHandle nh("~");
    chkrbrdPlaneDetector cPD(nh);
    ros::spin();
}