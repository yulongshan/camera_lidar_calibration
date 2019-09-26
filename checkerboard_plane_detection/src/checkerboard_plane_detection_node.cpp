#include <iostream>
#include <sstream>

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

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

class chkrbrdPlaneDetector {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    // Passthrough filter parameters
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;

    // Plane detection RANSAC param
    double ransac_threshold;

public:
    chkrbrdPlaneDetector() {
        cloud_sub = nh.subscribe("/cloud_in", 1,
                                 &chkrbrdPlaneDetector::callback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_out", 1);

        x_min = readParam<double>(nh, "x_min");
        x_max = readParam<double>(nh, "x_max");
        y_min = readParam<double>(nh, "y_min");
        y_max = readParam<double>(nh, "y_max");
        z_min = readParam<double>(nh, "z_min");
        z_max = readParam<double>(nh, "z_max");

        ransac_threshold = readParam<double>(nh, "ransac_threshold");
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

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
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
                                pcl::SampleConsensusModelPlane<pcl::PointXYZI>(cloud_filtered_xy));
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
        ransac.setDistanceThreshold(ransac_threshold);
        ransac.computeModel();
        std::vector<int> inlier_indices;
        ransac.getInliers(inlier_indices);
        pcl::copyPointCloud<pcl::PointXYZI>(*cloud_filtered_xy,
                                            inlier_indices,
                                            *plane);
//        ROS_INFO_STREAM("Model Coeffs: " << ransac.model_coefficients_);
//        ROS_INFO_STREAM("No of points on plane: " << plane->points.size());
//        Eigen::VectorXf plane_coeff = ransac.model_coefficients_;
//        std::cout << plane_coeff.transpose() << std::endl;
//        Eigen::Vector3f normal = Eigen::Vector3f(plane_coeff(0), plane_coeff(1), plane_coeff(2));
        // Publish detected plane
        sensor_msgs::PointCloud2 cloud_out_ros;
        pcl::toROSMsg(*plane, cloud_out_ros);
        cloud_out_ros.header.stamp = cloud_msg->header.stamp;
        cloud_out_ros.header.frame_id = cloud_msg->header.frame_id;
        cloud_pub.publish(cloud_out_ros);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "checkerboard_plane_detection_node");
    chkrbrdPlaneDetector cPD;
    ros::spin();
}