#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <fstream>

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::Image> SyncPolicy;

class projectionLidarLines {
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line1_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line2_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line3_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line4_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;

    std::string result_str;
    std::string cam_config_file_path;

    Eigen::Matrix4d C_T_L;

    cv::Mat D, K;

    std::vector<cv::Point3d> objectPoints_L;
    std::vector<cv::Point2d> imagePoints;

    cv::Mat c_R_l, tvec;
    cv::Mat rvec;
    Eigen::Matrix3d C_R_L;
    Eigen::Vector3d C_t_L;

public:
    projectionLidarLines() {
        line1_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line1_out", 1);
        line2_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line2_out", 1);
        line3_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line3_out", 1);
        line4_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line4_out", 1);
        image_sub = new
                message_filters::Subscriber
                        <sensor_msgs::Image>(nh, "/image_in", 1);

        sync = new
                message_filters::Synchronizer
                        <SyncPolicy>(SyncPolicy(10),
                                     *line1_sub, *line2_sub, *line3_sub, *line4_sub,
                                     *image_sub);
        sync->registerCallback(boost::bind(&projectionLidarLines::callback,
                                           this, _1, _2, _3, _4, _5));

        result_str =
                "/home/subodh/catkin_ws/src/camera_lidar_calibration/calibration/result/C_T_L.txt";

        cam_config_file_path =
                "/home/subodh/catkin_ws/src/camera_lidar_calibration/projection/calib/nerian_left_r_faccal.yaml";
        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_cam_config.isOpened());
        K = cv::Mat::zeros(3, 3, CV_64F);
        D = cv::Mat::zeros(4, 1, CV_64F);
        fs_cam_config["k1"] >> D.at<double>(0);
        fs_cam_config["k2"] >> D.at<double>(1);
        fs_cam_config["p1"] >> D.at<double>(2);
        fs_cam_config["p2"] >> D.at<double>(3);
        fs_cam_config["fx"] >> K.at<double>(0, 0);
        fs_cam_config["fy"] >> K.at<double>(1, 1);
        fs_cam_config["cx"] >> K.at<double>(0, 2);
        fs_cam_config["cy"] >> K.at<double>(1, 2);


        C_T_L = Eigen::Matrix4d::Identity();

        std::ifstream myReadFile(result_str.c_str());
        std::string word;
        int i = 0;
        int j = 0;
        while (myReadFile >> word){
            C_T_L(i, j) = atof(word.c_str());
            j++;
            if(j>3) {
                j = 0;
                i++;
            }
        }

        C_R_L = C_T_L.block(0, 0, 3, 3);
        C_t_L = C_T_L.block(0, 3, 3, 1);


        cv::eigen2cv(C_R_L, c_R_l);
        cv::Rodrigues(c_R_l, rvec);
        cv::eigen2cv(C_t_L, tvec);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &line1_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line2_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line3_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line4_msg,
                  const sensor_msgs::ImageConstPtr &image_msg) {
        objectPoints_L.clear();
        imagePoints.clear();

        cv::Mat image_in;

        image_in = cv_bridge::toCvShare(image_msg, "bgr8")->image;

        pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
        pcl::fromROSMsg(*line1_msg, line_1_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
        pcl::fromROSMsg(*line2_msg, line_2_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_3_pcl;
        pcl::fromROSMsg(*line3_msg, line_3_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_4_pcl;
        pcl::fromROSMsg(*line4_msg, line_4_pcl);

        for(size_t i = 0; i < line_1_pcl.points.size(); i++) {
            cv::Point3d pointCloud_L;
            pointCloud_L.x = line_1_pcl.points[i].x;
            pointCloud_L.y = line_1_pcl.points[i].y;
            pointCloud_L.z = line_1_pcl.points[i].z;
            objectPoints_L.push_back(pointCloud_L);
        }

        for(size_t i = 0; i < line_2_pcl.points.size(); i++) {
            cv::Point3d pointCloud_L;
            pointCloud_L.x = line_2_pcl.points[i].x;
            pointCloud_L.y = line_2_pcl.points[i].y;
            pointCloud_L.z = line_2_pcl.points[i].z;
            objectPoints_L.push_back(pointCloud_L);
        }

        for(size_t i = 0; i < line_3_pcl.points.size(); i++) {
            cv::Point3d pointCloud_L;
            pointCloud_L.x = line_3_pcl.points[i].x;
            pointCloud_L.y = line_3_pcl.points[i].y;
            pointCloud_L.z = line_3_pcl.points[i].z;
            objectPoints_L.push_back(pointCloud_L);
        }

        for(size_t i = 0; i < line_4_pcl.points.size(); i++) {
            cv::Point3d pointCloud_L;
            pointCloud_L.x = line_4_pcl.points[i].x;
            pointCloud_L.y = line_4_pcl.points[i].y;
            pointCloud_L.z = line_4_pcl.points[i].z;
            objectPoints_L.push_back(pointCloud_L);
        }
        cv::projectPoints(objectPoints_L, rvec, tvec, K, D, imagePoints, cv::noArray());
        for(size_t i = 0; i < imagePoints.size(); i++) {
            cv::circle(image_in, imagePoints[i], 1, cv::Scalar(0, 255, 0), -1, 1, 0);
        }
        cv::imshow("view", image_in);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "projection_node");
    projectionLidarLines pLL;
    ros::spin();
    return 0;
}