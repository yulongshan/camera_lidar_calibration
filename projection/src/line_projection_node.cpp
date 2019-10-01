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

#include "line_msg/line.h"

#include <iostream>
#include <fstream>

typedef message_filters::sync_policies::ApproximateTime
        <line_msg::line,
         line_msg::line,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::Image> SyncPolicy;

class projectionLidarLines {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<line_msg::line> *line1_image_sub;
    message_filters::Subscriber<line_msg::line> *line2_image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line1_cloud_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line2_cloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;

    std::string result_str;
    std::string cam_config_file_path;

    Eigen::Matrix4d C_T_L;

    cv::Mat D, K;
    int image_width, image_height;


    double fov_x, fov_y;

    cv::Mat c_R_l, tvec;
    cv::Mat rvec;
    Eigen::Matrix3d C_R_L;
    Eigen::Vector3d C_t_L;

public:
    projectionLidarLines() {
        line1_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image1", 1);
        line2_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image2", 1);
        line1_cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line1_out", 1);
        line2_cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/line2_out", 1);
        image_sub = new
                message_filters::Subscriber
                         <sensor_msgs::Image>(nh, "/image_in", 1);
        sync = new message_filters::Synchronizer
                        <SyncPolicy>(SyncPolicy(10),
                                     *line1_image_sub,
                                     *line2_image_sub,
                                     *line1_cloud_sub,
                                     *line2_cloud_sub,
                                     *image_sub);
        sync->registerCallback(boost::bind(&projectionLidarLines::callback,
                                           this, _1, _2, _3, _4, _5));

        result_str = readParam<std::string>(nh, "result_str");
        cam_config_file_path = readParam<std::string>(nh, "cam_config_file_path");

        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        ROS_ASSERT(fs_cam_config.isOpened());
        K = cv::Mat::zeros(3, 3, CV_64F);
        D = cv::Mat::zeros(4, 1, CV_64F);
        fs_cam_config["image_height"] >> image_height;
        fs_cam_config["image_width"] >> image_width;
        fs_cam_config["k1"] >> D.at<double>(0);
        fs_cam_config["k2"] >> D.at<double>(1);
        fs_cam_config["p1"] >> D.at<double>(2);
        fs_cam_config["p2"] >> D.at<double>(3);
        fs_cam_config["fx"] >> K.at<double>(0, 0);
        fs_cam_config["fy"] >> K.at<double>(1, 1);
        fs_cam_config["cx"] >> K.at<double>(0, 2);
        fs_cam_config["cy"] >> K.at<double>(1, 2);

        fov_x = 2*atan2(image_width, 2*K.at<double>(0, 0))*180/CV_PI;
        fov_y = 2*atan2(image_height, 2*K.at<double>(1, 1))*180/CV_PI;

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

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    std::vector<cv::Point2d> getProjectedPts(pcl::PointCloud<pcl::PointXYZ> cloud_in) {
        std::vector<cv::Point3d> objectPoints_L;
        std::vector<cv::Point2d> imagePoints;
        for(int i = 0; i < cloud_in.points.size(); i++) {
            if(cloud_in.points[i].x < 0 || cloud_in.points[i].x > 3)
                continue;

            Eigen::Vector4d pointCloud_L;
            pointCloud_L[0] = cloud_in.points[i].x;
            pointCloud_L[1] = cloud_in.points[i].y;
            pointCloud_L[2] = cloud_in.points[i].z;
            pointCloud_L[3] = 1;

            Eigen::Vector3d pointCloud_C;
            pointCloud_C = C_T_L.block(0, 0, 3, 4) * pointCloud_L;

            double X = pointCloud_C[0];
            double Y = pointCloud_C[1];
            double Z = pointCloud_C[2];

            double Xangle = atan2(X, Z)*180/CV_PI;
            double Yangle = atan2(Y, Z)*180/CV_PI;


            if(Xangle < -fov_x/2 || Xangle > fov_x/2)
                continue;

            if(Yangle < -fov_y/2 || Yangle > fov_y/2)
                continue;
            objectPoints_L.push_back(cv::Point3d(pointCloud_L[0], pointCloud_L[1], pointCloud_L[2]));
        }
        if(cloud_in.points.size() > 0)
            cv::projectPoints(objectPoints_L, rvec, tvec, K, D, imagePoints, cv::noArray());
        return imagePoints;
    }

    cv::Vec3f getEqnOfLine(cv::Vec4f line) {
        double x_a = line[0];
        double y_a = line[1];
        double x_b = line[2];
        double y_b = line[3];

        if(x_a == y_a && x_b == y_b) {
            return cv::Vec3f(0, 0, 0);
        } else if(x_a == x_b) {
            // eqn: x = x_a or x = x_b
            return cv::Vec3f(1, 0, -x_a);
        } else if(y_a == y_b){
            // eqn: y = y_a or y = y_b
            return cv::Vec3f(0, 1, -y_a);
        } else {
            double m = (y_b - y_a)/(x_b - x_a);
            double a = m;
            double b = -1;
            double c = y_a - m*x_a;
            return cv::Vec3f(a, b, c);
        }
    }

    double distanceFromLine(cv::Vec3f eqn, cv::Point2f pt) {
        float a = eqn(0);
        float b = eqn(1);
        float c = eqn(2);
        float x_0 = pt.x;
        float y_0 = pt.y;
        double dist = fabs(a*x_0+b*y_0+c)/sqrt(a*a+b*b);
    }

    void callback(const line_msg::lineConstPtr& line1_img_msg,
                  const line_msg::lineConstPtr& line2_img_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line1_cloud_msg,
                  const sensor_msgs::PointCloud2ConstPtr& line2_cloud_msg,
                  const sensor_msgs::ImageConstPtr& image_msg) {

        cv::Mat image_in;
        try {
            image_in = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
        }
        cv::Point2f line1_start = cv::Point2f(line1_img_msg->a1, line1_img_msg->b1);
        cv::Point2f line1_end = cv::Point2f(line1_img_msg->a2, line1_img_msg->b2);
        cv::Point2f line2_start = cv::Point2f(line2_img_msg->a1, line2_img_msg->b1);
        cv::Point2f line2_end = cv::Point2f(line2_img_msg->a2, line2_img_msg->b2);
        cv::line(image_in, line1_start, line1_end, cv::Scalar(0, 255, 255), 2, cv::LINE_8);
        cv::line(image_in, line2_start, line2_end, cv::Scalar(0, 255, 255), 2, cv::LINE_8);

        pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
        pcl::fromROSMsg(*line1_cloud_msg, line_1_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
        pcl::fromROSMsg(*line2_cloud_msg, line_2_pcl);

        std::vector<cv::Point2d> imagePts1 = getProjectedPts(line_1_pcl);
        std::vector<cv::Point2d> imagePts2 = getProjectedPts(line_2_pcl);

        cv::Vec3f line1 = getEqnOfLine(cv::Vec4f(line1_start.x, line1_start.y, line1_end.x, line1_end.y));
        cv::Vec3f line2 = getEqnOfLine(cv::Vec4f(line2_start.x, line2_start.y, line2_end.x, line2_end.y));

        double distance;
        for(int i = 0; i < imagePts1.size(); i++){
            distance = distanceFromLine(line1, imagePts1[i]);
            cv::circle(image_in, imagePts1[i], 5, cv::Scalar(171, 171, 0), -1, 1, 0);
        }
        ROS_INFO_STREAM("Avg distance Line 1: " << distance/imagePts1.size());

        distance = 0;
        for(int i = 0; i < imagePts2.size(); i++){
            distance = distanceFromLine(line2, imagePts2[i]);
            cv::circle(image_in, imagePts2[i], 5, cv::Scalar(171, 171, 0), -1, 1, 0);
        }
        ROS_INFO_STREAM("Avg distance Line 2: " << distance/imagePts2.size());

        cv::imshow("image view", image_in);
        cv::waitKey(30);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_projection_node");
    projectionLidarLines pLL;
    ros::spin();
    return 0;
}
