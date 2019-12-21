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

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <fstream>

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

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
         sensor_msgs::Image,
         sensor_msgs::Image> SyncPolicy;

class synchronizeLidarCamera {
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_left_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_right_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;

    std::string data_folder_name;
    std::string left_image_folder_name;
    std::string right_image_folder_name;
    std::string point_cloud_folder_name;
    std::string timestamps_file_name;

    int frame_no;

    std::ofstream timestamps_file;

public:
    synchronizeLidarCamera(ros::NodeHandle n) {
        nh = n;
        cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/cloud_in", 1);
        image_left_sub = new
                message_filters::Subscriber
                        <sensor_msgs::Image>(nh, "/image_left", 1);
        image_right_sub = new
                message_filters::Subscriber
                        <sensor_msgs::Image>(nh, "/image_right", 1);

        sync = new
                message_filters::Synchronizer
                        <SyncPolicy>(SyncPolicy(10),
                                     *cloud_sub,
                                     *image_left_sub,
                                     *image_right_sub);
        sync->registerCallback(boost::bind(&synchronizeLidarCamera::callback,
                                           this, _1, _2, _3));

        data_folder_name = readParam<std::string>(nh, "data_folder_name");
        left_image_folder_name = data_folder_name + "/left";
        right_image_folder_name = data_folder_name + "/right";
        point_cloud_folder_name = data_folder_name + "/pointcloud";
        timestamps_file_name = data_folder_name + "/timestamps.txt";
        frame_no = 0;
        timestamps_file.open(timestamps_file_name);
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name) {
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

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                  const sensor_msgs::ImageConstPtr &image_left_msg,
                  const sensor_msgs::ImageConstPtr &image_right_msg) {
        double current_time = ros::Time::now().toSec();
        cv::Mat image_left_in;
        try {
            image_left_in = cv_bridge::toCvShare(image_left_msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_left_msg->encoding.c_str());
        }

        cv::Mat image_right_in;
        try {
            image_right_in = cv_bridge::toCvShare(image_right_msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_right_msg->encoding.c_str());
        }

        pcl::PointCloud<PointXYZIr> cloud_pcl;
        pcl::fromROSMsg(*cloud_msg, cloud_pcl);


        std::string left_image_name = "/left" + std::to_string(frame_no) + ".png";
        std::string right_image_name = "/right" + std::to_string(frame_no) + ".png";
        std::string pointcloud_name = "/pointcloud" + std::to_string(frame_no) + ".pcd";

        pcl::io::savePCDFile(point_cloud_folder_name+pointcloud_name,
                             *cloud_msg, Eigen::Vector4f::Zero (),
                             Eigen::Quaternionf::Identity (), true);
        imwrite(left_image_folder_name+left_image_name, image_left_in);
        imwrite(right_image_folder_name+right_image_name, image_right_in);
        timestamps_file << std::to_string(current_time) << "\n";
        frame_no++;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "synchronizer_node");
    ros::NodeHandle nh("~");
    synchronizeLidarCamera sLC(nh);
    ros::spin();
    return 0;
}