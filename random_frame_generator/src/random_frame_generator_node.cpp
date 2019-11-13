#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <random>
#include <iostream>

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

class frameGenerator {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher cloud_pub_;
    image_transport::Publisher image_left_pub_;
    image_transport::Publisher image_right_pub_;

    std::string image_left_folder_name;
    std::string image_right_folder_name;
    std::string pointcloud_folder_name;

    std::string lidar_output_topic_name;
    std::string image_left_topic_name;
    std::string image_right_topic_name;

    int output_fps;
    bool randomize_frames;

public:
    frameGenerator(): it_(nh_) {
        image_left_folder_name =
                readParam<std::string>(nh_, "image_left_folder_name");
        image_right_folder_name =
                readParam<std::string>(nh_, "image_right_folder_name");
        pointcloud_folder_name =
                readParam<std::string>(nh_, "pointcloud_folder_name");
        lidar_output_topic_name =
                readParam<std::string>(nh_, "lidar_output_topic_name");
        image_left_topic_name =
                readParam<std::string>(nh_, "image_left_topic_name");
        image_right_topic_name =
                readParam<std::string>(nh_, "image_right_topic_name");
        output_fps =
                readParam<int>(nh_, "output_fps");
        cloud_pub_ =
                nh_.advertise<sensor_msgs::PointCloud2>(lidar_output_topic_name, 1);
        image_left_pub_ = it_.advertise(image_left_topic_name, 1);
        image_right_pub_ = it_.advertise(image_right_topic_name, 1);
        randomize_frames = readParam<bool>(nh_, "randomize_frames");
        ROS_INFO_STREAM("Press [ENTER] to continue");
        std::cin.get();
        readData();
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else{
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void readData() {
        std::vector<cv::String> filenamesImgLeft, filenamesImgRight;
        std::vector<cv::String> filenamesPCD;

        cv::glob(image_left_folder_name + "/*.png", filenamesImgLeft);
        cv::glob(image_right_folder_name + "/*.png", filenamesImgRight);
        cv::glob(pointcloud_folder_name + "/*.pcd", filenamesPCD);

        ROS_ASSERT(filenamesImgLeft.size() == filenamesImgRight.size());
        ROS_ASSERT(filenamesPCD.size() == filenamesImgLeft.size());

        int no_of_frames = filenamesImgLeft.size();
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist6(0,
                no_of_frames-1);
        std::vector<int> random_numbers;

        if(randomize_frames) {
            int count = 0;
            while(count < no_of_frames) {
                int random_number = dist6(rng);
                int flag = 0;
                for(int i = 0; i < random_numbers.size(); i++) {
                    if(random_numbers[i] == random_number) {
                        flag = 1;
                        break;
                    }
                }
                if(flag != 1) {
                    random_numbers.push_back(random_number);
                    count ++;
                }
            }
        } else {
            for(int i = 0; i < no_of_frames; i++) {
                random_numbers.push_back(i);
            }
        }

        ROS_WARN_STREAM("RANDOM no size: " << random_numbers.size());
        ros::Rate loop_rate(output_fps);
        int i = 0;
        ros::Time present_time, previous_time;
        while (ros::ok() && i < no_of_frames) {
            present_time = ros::Time::now();
            if (i!=0) {
                double time_taken = present_time.toSec() - previous_time.toSec();
                ROS_INFO_STREAM("Publishing Data at " << 1/time_taken << " [Hz]");
            }
            ros::Time current_time = ros::Time::now();
            int query_integer = random_numbers[i];
            cv::Mat image_left = cv::imread(filenamesImgLeft[query_integer]);
            cv::Mat image_right = cv::imread(filenamesImgRight[query_integer]);
            pcl::PointCloud<PointXYZIr>::Ptr
                    cloud (new pcl::PointCloud<PointXYZIr>);
            pcl::io::loadPCDFile<PointXYZIr>(filenamesPCD[query_integer], *cloud);

            sensor_msgs::ImagePtr msg_left =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                       image_left).toImageMsg();
            sensor_msgs::ImagePtr msg_right =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                       image_right).toImageMsg();
            msg_left->header.stamp = current_time;
            msg_right->header.stamp = current_time;
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(*cloud, cloud_ros);
            cloud_ros.header.frame_id = "velodyne";
            cloud_ros.header.stamp = current_time;

            image_left_pub_.publish(msg_left);
            image_right_pub_.publish(msg_right);
            cloud_pub_.publish(cloud_ros);
            loop_rate.sleep();
            i++;
            previous_time = present_time;
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "frame_generator");
    frameGenerator fG;
    ros::spin();
    return 0;
}
