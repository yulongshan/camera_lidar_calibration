#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Eigen>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "normal_msg/normal.h"

#include <boost/filesystem.hpp>

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
        <normal_msg::normal,
         normal_msg::normal,
         sensor_msgs::Image,
         sensor_msgs::PointCloud2> SyncPolicy;

class motionDetector {
private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<normal_msg::normal> *tvec_sub;
    message_filters::Subscriber<normal_msg::normal> *rvec_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;

    std::vector<Eigen::VectorXd> pose3D_queue;
    bool state; // true: at motion
    bool old_state;

    std::vector<cv::Mat> rest_images;
    std::vector<sensor_msgs::PointCloud2> rest_lidar_data;
    std::string data_folder_name;
    std::string image_folder_name;
    std::string lidar_folder_name;
    std::string camera_name;
    std::string lidar_name;
    int no_of_rest_frames;
    int buffer_size;

public:
    motionDetector(ros::NodeHandle _nh) {
        nh_ = _nh;
        tvec_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh_, "/tvec_in", 1);
        rvec_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh_, "/rvec_in", 1);
        image_sub = new
                message_filters::Subscriber
                        <sensor_msgs::Image>(nh_, "/image_in", 1);
        cloud_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh_, "/cloud_in", 1);

        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),
                                                               *tvec_sub,
                                                               *rvec_sub,
                                                               *image_sub,
                                                               *cloud_sub);

        sync->registerCallback(boost::bind(&motionDetector::callback, this, _1, _2, _3, _4));

        data_folder_name = readParam<std::string>(nh_, "data_folder_name");
        image_folder_name = data_folder_name+"/images/";
        lidar_folder_name = data_folder_name+"/lidar_data/";
        boost::filesystem::remove_all(image_folder_name);
        boost::filesystem::create_directory(image_folder_name);
        boost::filesystem::remove_all(lidar_folder_name);
        boost::filesystem::create_directory(lidar_folder_name);
        buffer_size = readParam<int>(nh_, "buffer_size");

        // Assume that we start at rest
        state = false;
        old_state = false;
        no_of_rest_frames = 0;
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

    double getStandardDev(std::vector<double> v) {
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
//        std::cout << "std: " << stdev << std::endl;
        return stdev;
    }

    void storeFrames() {
        ROS_INFO_STREAM("Storing Motionless Frames");
        int frame_no = 0;
        int no_of_frames = rest_images.size();
        ROS_ASSERT(no_of_frames > 0);
        frame_no = no_of_frames/2;
        std::string left_image_name = "/frame" + std::to_string(no_of_rest_frames) + ".png";
        std::string pointcloud_name = "/pointcloud" + std::to_string(no_of_rest_frames) + ".pcd";
        pcl::io::savePCDFile(lidar_folder_name+pointcloud_name,
                             rest_lidar_data[frame_no], Eigen::Vector4f::Zero (),
                             Eigen::Quaternionf::Identity (), true);
        no_of_rest_frames++;
        imwrite(image_folder_name+left_image_name, rest_images[frame_no]);
        ROS_INFO_STREAM("Stored Motionless Frames");
    }

    void callback(const normal_msg::normalConstPtr &tvec_msg,
                  const normal_msg::normalConstPtr &rvec_msg,
                  const sensor_msgs::ImageConstPtr &image_msg,
                  const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        cv::Mat image_in;
        try {
            image_in = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
        }

        Eigen::Vector3d tvec = Eigen::Vector3d(tvec_msg->a, tvec_msg->b, tvec_msg->c);
        cv::Mat rvec = cv::Mat::zeros(cv::Size(3, 1), CV_64F);
        rvec.at<double>(0) = rvec_msg->a;
        rvec.at<double>(1) = rvec_msg->b;
        rvec.at<double>(2) = rvec_msg->c;

        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        Eigen::Matrix3d R_eig;
        cv::cv2eigen(R_cv, R_eig);
        Eigen::Vector3d rpy = R_eig.eulerAngles(0, 1, 2)*180/M_PI;

        Eigen::VectorXd xyz_rpy(6);
        xyz_rpy(0) = tvec(0);
        xyz_rpy(1) = tvec(1);
        xyz_rpy(2) = tvec(2);
        xyz_rpy(3) = rvec_msg->a;
        xyz_rpy(4) = rvec_msg->b;
        xyz_rpy(5) = rvec_msg->c;

        pose3D_queue.push_back(xyz_rpy);
        if(pose3D_queue.size() == buffer_size) {
            std::vector<double> px;
            std::vector<double> py;
            std::vector<double> pz;
            std::vector<double> rx;
            std::vector<double> ry;
            std::vector<double> rz;
            for(int i = 0; i < buffer_size; i++) {
                Eigen::VectorXd pose3D = pose3D_queue[i];
//                std::cout << pose3D.transpose() << std::endl;
                px.push_back(pose3D(0));
                py.push_back(pose3D(1));
                pz.push_back(pose3D(2));
                rx.push_back(pose3D(3));
                ry.push_back(pose3D(4));
                rz.push_back(pose3D(5));
            }
            double std_px = getStandardDev(px);
            double std_py = getStandardDev(py);
            double std_pz = getStandardDev(pz);
            double std_rx = getStandardDev(rx);
            double std_ry = getStandardDev(ry);
            double std_rz = getStandardDev(rz);

            bool pos_cond = std_px <= 0.01 && std_py <= 0.01 && std_pz <= 0.01;
            bool rot_cond = std_rx <= 0.01 && std_ry <= 0.01 && std_rz <= 0.01;
            if (pos_cond && rot_cond)
                state = false;
            else
                state = true;

            if (state && !old_state) {
                ROS_WARN_STREAM("State Transition from Rest to Motion");
                ROS_ASSERT(rest_lidar_data.size() == rest_images.size());
                if (rest_lidar_data.size() > 0) {
                    storeFrames();
                    rest_lidar_data.clear();
                    rest_images.clear();
                }
            }

            if (!state && old_state) {
                ROS_WARN_STREAM("State Transition from Motion to Rest");
            }

            if (state) {
                ROS_INFO_STREAM("At Motion");
            }
            else {
                ROS_WARN_STREAM("At Rest");
                rest_lidar_data.push_back(*cloud_msg);
                rest_images.push_back(image_in);
            }
            pose3D_queue.erase(pose3D_queue.begin());
        }
        old_state = state;
        ROS_INFO_STREAM("No of still frames: " << no_of_rest_frames);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_detector_node");
    ros::NodeHandle nh("~");
    motionDetector mD(nh);
    ros::spin();
}