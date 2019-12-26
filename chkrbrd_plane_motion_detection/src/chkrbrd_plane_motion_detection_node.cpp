#include <iostream>
#include <sstream>
#include <fstream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <velodyne_pointcloud/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
        sensor_msgs::Image> SyncPolicy;

class checkerboardMotionDetector {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub;
    message_filters::Synchronizer<SyncPolicy> *sync;

    ros::Publisher cloud_plane_pub;

    std::string node_name;

    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    int min_pts;
    double ransac_threshold;

    std::string cam_config_file_path;
    int image_width, image_height;

    cv::Mat projection_matrix;
    cv::Mat distCoeff;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;

    std_msgs::Header header_info;

    double dx, dy;
    int checkerboard_rows, checkerboard_cols;
    cv::Mat tvec_cv, rvec_cv;
    int buffer_size;

    std::vector<Eigen::Vector3d> rot3D_queue;
    bool state; // true: at motion
    bool old_state;

    int no_of_rest_frames;

    std::vector<cv::Mat> rest_images;
    std::vector<sensor_msgs::PointCloud2> rest_lidar_data;
    std::string data_folder_name;
    std::string image_folder_name;
    std::string lidar_folder_name;

public:
    checkerboardMotionDetector(ros::NodeHandle n) {
        nh = n;
        node_name = ros::this_node::getName();
        cloud_sub =  new
                message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/lidar_in_topic", 1);
        image_sub = new
                message_filters::Subscriber<sensor_msgs::Image>(nh, "/camera_in_topic", 1);

        cloud_plane_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_plane_out", 1);

        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *cloud_sub, *image_sub);
        sync->registerCallback(boost::bind(&checkerboardMotionDetector::callback, this, _1, _2));

        projection_matrix = cv::Mat::zeros(3, 3, CV_64F);
        distCoeff = cv::Mat::zeros(5, 1, CV_64F);

        x_min = readParam<double>(nh, "x_min");
        x_max = readParam<double>(nh, "x_max");
        y_min = readParam<double>(nh, "y_min");
        y_max = readParam<double>(nh, "y_max");
        z_min = readParam<double>(nh, "z_min");
        z_max = readParam<double>(nh, "z_max");
        min_pts = readParam<int>(nh, "min_pts");

        dx = readParam<double>(nh, "dx");
        dy = readParam<double>(nh, "dy");
        checkerboard_rows = readParam<int>(nh, "checkerboard_rows");
        checkerboard_cols = readParam<int>(nh, "checkerboard_cols");

        ransac_threshold = readParam<double>(nh, "ransac_threshold");
        cam_config_file_path = readParam<std::string>(nh, "cam_config_file_path");
        readCameraParams(cam_config_file_path,
                         image_height,
                         image_width,
                         distCoeff,
                         projection_matrix);
        ROS_INFO_STREAM("[ " <<node_name<< " ]: " << " distCoeff: \n" << distCoeff);
        ROS_INFO_STREAM("[ " <<node_name<< " ]: " << " projection_matrix: \n" << projection_matrix);

        tvec_cv = cv::Mat::zeros(3, 1, CV_64F);
        rvec_cv = cv::Mat::zeros(3, 1, CV_64F);
        buffer_size = readParam<int>(nh, "buffer_size");

        for(int i = 0; i < checkerboard_rows; i++)
            for (int j = 0; j < checkerboard_cols; j++)
                object_points.emplace_back(cv::Point3f(i*dx, j*dy, 0.0));

        // Assume that we start at rest
        state = false;
        old_state = false;
        no_of_rest_frames = 0;

        data_folder_name = readParam<std::string>(nh, "data_folder_name");
        image_folder_name = data_folder_name+"/images/";
        lidar_folder_name = data_folder_name+"/lidar_data/";
        boost::filesystem::remove_all(image_folder_name);
        boost::filesystem::create_directory(image_folder_name);
        boost::filesystem::remove_all(lidar_folder_name);
        boost::filesystem::create_directory(lidar_folder_name);
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("[ " <<node_name<< " ]: " << " Loaded " << name << ": " << ans);
        } else {
            ROS_ERROR_STREAM("[ " <<node_name<< " ]: " << " Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void readCameraParams(std::string cam_config_file_path,
                          int &image_height,
                          int &image_width,
                          cv::Mat &D,
                          cv::Mat &K) {
        cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
        if(!fs_cam_config.isOpened())
            std::cerr << "Error: Wrong path: " << cam_config_file_path << std::endl;
        fs_cam_config["image_height"] >> image_height;
        fs_cam_config["image_width"] >> image_width;
        fs_cam_config["k1"] >> D.at<double>(0);
        fs_cam_config["k2"] >> D.at<double>(1);
        fs_cam_config["p1"] >> D.at<double>(2);
        fs_cam_config["p2"] >> D.at<double>(3);
        fs_cam_config["k3"] >> D.at<double>(4);
        fs_cam_config["fx"] >> K.at<double>(0, 0);
        fs_cam_config["fy"] >> K.at<double>(1, 1);
        fs_cam_config["cx"] >> K.at<double>(0, 2);
        fs_cam_config["cy"] >> K.at<double>(1, 2);
    }

    pcl::PointCloud<pcl::PointXYZ> passThruFilterCld(pcl::PointCloud<pcl::PointXYZ> cloud_in) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_in_ptr = cloud_in;
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                plane(new pcl::PointCloud<pcl::PointXYZ>);

        // Pass Thru Filters
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_in_ptr);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min, x_max);
        pass_x.filter(*cloud_filtered_x);
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_filtered_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min, y_max);
        pass_y.filter(*cloud_filtered_xy);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_y.setInputCloud(cloud_filtered_xy);
        pass_y.setFilterFieldName("z");
        pass_y.setFilterLimits(z_min, z_max);
        pass_y.filter(*cloud_filtered_xyz);

        return *cloud_filtered_xyz;
    }

    void publishCloud(pcl::PointCloud<pcl::PointXYZ> cloud_in, ros::Publisher cloud_pub) {
        sensor_msgs::PointCloud2 cloud_out_ros;
        pcl::toROSMsg(cloud_in, cloud_out_ros);
        cloud_out_ros.header.stamp = header_info.stamp;
        cloud_out_ros.header.frame_id = header_info.frame_id;
        cloud_pub.publish(cloud_out_ros);
    }

    bool detectedPlaneinLidar(pcl::PointCloud<pcl::PointXYZ> cloud_in) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_in_ptr = cloud_in;
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                plane(new pcl::PointCloud<pcl::PointXYZ>);
        // Plane Segmentation
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
                model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_in_ptr));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(ransac_threshold);
        ransac.computeModel();
        std::vector<int> inlier_indices;
        ransac.getInliers(inlier_indices);
        int no_of_inliers = inlier_indices.size();
        ROS_INFO_STREAM("[ " <<node_name<< " ]: " << "No of inliers: " << no_of_inliers);
        if( no_of_inliers > min_pts) {
            ROS_INFO_STREAM("[ " <<node_name<< " ]: " << " No of inliers pts in LiDAR: " << inlier_indices.size());
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in_ptr, inlier_indices, *plane);
            publishCloud(*plane, cloud_plane_pub);
            return true;
        } else {
            return false;
        }
    }

    double getStandardDev(std::vector<double> v) {
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
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

    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                  const sensor_msgs::ImageConstPtr &image_msg) {
        header_info.stamp = cloud_msg->header.stamp;
        header_info.frame_id = cloud_msg->header.frame_id;

        pcl::PointCloud<pcl::PointXYZ> cloud_in_pcl;
        pcl::fromROSMsg(*cloud_msg, cloud_in_pcl);
        pcl::PointCloud<pcl::PointXYZ> cloud_in_passthru_filtered = passThruFilterCld(cloud_in_pcl);
        bool detected_plane_in_lidar = detectedPlaneinLidar(cloud_in_passthru_filtered);
        bool detected_plane_in_camera = false;
        cv::Mat image_in;
        try {
            image_in = cv_bridge::toCvShare(image_msg, "bgr8")->image;
            detected_plane_in_camera = cv::findChessboardCorners(image_in,
                                                           cv::Size(checkerboard_cols, checkerboard_rows),
                                                           image_points,
                                                           cv::CALIB_CB_ADAPTIVE_THRESH+
                                                           cv::CALIB_CB_NORMALIZE_IMAGE);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                      image_msg->encoding.c_str());
        }
        if(detected_plane_in_lidar && detected_plane_in_camera) {
            ROS_INFO_STREAM("[ " <<node_name<< " ]: " << "Board detected both in LiDAR and Camera");
            if(image_points.size() == object_points.size()){
                cv::solvePnP(object_points, image_points, projection_matrix, distCoeff, rvec_cv, tvec_cv, false, CV_ITERATIVE);
                Eigen::Vector3d tvec = Eigen::Vector3d(tvec_cv.at<double>(0),
                                                       tvec_cv.at<double>(1),
                                                       tvec_cv.at<double>(2));
                cv::Mat R_cv;
                cv::Rodrigues(rvec_cv, R_cv);
                Eigen::Matrix3d R_eig;
                cv::cv2eigen(R_cv, R_eig);

                Eigen::Vector3d rvec_eig;
                rvec_eig(0) = rvec_cv.at<double>(0);
                rvec_eig(1) = rvec_cv.at<double>(1);
                rvec_eig(2) = rvec_cv.at<double>(2);
                rot3D_queue.push_back(rvec_eig);
                if(rot3D_queue.size() == buffer_size) {
                    std::vector<double> rx;
                    std::vector<double> ry;
                    std::vector<double> rz;
                    for(int i = 0; i < buffer_size; i++) {
                        Eigen::Vector3d pose3D = rot3D_queue[i];
                        rx.push_back(pose3D(0));
                        ry.push_back(pose3D(1));
                        rz.push_back(pose3D(2));
                    }
                    double std_rx = getStandardDev(rx);
                    double std_ry = getStandardDev(ry);
                    double std_rz = getStandardDev(rz);
                    bool rot_cond = std_rx <= 0.01 && std_ry <= 0.01 && std_rz <= 0.01;
                    if (rot_cond)
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
                    rot3D_queue.erase(rot3D_queue.begin());
                }
                old_state = state;
                ROS_INFO_STREAM("No of still frames: " << no_of_rest_frames);
            }
        } else {
            ROS_WARN_STREAM("[ " <<node_name<< " ]: " << "Board not detected in one or both sensors");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "chkrbrd_plane_motion_detection_node");
    ros::NodeHandle nh("~");
    checkerboardMotionDetector cMD(nh);
    ros::spin();
    return 0;
}

