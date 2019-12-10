#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Eigen>

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


typedef message_filters::sync_policies::ApproximateTime
        <normal_msg::normal,
         normal_msg::normal> SyncPolicy;

class motionDetector {
private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<normal_msg::normal> *tvec_sub;
    message_filters::Subscriber<normal_msg::normal> *rvec_sub;
    message_filters::Synchronizer<SyncPolicy> *sync;

    Eigen::Matrix4d T_eig_old;
    bool first_frame;
    Eigen::Matrix4d delta_T;

    std::vector<Eigen::VectorXd> pose3D_queue;

public:
    motionDetector(ros::NodeHandle _nh) {
        nh_ = _nh;
        tvec_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh_, "/tvec_in", 1);
        rvec_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh_, "/rvec_in", 1);
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),
                                                               *tvec_sub,
                                                               *rvec_sub);
        sync->registerCallback(boost::bind(&motionDetector::callback, this, _1, _2));

        first_frame = true;
        delta_T = T_eig_old = Eigen::Matrix4d::Identity();
    }

    void getStandardDev(std::vector<double> v) {
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
        std::cout << "std: " << stdev << std::endl;
    }

    void callback(const normal_msg::normalConstPtr &tvec_msg,
                  const normal_msg::normalConstPtr &rvec_msg) {
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
        if(pose3D_queue.size() == 10) {
            std::vector<double> px;
            std::vector<double> py;
            std::vector<double> pz;
            std::vector<double> rx;
            std::vector<double> ry;
            std::vector<double> rz;
            for(int i = 0; i < 10; i++) {
                Eigen::VectorXd pose3D = pose3D_queue[i];
                std::cout << pose3D.transpose() << std::endl;
                px.push_back(pose3D(0));
                py.push_back(pose3D(1));
                pz.push_back(pose3D(2));
                rx.push_back(pose3D(3));
                ry.push_back(pose3D(4));
                rz.push_back(pose3D(5));
            }
            getStandardDev(px);
            getStandardDev(py);
            getStandardDev(pz);
            getStandardDev(rx);
            getStandardDev(ry);
            getStandardDev(rz);
            pose3D_queue.erase(pose3D_queue.begin());
            std::cout << std::endl;
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_detector_node");
    ros::NodeHandle nh("~");
    motionDetector mD(nh);
    ros::spin();
}