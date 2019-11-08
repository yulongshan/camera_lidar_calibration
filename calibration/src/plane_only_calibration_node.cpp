#include <algorithm>
#include <random>
#include <chrono>
#include <ctime>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include "normal_msg/normal.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <calibration_error_term.h>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "ceres/rotation.h"
#include "ceres/covariance.h"

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <fstream>
#include <iostream>

struct dataFrame {
    pcl::PointCloud<pcl::PointXYZ> lidar_pts;
    Eigen::Vector3d normal;
    double noise;
};

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
                normal_msg::normal,
                normal_msg::normal> SyncPolicy;

class calib {
private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *chkrbrdplane_sub;
    message_filters::Subscriber<normal_msg::normal> *normal_sub;
    message_filters::Subscriber<normal_msg::normal> *tvec_sub;

    message_filters::Synchronizer<SyncPolicy> *sync;

    int no_of_plane_views, max_no_of_plane_views;

    Eigen::Matrix3d Rotn;
    Eigen::Vector3d axis_angle;
    Eigen::Vector3d translation;
    Eigen::VectorXd R_t;

    std::vector<dataFrame> plane_data;

    std::string result_str;
    bool initializeR;

    Eigen::Vector3d Nc_old;
    ceres::Problem problem_plane;
    ceres::Problem problem_line;

public:
    calib() {

        chkrbrdplane_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/points_plane", 1);
        normal_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/normal_plane", 1);
        tvec_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/tvec_plane", 1);

        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),
                                                               *chkrbrdplane_sub,
                                                               *normal_sub,
                                                               *tvec_sub);
        sync->registerCallback(boost::bind(&calib::callbackPlane, this, _1, _2, _3));

        max_no_of_plane_views = readParam<int>(nh, "max_no_of_plane_views");
        std::string result_str;
        ceres::Problem problem_plane;
        initializeR = readParam<bool>(nh, "initializeR");
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        if(initializeR) {
            Rotn(0, 0) = 0.0; Rotn(0, 1) = -1.0; Rotn(0, 2) = 0.0;
            Rotn(1, 0) = 0.0; Rotn(1, 1) = 0.0; Rotn(1, 2) = -1.0;
            Rotn(2, 0) = 1.0; Rotn(2, 1) = 0.0; Rotn(2, 2) = 0.0;
        } else {
            Rotn = transformation_matrix.block(0, 0, 3, 3);
        }
        ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
        translation = transformation_matrix.block(0, 3, 3, 1);
        R_t = Eigen::VectorXd(6);
        R_t(0) = axis_angle(0);
        R_t(1) = axis_angle(1);
        R_t(2) = axis_angle(2);
        R_t(3) = translation(0);
        R_t(4) = translation(1);
        R_t(5) = translation(2);
        ROS_INFO_STREAM("Intialized R_t = " << R_t);
        problem_plane.AddParameterBlock(R_t.data(), 6);
        result_str = readParam<std::string>(nh, "result_str");
        no_of_plane_views = 0;
        Nc_old = Eigen::Vector3d(0, 0, 0);
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

    void callbackPlane(const sensor_msgs::PointCloud2ConstPtr &plane_msg,
                       const normal_msg::normalConstPtr &norm_msg,
                       const normal_msg::normalConstPtr &tvec_msg) {
        ROS_WARN_STREAM("At Plane Callback");
        if(no_of_plane_views < max_no_of_plane_views) {
            pcl::PointCloud<pcl::PointXYZ> plane_pcl;
            pcl::fromROSMsg(*plane_msg, plane_pcl);

            Eigen::Vector3d r3(norm_msg->a, norm_msg->b, norm_msg->c);
            Eigen::Vector3d c_t_w(tvec_msg->a, tvec_msg->b, tvec_msg->c);
            Eigen::Vector3d Nc = (r3.dot(c_t_w))*r3;

            if(r3.dot(Nc_old) < 0.95) {
                dataFrame plane_datum;
                plane_datum.lidar_pts = plane_pcl;
                plane_datum.normal = Nc;
                plane_datum.noise = tvec_msg->w + norm_msg->w;
                plane_data.push_back(plane_datum);
                ROS_INFO_STREAM("No of plane views: " << ++no_of_plane_views);
                Nc_old = r3;
            }
            checkStatus();
        }
    }

    void addPlaneResidual(pcl::PointCloud<pcl::PointXYZ> lidar_pts,
                          Eigen::Vector3d normal) {
        double pi_sqrt = 1;
        for(int j = 0; j < lidar_pts.points.size(); j++){
            Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                     lidar_pts.points[j].y,
                                     lidar_pts.points[j].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                    (new CalibrationErrorTermPlane(point_3d, normal, 1));
            problem_plane.AddResidualBlock(cost_function, new ceres::ScaledLoss(NULL, pi_sqrt, ceres::TAKE_OWNERSHIP),
                                           R_t.data());
        }
    }

    void solvePlaneOptimization() {
        for(int i = 0; i < plane_data.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[i].lidar_pts;
            Eigen::Vector3d normal = plane_data[i].normal;
            addPlaneResidual(lidar_pts, normal);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem_plane, &summary);
        std::cout << summary.FullReport() << '\n';
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        std::cout << "RPY = " << Rotn.eulerAngles(0, 1, 2)*180/M_PI << std::endl;
        std::cout << "t = " << C_T_L.block(0, 3, 3, 1) << std::endl;
        ROS_WARN_STREAM("Writing the result");
        std::ofstream results;
        results.open(result_str);
        results << C_T_L;
        results.close();
        ROS_WARN_STREAM("Wrote result to: " << result_str);
        ros::shutdown();
    }

    void checkStatus() {
        if(no_of_plane_views >= max_no_of_plane_views) {
            solvePlaneOptimization();
        } else {
            ROS_WARN_STREAM("Collecting Data..");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_only_calib_node");
    calib cL;
    ros::spin();
    return 0;
}

