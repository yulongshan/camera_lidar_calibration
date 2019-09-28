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

typedef message_filters::sync_policies::ApproximateTime
       <sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        normal_msg::normal,
        normal_msg::normal,
        normal_msg::normal,
        normal_msg::normal> SyncPolicy1;

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2,
         normal_msg::normal,
         normal_msg::normal> SyncPolicy2;

class calib {
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line1_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line2_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line3_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *line4_sub;
    message_filters::Subscriber<normal_msg::normal> *normal1_sub;
    message_filters::Subscriber<normal_msg::normal> *normal2_sub;
    message_filters::Subscriber<normal_msg::normal> *normal3_sub;
    message_filters::Subscriber<normal_msg::normal> *normal4_sub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *chkrbrdplane_sub;
    message_filters::Subscriber<normal_msg::normal> *normal_sub;
    message_filters::Subscriber<normal_msg::normal> *tvec_sub;

    message_filters::Synchronizer<SyncPolicy1> *sync1;
    message_filters::Synchronizer<SyncPolicy2> *sync2;

    int no_of_line_views, max_no_of_line_views;
    int no_of_plane_views, max_no_of_plane_views;

    Eigen::Matrix3d Rotn;
    Eigen::Vector3d axis_angle;
    Eigen::Vector3d translation;
    Eigen::VectorXd R_t;

    std::string result_str;

    bool useLines;
    bool usePlane;

    Eigen::Vector3d Nc_old;
    ceres::Problem problem;
public:
    calib() {
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
        chkrbrdplane_sub = new
                message_filters::Subscriber
                        <sensor_msgs::PointCloud2>(nh, "/velodyne_points/plane", 1);
        normal1_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/normal1", 1);
        normal2_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/normal2", 1);
        normal3_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/normal3", 1);
        normal4_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/normal4", 1);
        normal_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/normal_plane", 1);
        tvec_sub = new
                message_filters::Subscriber
                        <normal_msg::normal>(nh, "/tvec_plane", 1);

        sync1 = new message_filters::Synchronizer<SyncPolicy1>(SyncPolicy1(10),
                *line1_sub, *line2_sub, *line3_sub, *line4_sub,
                *normal1_sub, *normal2_sub, *normal3_sub, *normal4_sub);
        sync1->registerCallback(boost::bind(&calib::callbackLines, this, _1, _2, _3,
                                                                         _4, _5, _6,
                                                                         _7, _8));

        sync2 = new message_filters::Synchronizer<SyncPolicy2>(SyncPolicy2(10),
                                                               *chkrbrdplane_sub,
                                                               *normal_sub,
                                                               *tvec_sub);
        sync2->registerCallback(boost::bind(&calib::callbackPlane, this, _1, _2, _3));

        Rotn = Eigen::Matrix3d::Zero();
        Rotn(0, 0) = 0;
        Rotn(0, 1) = -1;
        Rotn(0, 2) = 0;
        Rotn(1, 0) = 0;
        Rotn(1, 1) = 0;
        Rotn(1, 2) = -1;
        Rotn(2, 0) = 1;
        Rotn(2, 1) = 0;
        Rotn(2, 2) = 0;
        ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
        translation = Eigen::Vector3d(0,0, 0);
        R_t = Eigen::VectorXd(6);
        R_t(0) = axis_angle(0);
        R_t(1) = axis_angle(1);
        R_t(2) = axis_angle(2);
        R_t(3) = translation(0);
        R_t(4) = translation(1);
        R_t(5) = translation(2);

        problem.AddParameterBlock(R_t.data(), 6);

        result_str = "/home/subodh/catkin_ws/src/camera_lidar_calibration/calibration/result/C_T_L.txt";

        no_of_line_views = 0;
        no_of_plane_views = 0;

        useLines = readParam<bool>(nh, "useLines");
        usePlane = readParam<bool>(nh, "usePlane");

        if(!useLines && !usePlane) {
            ROS_ERROR("You have to atlease useLines or usePlane, shouldn't set both to false");
            ros::shutdown();
        }

        if(useLines) {
            max_no_of_line_views = readParam<int>(nh, "max_no_of_line_views");
        }

        if(usePlane) {
            max_no_of_plane_views = readParam<int>(nh, "max_no_of_plane_views");
        }

        Nc_old = Eigen::Vector3d(0, 0, 0);
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name)
    {
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
        if(usePlane && no_of_plane_views < max_no_of_plane_views) {
            pcl::PointCloud<pcl::PointXYZ> plane_pcl;
            pcl::fromROSMsg(*plane_msg, plane_pcl);

            Eigen::Vector3d r3(norm_msg->a, norm_msg->b, norm_msg->c);
            Eigen::Vector3d c_t_w(tvec_msg->a, tvec_msg->b, tvec_msg->c);
            Eigen::Vector3d Nc = (r3.dot(c_t_w))*r3;
            ceres::LossFunction *loss_function = NULL;
            if(r3.dot(Nc_old) < 0.95) {
                double p_i_sqrt = 1/sqrt((double)plane_pcl.points.size());
                for(int i = 0; i < plane_pcl.points.size(); i++) {
                    Eigen::Vector3d lidar_point(plane_pcl.points[i].x,
                                                plane_pcl.points[i].y,
                                                plane_pcl.points[i].z);
                    // Add residual here
                    ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                            (new CalibrationErrorTermPlane(lidar_point, Nc, p_i_sqrt));
                    problem.AddResidualBlock(cost_function, loss_function, R_t.data());
                }
                ROS_INFO_STREAM("No of plane views: " << ++no_of_plane_views);
                Nc_old = r3;
            }
        }
        checkStatus();
    }

    void callbackLines(const sensor_msgs::PointCloud2ConstPtr &line1_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line2_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line3_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line4_msg,
                  const normal_msg::normalConstPtr &norm1_msg,
                  const normal_msg::normalConstPtr &norm2_msg,
                  const normal_msg::normalConstPtr &norm3_msg,
                  const normal_msg::normalConstPtr &norm4_msg) {
        if(useLines && no_of_line_views < max_no_of_line_views) {
            pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
            pcl::fromROSMsg(*line1_msg, line_1_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
            pcl::fromROSMsg(*line2_msg, line_2_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_3_pcl;
            pcl::fromROSMsg(*line3_msg, line_3_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_4_pcl;
            pcl::fromROSMsg(*line4_msg, line_4_pcl);

            ceres::LossFunction *loss_function = NULL;

            Eigen::Vector3d normal1 = Eigen::Vector3d(norm1_msg->a,
                                                      norm1_msg->b,
                                                      norm1_msg->c);
            double p_i_sqrt_1 = 1/sqrt((double)line_1_pcl.points.size());
            for(int i = 0; i < line_1_pcl.points.size(); i++) {
                Eigen::Vector3d lidar_point(line_1_pcl.points[i].x,
                                            line_1_pcl.points[i].y,
                                            line_1_pcl.points[i].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(lidar_point, normal1, p_i_sqrt_1));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }

            Eigen::Vector3d normal2 = Eigen::Vector3d(norm2_msg->a,
                                                      norm2_msg->b,
                                                      norm2_msg->c);
            double p_i_sqrt_2 = 1/sqrt((double)line_2_pcl.points.size());
            for(int i = 0; i < line_2_pcl.points.size(); i++) {
                Eigen::Vector3d lidar_point(line_2_pcl.points[i].x,
                                            line_2_pcl.points[i].y,
                                            line_2_pcl.points[i].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(lidar_point, normal2, p_i_sqrt_2));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }

            Eigen::Vector3d normal3 = Eigen::Vector3d(norm3_msg->a,
                                                      norm3_msg->b,
                                                      norm3_msg->c);
            double p_i_sqrt_3 = 1/sqrt((double)line_3_pcl.points.size());
            for(int i = 0; i < line_3_pcl.points.size(); i++) {
                Eigen::Vector3d lidar_point(line_3_pcl.points[i].x,
                                            line_3_pcl.points[i].y,
                                            line_3_pcl.points[i].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(lidar_point, normal3, p_i_sqrt_3));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }

            Eigen::Vector3d normal4 = Eigen::Vector3d(norm4_msg->a,
                                                      norm4_msg->b,
                                                      norm4_msg->c);
            double p_i_sqrt_4 = 1/sqrt((double)line_4_pcl.points.size());
            for(int i = 0; i < line_4_pcl.points.size(); i++) {
                Eigen::Vector3d lidar_point(line_4_pcl.points[i].x,
                                            line_4_pcl.points[i].y,
                                            line_4_pcl.points[i].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(lidar_point, normal4, p_i_sqrt_4));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
            ROS_INFO_STREAM("No of line views: " << ++no_of_line_views);
        }
        checkStatus();
    }

    void checkStatus() {
        bool lineOnlyCond = !usePlane && useLines && no_of_line_views >= max_no_of_line_views;
        bool planeOnlyCond = usePlane && !useLines && no_of_plane_views >= max_no_of_plane_views;
        bool bothLineAndPlane = usePlane && useLines &&
                                no_of_plane_views >= max_no_of_plane_views &&
                                no_of_line_views >= max_no_of_line_views;

        if(lineOnlyCond || planeOnlyCond || bothLineAndPlane) {
            solveOptimizationProb();
        } else {
            ROS_INFO_STREAM("No of line views: " << no_of_line_views);
            ROS_INFO_STREAM("No of plane views: " << no_of_plane_views);
        }
    }
    void solveOptimizationProb() {
        /// Step 4: Solve it
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << '\n';
        /// Printing and Storing C_T_L in a file
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        std::cout << "RPY = " << Rotn.eulerAngles(0, 1, 2)*180/M_PI << std::endl;
        std::cout << "t = " << C_T_L.block(0, 3, 3, 1) << std::endl;
        std::ofstream results;
        results.open(result_str);
        results << C_T_L;
        results.close();
        ros::shutdown();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "calib_node");
    calib cL;
    ros::spin();
    return 0;
}
