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

typedef message_filters::sync_policies::ApproximateTime
       <sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        sensor_msgs::PointCloud2,
        normal_msg::normal,
        normal_msg::normal,
        normal_msg::normal,
        normal_msg::normal> SyncPolicy;

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

    message_filters::Synchronizer<SyncPolicy> *sync;

    int no_of_views;

    Eigen::Matrix3d Rotn;
    Eigen::Vector3d axis_angle;
    Eigen::Vector3d translation;
    Eigen::VectorXd R_t;

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
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10),
                *line1_sub, *line2_sub, *line3_sub, *line4_sub,
                *normal1_sub, *normal2_sub, *normal3_sub, *normal4_sub);
        sync->registerCallback(boost::bind(&calib::callback, this, _1, _2, _3, _4
                                                                 , _5, _6, _7, _8));
        no_of_views = 0;

        Rotn = Eigen::Matrix3d::Identity();
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
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr &line1_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line2_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line3_msg,
                  const sensor_msgs::PointCloud2ConstPtr &line4_msg,
                  const normal_msg::normalConstPtr &norm1_msg,
                  const normal_msg::normalConstPtr &norm2_msg,
                  const normal_msg::normalConstPtr &norm3_msg,
                  const normal_msg::normalConstPtr &norm4_msg) {
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
        for(int i = 0; i < line_1_pcl.points.size(); i++) {
            Eigen::Vector3d lidar_point(line_1_pcl.points[i].x,
                                        line_1_pcl.points[i].y,
                                        line_1_pcl.points[i].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTerm, 1, 6>
                    (new CalibrationErrorTerm(lidar_point, normal1));
            problem.AddResidualBlock(cost_function, loss_function, R_t.data());
        }

        Eigen::Vector3d normal2 = Eigen::Vector3d(norm2_msg->a,
                                                  norm2_msg->b,
                                                  norm2_msg->c);
        for(int i = 0; i < line_2_pcl.points.size(); i++) {
            Eigen::Vector3d lidar_point(line_2_pcl.points[i].x,
                                        line_2_pcl.points[i].y,
                                        line_2_pcl.points[i].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTerm, 1, 6>
                    (new CalibrationErrorTerm(lidar_point, normal2));
            problem.AddResidualBlock(cost_function, loss_function, R_t.data());
        }

        Eigen::Vector3d normal3 = Eigen::Vector3d(norm3_msg->a,
                                                  norm3_msg->b,
                                                  norm3_msg->c);
        for(int i = 0; i < line_3_pcl.points.size(); i++) {
            Eigen::Vector3d lidar_point(line_3_pcl.points[i].x,
                                        line_3_pcl.points[i].y,
                                        line_3_pcl.points[i].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTerm, 1, 6>
                    (new CalibrationErrorTerm(lidar_point, normal3));
            problem.AddResidualBlock(cost_function, loss_function, R_t.data());
        }

        Eigen::Vector3d normal4 = Eigen::Vector3d(norm4_msg->a,
                                                  norm4_msg->b,
                                                  norm4_msg->c);
        for(int i = 0; i < line_4_pcl.points.size(); i++) {
            Eigen::Vector3d lidar_point(line_4_pcl.points[i].x,
                                        line_4_pcl.points[i].y,
                                        line_4_pcl.points[i].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTerm, 1, 6>
                    (new CalibrationErrorTerm(lidar_point, normal4));
            problem.AddResidualBlock(cost_function, loss_function, R_t.data());
        }

        // This is temporary, think of an intelligent way
        ROS_INFO_STREAM("Collected view: " << ++no_of_views);
        if(no_of_views > 50) {
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

            std::cout << "RPY = " << Rotn.eulerAngles(0, 1, 2)*180/M_PI << std::endl;
            std::cout << "t = " << C_T_L.block(0, 3, 3, 1) << std::endl;
            ros::shutdown();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "calib_node");
    calib cL;
    ros::spin();
    return 0;
}
