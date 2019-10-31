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

    std::vector<dataFrame> plane_data;
    std::vector<dataFrame> line1_data;
    std::vector<dataFrame> line2_data;
    std::vector<dataFrame> line3_data;
    std::vector<dataFrame> line4_data;

    std::string result_str;

    bool useLines;
    bool usePlane;
    bool initializeR;
    bool jointSol;
    bool planeFirst;

    Eigen::Vector3d Nc_old;
    ceres::Problem problem;
    ceres::Problem problem_plane;
    ceres::Problem problem_line;

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

        problem.AddParameterBlock(R_t.data(), 6);
        result_str = readParam<std::string>(nh, "result_str");
        no_of_line_views = 0;
        no_of_plane_views = 0;

        useLines = readParam<bool>(nh, "useLines");
        usePlane = readParam<bool>(nh, "usePlane");

        if(!useLines && !usePlane) {
            ROS_ERROR("You have to atleast use Lines or use Plane, shouldn't set both to false");
            ros::shutdown();
        }

        if(useLines) {
            max_no_of_line_views = readParam<int>(nh, "max_no_of_line_views");
        } else {
            max_no_of_line_views = 0;
        }

        if(usePlane) {
            max_no_of_plane_views = readParam<int>(nh, "max_no_of_plane_views");
        } else {
            max_no_of_plane_views = 0;
        }

        Nc_old = Eigen::Vector3d(0, 0, 0);

        if(useLines && usePlane) {
            jointSol = readParam<bool>(nh, "jointSol");
            if(!jointSol)
                planeFirst = readParam<bool>(nh, "planeFirst");
        }
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

    void addGaussianNoise(Eigen::Matrix4d &transformation) {
        std::vector<double> data_rot = {0, 0, 0};
        const double mean_rot = 0.0;
        std::default_random_engine generator_rot;
        generator_rot.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<double> dist(mean_rot, 10);

        // Add Gaussian noise
        for (auto& x : data_rot) {
            x = x + dist(generator_rot);
        }

        double roll = data_rot[0]*M_PI/180;
        double pitch = data_rot[1]*M_PI/180;
        double yaw = data_rot[2]*M_PI/180;

        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        std::vector<double> data_trans = {0, 0, 0};
        const double mean_trans = 0.0;
        std::default_random_engine generator_trans;
        generator_trans.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<double> dist_trans(mean_trans, 0.5);

        // Add Gaussian noise
        for (auto& x : data_trans) {
            x = x + dist_trans(generator_trans);
        }

        Eigen::Vector3d trans;
        trans(0) = data_trans[0];
        trans(1) = data_trans[1];
        trans(2) = data_trans[2];

        Eigen::Matrix4d trans_noise = Eigen::Matrix4d::Identity();
        trans_noise.block(0, 0, 3, 3) = m;
        trans_noise.block(0, 3, 3, 1) = trans;
        transformation = transformation*trans_noise;
    }

    void addPlaneResidual(pcl::PointCloud<pcl::PointXYZ> lidar_pts,
                               Eigen::Vector3d normal,
                               double noise) {
//        double pi_sqrt = 1/sqrt(noise*(double)lidar_pts.size());
//        double pi_sqrt = 1/sqrt((double)lidar_pts.size());
//        double pi_sqrt = 1/sqrt(noise);
        double pi_sqrt = 1;
        ROS_WARN_STREAM("pi_sqrt: " << pi_sqrt);
        std::cout << pi_sqrt << std::endl;
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

    void addLineResidual(pcl::PointCloud<pcl::PointXYZ> lidar_pts,
                              Eigen::Vector3d normal) {
        ceres::LossFunction *loss_function = NULL;
        double pi_sqrt = 1/sqrt((double)lidar_pts.size());
        for(int j = 0; j < lidar_pts.points.size(); j++){
            Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                     lidar_pts.points[j].y,
                                     lidar_pts.points[j].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                    (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
            problem_line.AddResidualBlock(cost_function, loss_function, R_t.data());
        }
    }

    void addPlaneResidualJoint(pcl::PointCloud<pcl::PointXYZ> lidar_pts,
                               Eigen::Vector3d normal) {
        ceres::LossFunction *loss_function = NULL;
        double pi_sqrt = 1/sqrt((double)lidar_pts.size());
        for(int j = 0; j < lidar_pts.points.size(); j++){
            Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                     lidar_pts.points[j].y,
                                     lidar_pts.points[j].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                    (new CalibrationErrorTermPlane(point_3d, normal, pi_sqrt));
            problem.AddResidualBlock(cost_function, loss_function, R_t.data());
        }
    }

    void addLineResidualJoint(pcl::PointCloud<pcl::PointXYZ> lidar_pts,
                              Eigen::Vector3d normal) {
        ceres::LossFunction *loss_function = NULL;
        double pi_sqrt = 1/sqrt((double)lidar_pts.size());
        for(int j = 0; j < lidar_pts.points.size(); j++){
            Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                     lidar_pts.points[j].y,
                                     lidar_pts.points[j].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                    ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                    (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
            problem.AddResidualBlock(cost_function, loss_function, R_t.data());
        }
    }

    void solvePlaneOptimization() {
        for(int i = 0; i < plane_data.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[i].lidar_pts;
            Eigen::Vector3d normal = plane_data[i].normal;
            addPlaneResidual(lidar_pts, normal, plane_data[i].noise);
//            addPlaneResidual(lidar_pts, normal, 1);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem_plane, &summary);
        std::cout << summary.FullReport() << '\n';
    }

    void solveLineOptimization() {
        for(int i = 0; i < line1_data.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[i].lidar_pts;
            Eigen::Vector3d normal = line1_data[i].normal;
            addLineResidual(lidar_pts, normal);
        }
        for(int i = 0; i < line2_data.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[i].lidar_pts;
            Eigen::Vector3d normal = line2_data[i].normal;
            addLineResidual(lidar_pts, normal);
        }
        for(int i = 0; i < line3_data.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[i].lidar_pts;
            Eigen::Vector3d normal = line3_data[i].normal;
            addLineResidual(lidar_pts, normal);
        }
        for(int i = 0; i < line4_data.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[i].lidar_pts;
            Eigen::Vector3d normal = line4_data[i].normal;
            addLineResidual(lidar_pts, normal);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem_line, &summary);
        std::cout << summary.FullReport() << '\n';
    }

    void solveJointOptimizationProb() {
        // Add planar residuals
        if(plane_data.size() > 0) {
            for(int i = 0; i < plane_data.size(); i++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[i].lidar_pts;
                Eigen::Vector3d normal = plane_data[i].normal;
                addPlaneResidualJoint(lidar_pts, normal);
            }
        }

        if(line1_data.size() > 0 &&
           line2_data.size() > 0 &&
           line3_data.size() > 0 &&
           line4_data.size() > 0) {
            for(int i = 0; i < line1_data.size(); i++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[i].lidar_pts;
                Eigen::Vector3d normal = line1_data[i].normal;
                addLineResidualJoint(lidar_pts, normal);
            }
            for(int i = 0; i < line2_data.size(); i++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[i].lidar_pts;
                Eigen::Vector3d normal = line2_data[i].normal;
                addLineResidualJoint(lidar_pts, normal);
            }
            for(int i = 0; i < line3_data.size(); i++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[i].lidar_pts;
                Eigen::Vector3d normal = line3_data[i].normal;
                addLineResidualJoint(lidar_pts, normal);
            }
            for(int i = 0; i < line4_data.size(); i++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[i].lidar_pts;
                Eigen::Vector3d normal = line4_data[i].normal;
                addLineResidualJoint(lidar_pts, normal);
            }
        }

        /// Step 4: Solve it
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.FullReport() << '\n';
    }

    void callbackPlane(const sensor_msgs::PointCloud2ConstPtr &plane_msg,
                       const normal_msg::normalConstPtr &norm_msg,
                       const normal_msg::normalConstPtr &tvec_msg) {
        ROS_WARN_STREAM("At Plane Callback");
        if(usePlane && no_of_plane_views < max_no_of_plane_views) {
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

    void callbackLines(const sensor_msgs::PointCloud2ConstPtr &line1_msg,
                       const sensor_msgs::PointCloud2ConstPtr &line2_msg,
                       const sensor_msgs::PointCloud2ConstPtr &line3_msg,
                       const sensor_msgs::PointCloud2ConstPtr &line4_msg,
                       const normal_msg::normalConstPtr &norm1_msg,
                       const normal_msg::normalConstPtr &norm2_msg,
                       const normal_msg::normalConstPtr &norm3_msg,
                       const normal_msg::normalConstPtr &norm4_msg) {
        ROS_WARN_STREAM("At Line Callback");
        if(useLines && no_of_line_views < max_no_of_line_views) {
            pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
            pcl::fromROSMsg(*line1_msg, line_1_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
            pcl::fromROSMsg(*line2_msg, line_2_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_3_pcl;
            pcl::fromROSMsg(*line3_msg, line_3_pcl);
            pcl::PointCloud<pcl::PointXYZ> line_4_pcl;
            pcl::fromROSMsg(*line4_msg, line_4_pcl);

            ROS_WARN_STREAM("Line 1 Size: " << line_1_pcl.points.size() << "\n"
                         << "Line 2 Size: " << line_2_pcl.points.size() << "\n"
                         << "Line 3 Size: " << line_3_pcl.points.size() << "\n"
                         << "Line 4 Size: " << line_4_pcl.points.size());
//            ceres::LossFunction *loss_function = NULL;
            double no_pts_line1 = line_1_pcl.points.size();
            double no_pts_line2 = line_2_pcl.points.size();
            double no_pts_line3 = line_3_pcl.points.size();
            double no_pts_line4 = line_4_pcl.points.size();
            if (no_pts_line1 >= 2 && no_pts_line2 >= 2 &&
                no_pts_line3 >= 2 && no_pts_line4 >= 2) {
                Eigen::Vector3d normal1 = Eigen::Vector3d(norm1_msg->a,
                                                          norm1_msg->b,
                                                          norm1_msg->c);
                Eigen::Vector3d normal2 = Eigen::Vector3d(norm2_msg->a,
                                                          norm2_msg->b,
                                                          norm2_msg->c);
                Eigen::Vector3d normal3 = Eigen::Vector3d(norm3_msg->a,
                                                          norm3_msg->b,
                                                          norm3_msg->c);
                Eigen::Vector3d normal4 = Eigen::Vector3d(norm4_msg->a,
                                                          norm4_msg->b,
                                                          norm4_msg->c);

                dataFrame line1_datum;
                line1_datum.lidar_pts = line_1_pcl;
                line1_datum.normal = normal1;
                line1_data.push_back(line1_datum);

                dataFrame line2_datum;
                line2_datum.lidar_pts = line_2_pcl;
                line2_datum.normal = normal2;
                line2_data.push_back(line2_datum);

                dataFrame line3_datum;
                line3_datum.lidar_pts = line_3_pcl;
                line3_datum.normal = normal3;
                line3_data.push_back(line3_datum);

                dataFrame line4_datum;
                line4_datum.lidar_pts = line_4_pcl;
                line4_datum.normal = normal4;
                line4_data.push_back(line4_datum);

                ROS_INFO_STREAM("No of line views: " << ++no_of_line_views);
                checkStatus();
            } else {
                ROS_WARN_STREAM("Insufficient points in line");
            }
        }
    }

    void checkStatus() {
        ROS_WARN_STREAM("At Check Status");
        bool lineOnlyCond = !usePlane && useLines && no_of_line_views >= max_no_of_line_views;
        bool planeOnlyCond = usePlane && !useLines && no_of_plane_views >= max_no_of_plane_views;
        bool bothLineAndPlane = usePlane && useLines &&
                                no_of_plane_views >= max_no_of_plane_views &&
                                no_of_line_views >= max_no_of_line_views;

        if(bothLineAndPlane) {
            if(jointSol) {
                ROS_WARN_STREAM("Solving Joint Optimization");
                solveJointOptimizationProb();
            } else {
                ROS_WARN_STREAM("Solving Serially");
                if(planeFirst) {
                    ROS_WARN_STREAM("Solving plane first, line second");
                    solvePlaneOptimization();
                    solveLineOptimization();
                } else {
                    ROS_WARN_STREAM("Solving line first, plane second");
                    solveLineOptimization();
                    solvePlaneOptimization();
                }
            }
            logOutput();
        } else if(lineOnlyCond) {
            ROS_WARN_STREAM("Solving Line Optimization Only");
            solveLineOptimization();
            logOutput();
        } else if(planeOnlyCond) {
            ROS_WARN_STREAM("Solving Plane Optimization Only");
            solvePlaneOptimization();
            logOutput();
        } else {
            ROS_INFO_STREAM("No of line views: " << no_of_line_views);
            ROS_INFO_STREAM("No of plane views: " << no_of_plane_views);
        }
    }

    void logOutput() {
        /// Printing and Storing C_T_L in a file
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "calib_node");
    calib cL;
    ros::spin();
    return 0;
}
