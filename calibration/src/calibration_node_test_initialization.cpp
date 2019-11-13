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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "line_msg/line.h"

#include <fstream>
#include <iostream>

struct dataFrame {
    pcl::PointCloud<pcl::PointXYZ> lidar_pts;
    Eigen::Vector3d normal;
    double noise;
};

struct cloudAndImageLine {
    pcl::PointCloud<pcl::PointXYZ> line_cloud1;
    pcl::PointCloud<pcl::PointXYZ> line_cloud2;
    pcl::PointCloud<pcl::PointXYZ> line_cloud3;
    pcl::PointCloud<pcl::PointXYZ> line_cloud4;
    cv::Vec3f line_image1;
    cv::Vec3f line_image2;
    cv::Vec3f line_image3;
    cv::Vec3f line_image4;
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


typedef message_filters::sync_policies::ApproximateTime
        <line_msg::line,
         line_msg::line,
         line_msg::line,
         line_msg::line,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2,
         sensor_msgs::PointCloud2> SyncPolicy3;

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
    message_filters::Subscriber<line_msg::line> *line1_image_sub;
    message_filters::Subscriber<line_msg::line> *line2_image_sub;
    message_filters::Subscriber<line_msg::line> *line3_image_sub;
    message_filters::Subscriber<line_msg::line> *line4_image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *chkrbrdplane_sub;
    message_filters::Subscriber<normal_msg::normal> *normal_sub;
    message_filters::Subscriber<normal_msg::normal> *tvec_sub;

    message_filters::Synchronizer<SyncPolicy1> *sync1;
    message_filters::Synchronizer<SyncPolicy2> *sync2;
    message_filters::Synchronizer<SyncPolicy3> *sync3;

    int no_of_line_views, max_no_of_line_views;
    int no_of_plane_views, max_no_of_plane_views;

    Eigen::Matrix3d Rotn;
    Eigen::Vector3d axis_angle;
    Eigen::Vector3d translation;
    Eigen::VectorXd R_t;
    Eigen::VectorXd R_t_init;

    std::vector<dataFrame> plane_data;
    std::vector<dataFrame> line1_data;
    std::vector<dataFrame> line2_data;
    std::vector<dataFrame> line3_data;
    std::vector<dataFrame> line4_data;

    std::vector<cloudAndImageLine> lidar_img_line_data;

    std::string result_str;
    std::string cam_config_file_path;

    cv::Mat D, K;
    int image_width, image_height;

    bool useLines;
    bool usePlane;
    bool initializeR;
    bool jointSol;
    bool planeFirst;

    Eigen::Vector3d Nc_old;

    std::string initializations_file;
    std::string results_file;

    double fov_x, fov_y;

    std::ofstream init_file;
    std::ofstream res_file;

    int no_of_diff_initializations;

    bool generate_debug_data;
    std::string debug_data_basefilename;
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

        line1_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image1", 1);
        line2_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image2", 1);
        line3_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image3", 1);
        line4_image_sub = new
                message_filters::Subscriber
                        <line_msg::line>(nh, "/line_image4", 1);

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


        sync3 = new message_filters::Synchronizer
                <SyncPolicy3>(SyncPolicy3(10),
                             *line1_image_sub,
                             *line2_image_sub,
                             *line3_image_sub,
                             *line4_image_sub,
                             *line1_sub,
                             *line2_sub,
                             *line3_sub,
                             *line4_sub);
        sync3->registerCallback(boost::bind(&calib::callbackImageAndLidarLines, this, _1, _2, _3, _4,
                                                                                      _5, _6, _7, _8));
        Rotn = Eigen::Matrix3d::Zero();

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

        initializations_file = readParam<std::string>(nh, "initializations_file");
        results_file = readParam<std::string>(nh, "results_file");
        no_of_diff_initializations = readParam<int>(nh, "no_of_diff_initializations");
        generate_debug_data = readParam<bool>(nh, "generate_debug_data");
        debug_data_basefilename = readParam<std::string>(nh, "debug_data_basefilename");
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

    std::vector<cv::Point2d> getProjectedPts(pcl::PointCloud<pcl::PointXYZ> cloud_in, Eigen::MatrixXd C_T_L) {
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
        Eigen::Matrix3d C_R_L = C_T_L.block(0, 0, 3, 3);
        Eigen::Vector3d C_t_L = C_T_L.block(0, 3, 3, 1);

        cv::Mat rvec, tvec, c_R_l;
        cv::eigen2cv(C_R_L, c_R_l);
        cv::Rodrigues(c_R_l, rvec);
        cv::eigen2cv(C_t_L, tvec);
        if(objectPoints_L.size() > 0)
            cv::projectPoints(objectPoints_L, rvec, tvec, K, D, imagePoints, cv::noArray());
        else
            ROS_ERROR("objectPoints_L.size() <= 0");
        return imagePoints;
    }

    double distanceFromLine(cv::Vec3f eqn, cv::Point2f pt) {
        float a = eqn(0);
        float b = eqn(1);
        float c = eqn(2);
        float x_0 = pt.x;
        float y_0 = pt.y;
        double dist = fabs(a*x_0+b*y_0+c)/sqrt(a*a+b*b);
    }

    double getReprojectionError(Eigen::MatrixXd C_T_L) {
        double dist_avg = 0;
        for (int i = 0; i < lidar_img_line_data.size(); i++) {
            pcl::PointCloud<pcl::PointXYZ> line_1_pcl = lidar_img_line_data[i].line_cloud1;
            pcl::PointCloud<pcl::PointXYZ> line_2_pcl = lidar_img_line_data[i].line_cloud2;
            pcl::PointCloud<pcl::PointXYZ> line_3_pcl = lidar_img_line_data[i].line_cloud3;
            pcl::PointCloud<pcl::PointXYZ> line_4_pcl = lidar_img_line_data[i].line_cloud4;
            cv::Vec3f line1 = lidar_img_line_data[i].line_image1;
            cv::Vec3f line2 = lidar_img_line_data[i].line_image2;
            cv::Vec3f line3 = lidar_img_line_data[i].line_image3;
            cv::Vec3f line4 = lidar_img_line_data[i].line_image4;

            std::vector<cv::Point2d> imagePts1 = getProjectedPts(line_1_pcl, C_T_L);
            std::vector<cv::Point2d> imagePts2 = getProjectedPts(line_2_pcl, C_T_L);
            std::vector<cv::Point2d> imagePts3 = getProjectedPts(line_3_pcl, C_T_L);
            std::vector<cv::Point2d> imagePts4 = getProjectedPts(line_4_pcl, C_T_L);

            double distance1 = 0;
            for(int i = 0; i < imagePts1.size(); i++){
                distance1 += distanceFromLine(line1, imagePts1[i]);
            }
            distance1 = distance1/imagePts1.size();

            double distance2 = 0;
            for(int i = 0; i < imagePts2.size(); i++){
                distance2 += distanceFromLine(line2, imagePts2[i]);
            }
            distance2 = distance2/imagePts2.size();

            double distance3 = 0;
            for(int i = 0; i < imagePts3.size(); i++){
                distance3 += distanceFromLine(line3, imagePts3[i]);
            }
            distance3 = distance3/imagePts3.size();

            double distance4 = 0;
            for(int i = 0; i < imagePts4.size(); i++){
                distance4 += distanceFromLine(line4, imagePts4[i]);
            }
            distance4 = distance4/imagePts4.size();

            dist_avg += (distance1 + distance2 + distance3 + distance4)/4;
        }
        return dist_avg/lidar_img_line_data.size();
    }

    void solvePlaneOptimization() {
        init_file.open(initializations_file);
        res_file.open(results_file);
        for (int i = 0; i < no_of_diff_initializations; i++) {
            time_t tstart, tend;
            tstart = time(0);
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            addGaussianNoise(transformation_matrix);
            Rotn = transformation_matrix.block(0, 0, 3, 3);
            ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
            translation = transformation_matrix.block(0, 3, 3, 1);
            R_t = Eigen::VectorXd(6);
            R_t(0) = axis_angle(0);
            R_t(1) = axis_angle(1);
            R_t(2) = axis_angle(2);
            R_t(3) = translation(0);
            R_t(4) = translation(1);
            R_t(5) = translation(2);

            R_t_init = R_t;

            ceres::Problem problem;

            problem.AddParameterBlock(R_t.data(), 6);
            ceres::LossFunction *loss_function = NULL;
            for(int k = 0; k < plane_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
                Eigen::Vector3d normal = plane_data[k].normal;
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
            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            tend = time(0);
            ROS_INFO_STREAM("Time taken for iteration: " <<  i << " is "<< difftime(tend, tstart) << " [s]\n");
            ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
            Eigen::MatrixXd C_T_L(3, 4);
            C_T_L.block(0, 0, 3, 3) = Rotn;
            C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
            double reprojection_error = getReprojectionError(C_T_L);
            init_file << R_t_init(0) << "," << R_t_init(1) << "," << R_t_init(2) << ","
                      << R_t_init(3) << "," << R_t_init(4) << "," << R_t_init(5) << "\n";
            res_file << R_t(0) << "," << R_t(1) << "," << R_t(2) << ","
                     << R_t(3) << "," << R_t(4) << "," << R_t(5) << "," << reprojection_error << "\n";
            ROS_WARN_STREAM("Line reprojerror: " << reprojection_error);
        }
        init_file.close();
        res_file.close();
        ros::shutdown();
    }

    void solveLineOptimization() {
        init_file.open(initializations_file);
        res_file.open(results_file);
        for (int i = 0; i < no_of_diff_initializations; i++) {
            time_t tstart, tend;
            tstart = time(0);
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            addGaussianNoise(transformation_matrix);
            Rotn = transformation_matrix.block(0, 0, 3, 3);
            ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
            translation = transformation_matrix.block(0, 3, 3, 1);
            R_t = Eigen::VectorXd(6);
            R_t(0) = axis_angle(0);
            R_t(1) = axis_angle(1);
            R_t(2) = axis_angle(2);
            R_t(3) = translation(0);
            R_t(4) = translation(1);
            R_t(5) = translation(2);

            R_t_init = R_t;

            ceres::Problem problem;

            problem.AddParameterBlock(R_t.data(), 6);
            for(int k = 0; k < line1_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
                Eigen::Vector3d normal = line1_data[k].normal;
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
            for(int k = 0; k < line2_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[k].lidar_pts;
                Eigen::Vector3d normal = line2_data[k].normal;
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
            for(int k = 0; k < line3_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[k].lidar_pts;
                Eigen::Vector3d normal = line3_data[k].normal;
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
            for(int k = 0; k < line4_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[k].lidar_pts;
                Eigen::Vector3d normal = line4_data[k].normal;
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
            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            tend = time(0);
            ROS_INFO_STREAM("Time taken for iteration: " <<  i << " is "<< difftime(tend, tstart) << " [s]\n");
            ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
            Eigen::MatrixXd C_T_L(3, 4);
            C_T_L.block(0, 0, 3, 3) = Rotn;
            C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
            double reprojection_error = getReprojectionError(C_T_L);
            init_file << R_t_init(0) << "," << R_t_init(1) << "," << R_t_init(2) << ","
                      << R_t_init(3) << "," << R_t_init(4) << "," << R_t_init(5) << "\n";
            res_file << R_t(0) << "," << R_t(1) << "," << R_t(2) << ","
                     << R_t(3) << "," << R_t(4) << "," << R_t(5) << "," << reprojection_error << "\n";
            ROS_WARN_STREAM("Line reprojerror: " << reprojection_error);
        }
        init_file.close();
        res_file.close();
        ros::shutdown();
    }

    void solvePlaneAndLineJointly() {
        init_file.open(initializations_file);
        res_file.open(results_file);
        for (int i = 0; i < no_of_diff_initializations; i++) {
            time_t tstart, tend;
            tstart = time(0);
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            addGaussianNoise(transformation_matrix);
            Rotn = transformation_matrix.block(0, 0, 3, 3);

            ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
            translation = transformation_matrix.block(0, 3, 3, 1);

            R_t = Eigen::VectorXd(6);
            R_t(0) = axis_angle(0);
            R_t(1) = axis_angle(1);
            R_t(2) = axis_angle(2);
            R_t(3) = translation(0);
            R_t(4) = translation(1);
            R_t(5) = translation(2);

            R_t_init = R_t;

            ceres::Problem problem;

            problem.AddParameterBlock(R_t.data(), 6);

            for(int k = 0; k < plane_data.size(); k++) {
                ceres::LossFunction *loss_function = NULL;
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
                Eigen::Vector3d normal = plane_data[k].normal;
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

            for(int k = 0; k < line1_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
                Eigen::Vector3d normal = line1_data[k].normal;
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

            for(int k = 0; k < line2_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[k].lidar_pts;
                Eigen::Vector3d normal = line2_data[k].normal;
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

            for(int k = 0; k < line3_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[k].lidar_pts;
                Eigen::Vector3d normal = line3_data[k].normal;
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

            for(int k = 0; k < line4_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[k].lidar_pts;
                Eigen::Vector3d normal = line4_data[k].normal;
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

            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            tend = time(0);
            ROS_INFO_STREAM("Time taken for iteration: " <<  i << " is "<< difftime(tend, tstart) << " [s]\n");
            ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
            Eigen::MatrixXd C_T_L(3, 4);
            C_T_L.block(0, 0, 3, 3) = Rotn;
            C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
            double reprojection_error = getReprojectionError(C_T_L);
            init_file << R_t_init(0) << "," << R_t_init(1) << "," << R_t_init(2) << ","
                      << R_t_init(3) << "," << R_t_init(4) << "," << R_t_init(5) << "\n";
            res_file << R_t(0) << "," << R_t(1) << "," << R_t(2) << ","
                     << R_t(3) << "," << R_t(4) << "," << R_t(5) << "," << reprojection_error << "\n";
            ROS_WARN_STREAM("Line reprojerror: " << reprojection_error);
        }
        init_file.close();
        res_file.close();
        ros::shutdown();
    }

    void solvePlaneThenLine() {
        init_file.open(initializations_file);
        res_file.open(results_file);
        for(int i = 0; i < no_of_diff_initializations; i++) {
            time_t tstart, tend;
            tstart = time(0);
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            addGaussianNoise(transformation_matrix);
            Rotn = transformation_matrix.block(0, 0, 3, 3);

            ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());
            translation = transformation_matrix.block(0, 3, 3, 1);

            R_t = Eigen::VectorXd(6);
            R_t(0) = axis_angle(0);
            R_t(1) = axis_angle(1);
            R_t(2) = axis_angle(2);
            R_t(3) = translation(0);
            R_t(4) = translation(1);
            R_t(5) = translation(2);

            R_t_init = R_t;

            ceres::Problem problem1;
            problem1.AddParameterBlock(R_t.data(), 6);
            for(int k = 0; k < plane_data.size(); k++) {
                ceres::LossFunction *loss_function = NULL;
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
                Eigen::Vector3d normal = plane_data[k].normal;
                double pi_sqrt = 1/sqrt((double)lidar_pts.size());
                for(int j = 0; j < lidar_pts.points.size(); j++){
                    Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                             lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                    // Add residual here
                    ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                            (new CalibrationErrorTermPlane(point_3d, normal, pi_sqrt));
                    problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
                }
            }
            ceres::Solver::Options options1;
            options1.max_num_iterations = 200;
            options1.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options1.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options1, &problem1, &summary);

            ceres::Problem problem2;

            problem2.AddParameterBlock(R_t.data(), 6);
            for(int k = 0; k < line1_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
                Eigen::Vector3d normal = line1_data[k].normal;
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
                    problem2.AddResidualBlock(cost_function, loss_function, R_t.data());
                }
            }

            for(int k = 0; k < line2_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[k].lidar_pts;
                Eigen::Vector3d normal = line2_data[k].normal;
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
                    problem2.AddResidualBlock(cost_function, loss_function, R_t.data());
                }
            }

            for(int k = 0; k < line3_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[k].lidar_pts;
                Eigen::Vector3d normal = line3_data[k].normal;
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
                    problem2.AddResidualBlock(cost_function, loss_function, R_t.data());
                }
            }

            for(int k = 0; k < line4_data.size(); k++) {
                pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[k].lidar_pts;
                Eigen::Vector3d normal = line4_data[k].normal;
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
                    problem2.AddResidualBlock(cost_function, loss_function, R_t.data());
                }
            }
            ceres::Solver::Options options2;
            options2.max_num_iterations = 200;
            options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options2.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary2;
            ceres::Solve(options2, &problem2, &summary2);
            tend = time(0);
            ROS_INFO_STREAM("Time taken for iteration: " <<  i << " is "<< difftime(tend, tstart) << " [s]\n");
            ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
            Eigen::MatrixXd C_T_L(3, 4);
            C_T_L.block(0, 0, 3, 3) = Rotn;
            C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
            double reprojection_error = getReprojectionError(C_T_L);
            init_file << R_t_init(0) << "," << R_t_init(1) << "," << R_t_init(2) << ","
                      << R_t_init(3) << "," << R_t_init(4) << "," << R_t_init(5) << "\n";
            res_file << R_t(0) << "," << R_t(1) << "," << R_t(2) << ","
                     << R_t(3) << "," << R_t(4) << "," << R_t(5) << "," << reprojection_error << "\n";
            ROS_WARN_STREAM("Line reprojerror: " << reprojection_error);
        }
        init_file.close();
        res_file.close();
        ros::shutdown();
    }

    void generateCSVFile(std::string filename,
                         pcl::PointCloud<pcl::PointXYZ> cloud_data_pcl) {
        std::ofstream csv_file;
        csv_file.open(filename);
        for(int i = 0; i < cloud_data_pcl.points.size(); i++) {
            double X = cloud_data_pcl.points[i].x;
            double Y = cloud_data_pcl.points[i].y;
            double Z = cloud_data_pcl.points[i].z;
            csv_file << X << "," << Y << "," << Z << "\n";
        }
        csv_file.close();

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
                if (generate_debug_data) {
                    std::string file_name = "/plane/lidar/lidar_plane_view"
                            +std::to_string(no_of_plane_views)+".csv";
                    std::string lidar_plane_file_name = debug_data_basefilename+file_name;
                    generateCSVFile(lidar_plane_file_name, plane_pcl);
                }
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

                if(generate_debug_data) {
                    std::string lidar_line1_file_name = debug_data_basefilename +
                            "/lines/lidar/line1_"
                            +std::to_string(no_of_line_views)+".csv";
                    generateCSVFile(lidar_line1_file_name, line_1_pcl);
                    std::string lidar_line2_file_name = debug_data_basefilename +
                            "/lines/lidar/line2_"
                            +std::to_string(no_of_line_views)+".csv";
                    generateCSVFile(lidar_line2_file_name, line_2_pcl);
                    std::string lidar_line3_file_name = debug_data_basefilename +
                            "/lines/lidar/line3_"
                            +std::to_string(no_of_line_views)+".csv";
                    generateCSVFile(lidar_line3_file_name, line_3_pcl);
                    std::string lidar_line4_file_name = debug_data_basefilename +
                            "/lines/lidar/line4_"
                            +std::to_string(no_of_line_views)+".csv";
                    generateCSVFile(lidar_line4_file_name, line_4_pcl);
                }
                ROS_INFO_STREAM("No of line views: " << ++no_of_line_views);
                checkStatus();
            } else {
                ROS_WARN_STREAM("Insufficient points in line");
            }
        }
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

    void callbackImageAndLidarLines(const line_msg::lineConstPtr& line1_img_msg,
                                    const line_msg::lineConstPtr& line2_img_msg,
                                    const line_msg::lineConstPtr& line3_img_msg,
                                    const line_msg::lineConstPtr& line4_img_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& line1_cloud_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& line2_cloud_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& line3_cloud_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& line4_cloud_msg) {
        ROS_WARN_STREAM("Callback for image and lidar lines");
        cv::Point2f line1_start = cv::Point2f(line1_img_msg->a1, line1_img_msg->b1);
        cv::Point2f line1_end = cv::Point2f(line1_img_msg->a2, line1_img_msg->b2);
        cv::Point2f line2_start = cv::Point2f(line2_img_msg->a1, line2_img_msg->b1);
        cv::Point2f line2_end = cv::Point2f(line2_img_msg->a2, line2_img_msg->b2);
        cv::Point2f line3_start = cv::Point2f(line3_img_msg->a1, line3_img_msg->b1);
        cv::Point2f line3_end = cv::Point2f(line3_img_msg->a2, line3_img_msg->b2);
        cv::Point2f line4_start = cv::Point2f(line4_img_msg->a1, line4_img_msg->b1);
        cv::Point2f line4_end = cv::Point2f(line4_img_msg->a2, line4_img_msg->b2);

        pcl::PointCloud<pcl::PointXYZ> line_1_pcl;
        pcl::fromROSMsg(*line1_cloud_msg, line_1_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_2_pcl;
        pcl::fromROSMsg(*line2_cloud_msg, line_2_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_3_pcl;
        pcl::fromROSMsg(*line3_cloud_msg, line_3_pcl);
        pcl::PointCloud<pcl::PointXYZ> line_4_pcl;
        pcl::fromROSMsg(*line4_cloud_msg, line_4_pcl);

        cv::Vec3f line1 = getEqnOfLine(cv::Vec4f(line1_start.x, line1_start.y, line1_end.x, line1_end.y));
        cv::Vec3f line2 = getEqnOfLine(cv::Vec4f(line2_start.x, line2_start.y, line2_end.x, line2_end.y));
        cv::Vec3f line3 = getEqnOfLine(cv::Vec4f(line3_start.x, line3_start.y, line3_end.x, line3_end.y));
        cv::Vec3f line4 = getEqnOfLine(cv::Vec4f(line4_start.x, line4_start.y, line4_end.x, line4_end.y));

        cloudAndImageLine data;
        data.line_cloud1 = line_1_pcl;
        data.line_cloud2 = line_2_pcl;
        data.line_cloud3 = line_3_pcl;
        data.line_cloud4 = line_4_pcl;
        data.line_image1 = line1;
        data.line_image2 = line2;
        data.line_image3 = line3;
        data.line_image4 = line4;

        lidar_img_line_data.push_back(data);
        ROS_WARN_STREAM("No of lidar image line pars collected: " << lidar_img_line_data.size());
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
                solvePlaneAndLineJointly();
            } else {
                ROS_WARN_STREAM("Solving Serially");
                solvePlaneThenLine();
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
            ROS_INFO_STREAM("No of lidar image line pairs collected: " << lidar_img_line_data.size());
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

