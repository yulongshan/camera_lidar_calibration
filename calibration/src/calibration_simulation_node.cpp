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
#include <pcl/common/eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/extract_indices.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "line_msg/line.h"

#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>

struct dataFrame {
    pcl::PointCloud<pcl::PointXYZ> lidar_pts;
    Eigen::Vector3d normal;
    Eigen::Vector3d tvec;
    double noise;
};

class CSVRow
{
public:
    std::string const& operator[](std::size_t index) const
    {
        return m_data[index];
    }
    std::size_t size() const
    {
        return m_data.size();
    }
    void readNextRow(std::istream& str)
    {
        std::string         line;
        std::getline(str, line);

        std::stringstream   lineStream(line);
        std::string         cell;

        m_data.clear();
        while(std::getline(lineStream, cell, ','))
        {
            m_data.push_back(cell);
        }
        // This checks for a trailing comma with no data after it.
        if (!lineStream && cell.empty())
        {
            // If there was a trailing comma then add an empty element.
            m_data.push_back("");
        }
    }
private:
    std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}

class CSVIterator
{
public:
    typedef std::input_iterator_tag     iterator_category;
    typedef CSVRow                      value_type;
    typedef std::size_t                 difference_type;
    typedef CSVRow*                     pointer;
    typedef CSVRow&                     reference;

    CSVIterator(std::istream& str)  :m_str(str.good()?&str:NULL) { ++(*this); }
    CSVIterator()                   :m_str(NULL) {}

    // Pre Increment
    CSVIterator& operator++()               {if (m_str) { if (!((*m_str) >> m_row)){m_str = NULL;}}return *this;}
    // Post increment
    CSVIterator operator++(int)             {CSVIterator    tmp(*this);++(*this);return tmp;}
    CSVRow const& operator*()   const       {return m_row;}
    CSVRow const* operator->()  const       {return &m_row;}

    bool operator==(CSVIterator const& rhs) {return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));}
    bool operator!=(CSVIterator const& rhs) {return !((*this) == rhs);}
private:
    std::istream*       m_str;
    CSVRow              m_row;
};


class calib {
private:
    ros::NodeHandle nh;

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

    std::string result_str;

    bool useLines;
    bool usePlane;
    bool jointSol;
    std::string base_file_name;
    Eigen::Vector3d r3_old;
    double plane_selection_threshold;

    std::string node_name;
    bool initializeR;
    int no_of_plane_views;
    int no_of_line_views;
public:
    calib(ros::NodeHandle n) {
        nh = n;
        node_name = ros::this_node::getName();
        usePlane = readParam<bool>(nh, "usePlane");
        useLines = readParam<bool>(nh, "useLines");
        jointSol = readParam<bool>(nh, "jointSol");
        initializeR = readParam<bool>(nh, "initializeR");
        no_of_plane_views = readParam<int>(nh, "no_of_plane_views");
        no_of_line_views = readParam<int>(nh, "no_of_line_views");
        base_file_name = readParam<std::string>(nh, "base_file_name");

        // Get Plane Data
        for(int i = 0; i < no_of_plane_views; i++) {
            std::string plane_pts_lidar = base_file_name + "/plane_pts_lidar" +std::to_string(i+1)+ ".csv";
            std::string r3tvec = base_file_name + "/r3tvec" + std::to_string(i+1) + ".csv";
            std::ifstream file_plane_pts_lidar(plane_pts_lidar);
            std::ifstream file_r3tvec(r3tvec);

            pcl::PointCloud<pcl::PointXYZ> planar_points;
            for(CSVIterator loop(file_plane_pts_lidar); loop != CSVIterator(); ++loop){
                pcl::PointXYZ pt;
                pt.x = (float)std::atof((*loop)[0].c_str());
                pt.y = (float)std::atof((*loop)[1].c_str());
                pt.z = (float)std::atof((*loop)[2].c_str());
                planar_points.points.push_back(pt);
            }

            Eigen::Vector3d r3;
            Eigen::Vector3d tvec;
            for(CSVIterator loop(file_r3tvec); loop != CSVIterator(); ++loop){
                double r3_x = std::atof((*loop)[0].c_str());
                double r3_y = std::atof((*loop)[1].c_str());
                double r3_z = std::atof((*loop)[2].c_str());
                double tvec_x = std::atof((*loop)[3].c_str());
                double tvec_y = std::atof((*loop)[4].c_str());
                double tvec_z = std::atof((*loop)[5].c_str());
                r3 = Eigen::Vector3d(r3_x, r3_y, r3_z);
                tvec = Eigen::Vector3d(tvec_x, tvec_y, tvec_z);
            }
            dataFrame planar_datum;
            planar_datum.lidar_pts = planar_points;
            planar_datum.tvec = tvec;
            planar_datum.normal = r3;
            plane_data.push_back(planar_datum);
        }

        // Get Line Data
        for (int i = 0; i < no_of_line_views; i++) {
            std::string all_normals = base_file_name + "/all_normals" +std::to_string(i+1)+ ".csv";
            std::string l1_l = base_file_name + "/l1_l" + std::to_string(i+1) + ".csv";
            std::string l2_l = base_file_name + "/l2_l" + std::to_string(i+1) + ".csv";
            std::string l3_l = base_file_name + "/l3_l" + std::to_string(i+1) + ".csv";
            std::string l4_l = base_file_name + "/l4_l" + std::to_string(i+1) + ".csv";

            std::ifstream file_all_normals(all_normals);
            std::ifstream file_l1_l(l1_l);
            std::ifstream file_l2_l(l2_l);
            std::ifstream file_l3_l(l3_l);
            std::ifstream file_l4_l(l4_l);

            pcl::PointCloud<pcl::PointXYZ> line1_points;
            for(CSVIterator loop(file_l1_l); loop != CSVIterator(); ++loop){
                pcl::PointXYZ pt;
                pt.x = (float)std::atof((*loop)[0].c_str());
                pt.y = (float)std::atof((*loop)[1].c_str());
                pt.z = (float)std::atof((*loop)[2].c_str());
//                std::cout << "Line 1 = " << pt.x << " " << pt.y << " " << pt.z << std::endl;
                line1_points.points.push_back(pt);
            }
            pcl::PointCloud<pcl::PointXYZ> line2_points;
            for(CSVIterator loop(file_l2_l); loop != CSVIterator(); ++loop){
                pcl::PointXYZ pt;
                pt.x = (float)std::atof((*loop)[0].c_str());
                pt.y = (float)std::atof((*loop)[1].c_str());
                pt.z = (float)std::atof((*loop)[2].c_str());
//                std::cout << "Line 2 = " << pt.x << " " << pt.y << " " << pt.z << std::endl;
                line2_points.points.push_back(pt);
            }
            pcl::PointCloud<pcl::PointXYZ> line3_points;
            for(CSVIterator loop(file_l3_l); loop != CSVIterator(); ++loop){
                pcl::PointXYZ pt;
                pt.x = (float)std::atof((*loop)[0].c_str());
                pt.y = (float)std::atof((*loop)[1].c_str());
                pt.z = (float)std::atof((*loop)[2].c_str());
//                std::cout << "Line 3 = " << pt.x << " " << pt.y << " " << pt.z << std::endl;
                line3_points.points.push_back(pt);
            }
            pcl::PointCloud<pcl::PointXYZ> line4_points;
            for(CSVIterator loop(file_l4_l); loop != CSVIterator(); ++loop){
                pcl::PointXYZ pt;
                pt.x = (float)std::atof((*loop)[0].c_str());
                pt.y = (float)std::atof((*loop)[1].c_str());
                pt.z = (float)std::atof((*loop)[2].c_str());
//                std::cout << "Line 4 = " << pt.x << " " << pt.y << " " << pt.z << std::endl;
                line4_points.points.push_back(pt);
            }
            std::vector<Eigen::Vector3d> all_normals_vec;
            for(CSVIterator loop(file_all_normals); loop != CSVIterator(); ++loop) {
                double x = std::atof((*loop)[0].c_str());
                double y = std::atof((*loop)[1].c_str());
                double z = std::atof((*loop)[2].c_str());
                all_normals_vec.push_back(Eigen::Vector3d(x, y, z));
            }

            dataFrame line1_datum;
            line1_datum.lidar_pts = line1_points;
            line1_datum.normal = all_normals_vec[0];
//            std::cout << all_normals_vec[0].transpose() << std::endl;
            line1_data.push_back(line1_datum);

            dataFrame line2_datum;
            line2_datum.lidar_pts = line2_points;
            line2_datum.normal = all_normals_vec[1];
//            std::cout << all_normals_vec[1].transpose() << std::endl;
            line2_data.push_back(line2_datum);

            dataFrame line3_datum;
            line3_datum.lidar_pts = line3_points;
            line3_datum.normal = all_normals_vec[2];
//            std::cout << all_normals_vec[2].transpose() << std::endl;
            line3_data.push_back(line3_datum);

            dataFrame line4_datum;
            line4_datum.lidar_pts = line4_points;
            line4_datum.normal = all_normals_vec[3];
//            std::cout << all_normals_vec[3].transpose() << std::endl;
            line4_data.push_back(line4_datum);
        }

        ROS_ASSERT(plane_data.size() == no_of_plane_views);
        ROS_ASSERT(line1_data.size() == line2_data.size());
        ROS_ASSERT(line2_data.size() == line3_data.size());
        ROS_ASSERT(line3_data.size() == line4_data.size());
        ROS_ASSERT(line4_data.size() == no_of_line_views);

        if(usePlane && !useLines)
            solvePlaneOptimization();
        else if(!usePlane && useLines)
            solveLineOptimization();
        else if(usePlane && useLines)
            if(jointSol)
                solvePlaneAndLine();
            else
                solvePlaneThenLine();
        else {
            ROS_ERROR_STREAM("You should atleast choose one between Plane or Line Constraint");
            ros::shutdown();
        }
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name) {
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM("[ "<< node_name << " ]: " << " Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void solvePlaneOptimization() {
        ROS_WARN_STREAM("Solving Plane Optimization");
        time_t tstart, tend;
        tstart = time(0);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
        Rotn = transformation_matrix.block(0, 0, 3, 3);
        if(initializeR) {
            Rotn(0, 0) = 0;
            Rotn(1, 1) = 0;
            Rotn(2, 2) = 0;
            Rotn(0, 1) = -1;
            Rotn(1, 2) = -1;
            Rotn(2, 0) = 1;
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

        R_t_init = R_t;

        ceres::Problem problem;

        problem.AddParameterBlock(R_t.data(), 6);
        ceres::LossFunction *loss_function = NULL;
        for(int k = 0; k < plane_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
            Eigen::Vector3d r3 = plane_data[k].normal;
            Eigen::Vector3d tvec = plane_data[k].tvec;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                            lidar_pts.points[j].y,
                                             lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                            (new CalibrationErrorTermPlane(point_3d, r3, tvec, pi_sqrt));
                problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        tend = time(0);
        ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Time taken for iteration:"<< difftime(tend, tstart) << " [s]\n");
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        ros::shutdown();
    }

    void solveLineOptimization() {
        ROS_WARN_STREAM("Solving Line Optimization");
        time_t tstart, tend;
        tstart = time(0);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
        Rotn = transformation_matrix.block(0, 0, 3, 3);
        if(initializeR) {
            Rotn(0, 0) = 0;
            Rotn(1, 1) = 0;
            Rotn(2, 2) = 0;
            Rotn(0, 1) = -1;
            Rotn(1, 2) = -1;
            Rotn(2, 0) = 1;
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
        options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        tend = time(0);
        ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Time taken for iteration: "<< difftime(tend, tstart) << " [s]\n");
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        ros::shutdown();
    }

    void solvePlaneThenLine() {
        ROS_WARN_STREAM("Solving Plane then Line Optimization");
        time_t tstart, tend;
        tstart = time(0);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
        Rotn = transformation_matrix.block(0, 0, 3, 3);
        if(initializeR) {
            Rotn(0, 0) = 0;
            Rotn(1, 1) = 0;
            Rotn(2, 2) = 0;
            Rotn(0, 1) = -1;
            Rotn(1, 2) = -1;
            Rotn(2, 0) = 1;
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

        R_t_init = R_t;

        ceres::LossFunction *loss_function = NULL;

        ceres::Problem problem1;
        problem1.AddParameterBlock(R_t.data(), 6);
        for(int k = 0; k < plane_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
            Eigen::Vector3d r3 = plane_data[k].normal;
            Eigen::Vector3d tvec = plane_data[k].tvec;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                            ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                            (new CalibrationErrorTermPlane(point_3d, r3, tvec, pi_sqrt));
                problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        ceres::Solver::Options options1;
        options1.minimizer_type = ceres::MinimizerType::TRUST_REGION;
        options1.max_num_iterations = 200;
        options1.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options1.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary;
        ceres::Solve(options1, &problem1, &summary);
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        ceres::Problem problem2;

        problem2.AddParameterBlock(R_t.data(), 6);
        for(int k = 0; k < line1_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
            Eigen::Vector3d normal = line1_data[k].normal;
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
        options2.minimizer_type = ceres::MinimizerType::TRUST_REGION;
        options2.max_num_iterations = 200;
        options2.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options2.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary2;
        ceres::Solve(options2, &problem2, &summary2);
        tend = time(0);
        ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Time taken for iteration: " << difftime(tend, tstart) << " [s]\n");
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        ros::shutdown();
    }

    void solvePlaneAndLine() {
        ROS_WARN_STREAM("Solving Plane + Line Optimization");
        time_t tstart, tend;
        tstart = time(0);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
//            addGaussianNoise(transformation_matrix);
        Rotn = transformation_matrix.block(0, 0, 3, 3);
        if(initializeR) {
            Rotn(0, 0) = 0;
            Rotn(1, 1) = 0;
            Rotn(2, 2) = 0;
            Rotn(0, 1) = -1;
            Rotn(1, 2) = -1;
            Rotn(2, 0) = 1;
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

        R_t_init = R_t;

        ceres::LossFunction *loss_function = NULL;

        ceres::Problem problem1;
        problem1.AddParameterBlock(R_t.data(), 6);
        for(int k = 0; k < plane_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = plane_data[k].lidar_pts;
            Eigen::Vector3d r3 = plane_data[k].normal;
            Eigen::Vector3d tvec = plane_data[k].tvec;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermPlane, 1, 6>
                        (new CalibrationErrorTermPlane(point_3d, r3, tvec, pi_sqrt));
                problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        problem1.AddParameterBlock(R_t.data(), 6);
        for(int k = 0; k < line1_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line1_data[k].lidar_pts;
            Eigen::Vector3d normal = line1_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        for(int k = 0; k < line2_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line2_data[k].lidar_pts;
            Eigen::Vector3d normal = line2_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        for(int k = 0; k < line3_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line3_data[k].lidar_pts;
            Eigen::Vector3d normal = line3_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }

        for(int k = 0; k < line4_data.size(); k++) {
            pcl::PointCloud<pcl::PointXYZ> lidar_pts = line4_data[k].lidar_pts;
            Eigen::Vector3d normal = line4_data[k].normal;
            double pi_sqrt = 1/sqrt((double)lidar_pts.size());
            for(int j = 0; j < lidar_pts.points.size(); j++){
                Eigen::Vector3d point_3d(lidar_pts.points[j].x,
                                         lidar_pts.points[j].y,
                                         lidar_pts.points[j].z);
                // Add residual here
                ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<CalibrationErrorTermLine, 1, 6>
                        (new CalibrationErrorTermLine(point_3d, normal, pi_sqrt));
                problem1.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
        }
        ceres::Solver::Options options1;
        options1.minimizer_type = ceres::MinimizerType::TRUST_REGION;
        options1.max_num_iterations = 200;
        options1.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options1.minimizer_progress_to_stdout = true;

        ceres::Solver::Summary summary1;
        ceres::Solve(options1, &problem1, &summary1);
        tend = time(0);
        ROS_INFO_STREAM("[ "<< node_name << " ]: " << " Time taken for iteration: " << difftime(tend, tstart) << " [s]\n");
        ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
        Eigen::MatrixXd C_T_L(3, 4);
        C_T_L.block(0, 0, 3, 3) = Rotn;
        C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
        std::cout << C_T_L << std::endl;
        ros::shutdown();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "calib_simulation_node");
    ros::NodeHandle nh("~");
    calib cL(nh);
    ros::spin();
    return 0;
}
