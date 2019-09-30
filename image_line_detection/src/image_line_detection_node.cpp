#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "normal_msg/normal.h"


//#define FX 6.4372590342756985e+02
//#define FY 6.4372590342756985e+02
//#define CX 3.9534097290039062e+02
//#define CY 3.0199901199340820e+02
//
double fx, fy, cx, cy;
double k1, k2, p1, p2;
int image_width, image_height;
std::string cam_config_file_path;

struct labelledLine {
    cv::Vec4f line;
    double slope;
    char labelX;
    char labelY;
};
std::vector<labelledLine> lls;

ros::Publisher normal_pub_lt;
ros::Publisher normal_pub_rt;
ros::Publisher normal_pub_rb;
ros::Publisher normal_pub_lb;
ros::Publisher normal_pub_chkrbrd;
ros::Publisher tvec_pub_chkrbrd;

std_msgs::Header global_header;

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

void readCameraParams() {
    cv::FileStorage fs_cam_config(cam_config_file_path, cv::FileStorage::READ);
    ROS_ASSERT(fs_cam_config.isOpened());
    fs_cam_config["image_height"] >> image_height;
    fs_cam_config["image_width"] >> image_width;
    fs_cam_config["k1"] >> k1;
    fs_cam_config["k2"] >> k2;
    fs_cam_config["p1"] >> p1;
    fs_cam_config["p2"] >> p2;
    fs_cam_config["fx"] >> fx;
    fs_cam_config["fy"] >> fy;
    fs_cam_config["cx"] >> cx;
    fs_cam_config["cy"] >> cy;
}


cv::Vec3f getEqnOfPlane(cv::Vec3f line) {
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat line_vec = cv::Mat::zeros(3, 1, CV_64FC1);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1.0;
//    std::cout << K << std::endl;
    line_vec.at<double>(0) = line(0);
    line_vec.at<double>(1) = line(1);
    line_vec.at<double>(2) = line(2);
    cv::transpose(K, K);
    cv::Mat normal_c = K*line_vec;
    cv::Vec3f normal_vec(normal_c.at<double>(0),
                         normal_c.at<double>(1),
                         normal_c.at<double>(2));
//    ROS_INFO_STREAM("Normal Equation: " << cv::normalize(normal_vec));
    return cv::normalize(normal_vec);
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

double getAngle(double slope1, double slope2) {
    double angle = atan2(slope1-slope2, 1+slope1*slope2);
    return angle;
}


double getDistance(cv::Vec4f line1, cv::Vec4f line2) {
    cv::Point2f l1_pta = cv::Point2f(line1[0], line1[1]);
    cv::Point2f l1_ptb = cv::Point2f(line1[2], line1[3]);
    cv::Point2f midPoint1 = 0.5*(l1_pta+l1_ptb);

    cv::Point2f l2_pta = cv::Point2f(line2[0], line2[1]);
    cv::Point2f l2_ptb = cv::Point2f(line2[2], line2[3]);
    cv::Point2f midPoint2 = 0.5*(l2_pta+l2_ptb);
    double distance = cv::norm(midPoint1-midPoint2);
    return distance;
}

cv::Point2f getIntersection(cv::Vec4f line_1, cv::Vec4f line_2) {
    cv::Vec3f line1 = getEqnOfLine(line_1);
    cv::Vec3f line2 = getEqnOfLine(line_2);
    cv::Vec3f intersection = line1.cross(line2);
    cv::Point2f pt(intersection[0]/intersection[2],
                   intersection[1]/intersection[2]);
    return pt;
}

std::vector<cv::Point2f> getPose(cv::Point2f pt1,
                                 cv::Point2f pt2,
                                 cv::Point2f pt3,
                                 cv::Point2f pt4) {
    std::vector<cv::Point2f> imagePoints;
    imagePoints.push_back(pt1);
    imagePoints.push_back(pt2);
    imagePoints.push_back(pt3);
    imagePoints.push_back(pt4);

    double side_len = 0.608;
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(0, 0, 0));
    objectPoints.push_back(cv::Point3f(0, side_len, 0));
    objectPoints.push_back(cv::Point3f(side_len, side_len, 0));
    objectPoints.push_back(cv::Point3f(side_len, 0, 0));

    cv::Mat K = cv::Mat::zeros(3, 3, CV_64FC1);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    K.at<double>(2, 2) = 1.0;

    cv::Mat D = cv::Mat::zeros(4, 1, CV_64FC1);

    cv::Mat rvec(3, 1, CV_64FC1);
    cv::Mat tvec(3, 1, CV_64FC1);

    cv::solvePnP(objectPoints, imagePoints, K, D, rvec, tvec);

    std::vector<cv::Point2f> imagePoints_proj;
    cv::projectPoints(objectPoints, rvec, tvec, K, D, imagePoints_proj, cv::noArray(), 0);
    cv::Mat C_R_W;
    cv::Rodrigues(rvec, C_R_W);

    normal_msg::normal n_plane;
    n_plane.header.stamp = global_header.stamp;
    n_plane.a = C_R_W.at<double>(0, 2);
    n_plane.b = C_R_W.at<double>(1, 2);
    n_plane.c = C_R_W.at<double>(2, 2);

    normal_msg::normal tvec_plane;
    tvec_plane.header.stamp = global_header.stamp;
    tvec_plane.a = tvec.at<double>(0);
    tvec_plane.b = tvec.at<double>(1);
    tvec_plane.c = tvec.at<double>(2);
    normal_pub_chkrbrd.publish(n_plane);
    tvec_pub_chkrbrd.publish(tvec_plane);
//    std::cout << tvec << std::endl;
    return imagePoints_proj;
}

void drawLineSegments(cv::Mat image_in) {
    ROS_ASSERT(lls.size() == 4);
    std::vector<double> slopes_ordered(4);
    std::vector<cv::Vec4f> lines_ordered(4);
    normal_msg::normal n_lt;
    normal_msg::normal n_rt;
    normal_msg::normal n_rb;
    normal_msg::normal n_lb;
    for(size_t i = 0; i < 4; i++) {
        char labelX = lls[i].labelX;
        char labelY = lls[i].labelY;
        if(labelX == 'l' && labelY == 't') {
            slopes_ordered[0] = lls[i].slope;
            lines_ordered[0] = lls[i].line;
            cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[0]);
            cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
//            ROS_INFO_STREAM("Normal Equation lt: " << normal_eqn);
            n_lt.header.stamp = global_header.stamp;
            n_lt.a = normal_eqn(0);
            n_lt.b = normal_eqn(1);
            n_lt.c = normal_eqn(2);
            normal_pub_lt.publish(n_lt);
        }

        if(labelX == 'r' && labelY == 't') {
            slopes_ordered[1] = lls[i].slope;
            lines_ordered[1] = lls[i].line;
            cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[1]);
            cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
//            ROS_INFO_STREAM("Normal Equation rt: " << normal_eqn);
            n_rt.header.stamp = global_header.stamp;
            n_rt.a = normal_eqn(0);
            n_rt.b = normal_eqn(1);
            n_rt.c = normal_eqn(2);
            normal_pub_rt.publish(n_rt);
        }

        if(labelX == 'r' && labelY == 'b') {
            slopes_ordered[2] = lls[i].slope;
            lines_ordered[2] = lls[i].line;
            cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[2]);
            cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
//            ROS_INFO_STREAM("Normal Equation rb: " << normal_eqn);
            n_rb.header.stamp = global_header.stamp;
            n_rb.a = normal_eqn(0);
            n_rb.b = normal_eqn(1);
            n_rb.c = normal_eqn(2);
            normal_pub_rb.publish(n_rb);
        }

        if(labelX == 'l' && labelY == 'b') {
            slopes_ordered[3] = lls[i].slope;
            lines_ordered[3] = lls[i].line;
            cv::Vec3f lines_eqn = getEqnOfLine(lines_ordered[3]);
            cv::Vec3f normal_eqn = getEqnOfPlane(lines_eqn);
//            ROS_INFO_STREAM("Normal Equation lb: " << normal_eqn);
            n_lb.header.stamp = global_header.stamp;
            n_lb.a = normal_eqn(0);
            n_lb.b = normal_eqn(1);
            n_lb.c = normal_eqn(2);
            normal_pub_lb.publish(n_lb);
        }
    }
    double angle1 =
            fabs(getAngle(slopes_ordered[0], slopes_ordered[1]))*180/M_PI;
    double angle2 =
            fabs(getAngle(slopes_ordered[1], slopes_ordered[2]))*180/M_PI;
    double angle3 =
            fabs(getAngle(slopes_ordered[2], slopes_ordered[3]))*180/M_PI;
    double angle4 =
            fabs(getAngle(slopes_ordered[3], slopes_ordered[0]))*180/M_PI;

    double dist02 = getDistance(lines_ordered[0], lines_ordered[2]);
    double dist13 = getDistance(lines_ordered[1], lines_ordered[3]);

    if(angle1 > 50 &&
        angle2 > 50 &&
            angle3 > 50 &&
                angle4 > 50 &&
                    dist02 > 150 &&
                        dist13 > 150) {
        for(size_t i = 0; i < 4; i++) {
            cv::Vec4f line_i = lls[i].line;

            cv::Point2f start_pt = cv::Point2f(line_i[0], line_i[1]);
            cv::Point2f end_pt = cv::Point2f(line_i[2], line_i[3]);
            cv::Point2f mid_pt = 0.5*(start_pt + end_pt);

            cv::Scalar line_color = cv::Scalar(0, 0, 0);
            line_color = cv::Scalar(255, 0, 0);
            std::string line_txt;

            char labelX = lls[i].labelX;
            char labelY = lls[i].labelY;

            if(labelX == 'l' && labelY == 't') {
                line_txt = "lt";
                line_color = cv::Scalar(255, 0, 0);
                slopes_ordered[0] = lls[i].slope;
            }

            if(labelX == 'r' && labelY == 't') {
                line_txt = "rt";
                line_color = cv::Scalar(0, 0, 255);
                slopes_ordered[1] = lls[i].slope;
            }

            if(labelX == 'r' && labelY == 'b') {
                line_txt = "rb";
                line_color = cv::Scalar(255, 255, 0);
                slopes_ordered[2] = lls[i].slope;
            }

            if(labelX == 'l' && labelY == 'b') {
                line_txt = "lb";
                line_color = cv::Scalar(0, 255, 0);
                slopes_ordered[3] = lls[i].slope;
            }

            cv::Scalar start_point_color = cv::Scalar(0, 255, 255);
            cv::Scalar end_point_color = cv::Scalar(255, 0, 255);
            cv::line(image_in, start_pt, end_pt, line_color, 2, cv::LINE_8);
            cv::circle(image_in, start_pt, 3,
                       start_point_color, cv::FILLED, cv::LINE_8);
            cv::circle(image_in, end_pt, 3,
                       end_point_color, cv::FILLED, cv::LINE_8);
            cv::putText(image_in,
                        line_txt,
                        mid_pt, cv::FONT_HERSHEY_DUPLEX,
                        1, cv::Scalar(0, 143, 143), 2);
        }

        cv::Point2f pt1 = getIntersection(lines_ordered[0], lines_ordered[1]);
        cv::Point2f pt2 = getIntersection(lines_ordered[1], lines_ordered[2]);
        cv::Point2f pt3 = getIntersection(lines_ordered[2], lines_ordered[3]);
        cv::Point2f pt4 = getIntersection(lines_ordered[3], lines_ordered[0]);
        std::vector<cv::Point2f> re_projected_pts = getPose(pt1, pt2, pt3, pt4);
        cv::circle(image_in, pt1, 7,
                   cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
        cv::circle(image_in, pt2, 7,
                   cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
        cv::circle(image_in, pt3, 7,
                   cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);
        cv::circle(image_in, pt4, 7,
                   cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_8);

        cv::circle(image_in, re_projected_pts[0], 7,
                   cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);
        cv::circle(image_in, re_projected_pts[1], 7,
                   cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_8);
        cv::circle(image_in, re_projected_pts[2], 7,
                   cv::Scalar(255, 0, 0), cv::FILLED, cv::LINE_8);
        cv::circle(image_in, re_projected_pts[3], 7,
                   cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_8);
    }
    cv::imshow("view", image_in);
    cv::waitKey(1);
}

double getSlope(cv::Vec4f line) {

    double x_a = line[0];
    double y_a = line[1];
    double x_b = line[2];
    double y_b = line[3];

    double m = (y_b - y_a)/(x_b - x_a);
    return m;
}

void chooseBestLines(std::vector<cv::Vec4f> lines) {
    lls.clear();
    std::vector<cv::Vec4f> best_lines;
    std::vector<std::pair<double, int>> lengths(lines.size());
    for(size_t i = 0; i < lines.size(); i++) {
        cv::Vec4f line_i = lines[i];

        cv::Point2f start_pt = cv::Point2f(line_i[0], line_i[1]);
        cv::Point2f end_pt = cv::Point2f(line_i[2], line_i[3]);

        double length = cv::norm(start_pt-end_pt);
        lengths.push_back(std::make_pair(length, i));
    }
    std::sort(lengths.begin(), lengths.end(),
            std::greater<std::pair<double, int>>());
    // Pick the 4 best lines
    for (int i = 0; i < 4; i++) {
        labelledLine ll;
        ll.line = lines[lengths[i].second];
        ll.slope = getSlope(ll.line);
        lls.push_back(ll);
        best_lines.push_back(lines[lengths[i].second]);
    }
}

void labelLines(char axis) {
    ROS_ASSERT(lls.size() == 4);
    std::vector<cv::Vec4f> sorted_lines;
    std::vector<std::pair<double, int>> dists_of_midpt;
    for(int i = 0; i < 4; i++) {
        cv::Vec4f line_i = lls[i].line;
        cv::Point2f start_pt = cv::Point2f(line_i[0], line_i[1]);
        cv::Point2f end_pt = cv::Point2f(line_i[2], line_i[3]);
        cv::Point2f mid_pt = 0.5*(start_pt + end_pt);
        if(axis == 'x')
            dists_of_midpt.push_back(std::make_pair(mid_pt.x, i));
        else if(axis == 'y')
            dists_of_midpt.push_back(std::make_pair(mid_pt.y, i));
        else
            ROS_ASSERT(axis == 'x' || axis == 'y');
    }
    ROS_ASSERT(dists_of_midpt.size() == 4);
    std::sort(dists_of_midpt.begin(),
              dists_of_midpt.end(),
              std::greater<std::pair<double, int>>());
    for (int i = 0; i < 4; i++) {
        if(axis == 'x') {
            if(i <= 1)
                lls[dists_of_midpt[i].second].labelX = 'r';
            else
                lls[dists_of_midpt[i].second].labelX = 'l';
        } else if (axis == 'y') {
            if(i <= 1)
                lls[dists_of_midpt[i].second].labelY = 'b';
            else
                lls[dists_of_midpt[i].second].labelY = 't';
        } else {
            ROS_ASSERT(axis == 'x' || axis == 'y');
        }
    }
}

void detectLines(cv::Mat image_in) {
    cv::Mat image_gray;
    cv::cvtColor(image_in, image_gray, CV_RGB2GRAY);
    int length_threshold = 100;
    float distance_threshold = 1.41421356f;
//    float distance_threshold = 1;
    double canny_th1 = 200.0;
    double canny_th2 = 200.0;
    int canny_aperture_size = 3;
    bool do_merge = true;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =
            cv::ximgproc::createFastLineDetector(length_threshold,
                                                     distance_threshold,
                                                     canny_th1, canny_th2,
                                                     canny_aperture_size,
                                                     do_merge);
    std::vector<cv::Vec4f> lines_fld;
    fld->detect(image_gray, lines_fld);
    if(lines_fld.size() >=4) {
        chooseBestLines(lines_fld);
        labelLines('x');
        labelLines('y');
        drawLineSegments(image_in);
    } else {
        cv::imshow("view", image_in);
        cv::waitKey(1);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try{
        global_header.stamp = msg->header.stamp;
        detectLines(cv_bridge::toCvShare(msg, "bgr8")->image);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_line_detector");
    ros::NodeHandle nh;
    cam_config_file_path = readParam<std::string>(nh, "cam_config_file_path");
    readCameraParams();
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
    normal_pub_lt = nh.advertise<normal_msg::normal>("/normal1", 1);
    normal_pub_rt = nh.advertise<normal_msg::normal>("/normal2", 1);
    normal_pub_rb = nh.advertise<normal_msg::normal>("/normal3", 1);
    normal_pub_lb = nh.advertise<normal_msg::normal>("/normal4", 1);
    normal_pub_chkrbrd = nh.advertise<normal_msg::normal>("/normal_plane", 1);
    tvec_pub_chkrbrd = nh.advertise<normal_msg::normal>("/tvec_plane", 1);
    ros::spin();
}
