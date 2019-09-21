
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <velodyne_pointcloud/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

struct lineWithLabel {
    pcl::PointCloud<pcl::PointXYZI> line_pts;
    char labelZ;
    char labelY;
};

class lidarLineDetection {
private:
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::Publisher line_1_pub;
    ros::Publisher line_2_pub;
    ros::Publisher line_3_pub;
    ros::Publisher line_4_pub;

    std::vector<lineWithLabel> lls;

    int good_frames;

public:
    lidarLineDetection() {
        cloud_sub = nh.subscribe("/cloud_in", 1,
                                 &lidarLineDetection::cloudHandler,
                                 this);
        good_frames = 0;
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_out", 1);
        line_1_pub = nh.advertise<sensor_msgs::PointCloud2>("/line1_out", 1);
        line_2_pub = nh.advertise<sensor_msgs::PointCloud2>("/line2_out", 1);
        line_3_pub = nh.advertise<sensor_msgs::PointCloud2>("/line3_out", 1);
        line_4_pub = nh.advertise<sensor_msgs::PointCloud2>("/line4_out", 1);
    }

    void remove_inliers(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                        std::vector<int> inliers_indices,
                        pcl::PointCloud<pcl::PointXYZI> &cloud_out)
    {

        std::vector<int> outliers_indicies;
        for (size_t i = 0; i < cloud_in.size(); i++)
        {
            if (find(inliers_indices.begin(), inliers_indices.end(), i) == inliers_indices.end())
            {
                outliers_indicies.push_back(i);
            }
        }
        pcl::copyPointCloud<pcl::PointXYZI>(cloud_in, outliers_indicies, cloud_out);
    }

    Eigen::Vector3d getCentroid(pcl::PointCloud<pcl::PointXYZI> cloud_in) {
        Eigen::Vector3d centroid;
        double x, y, z;
        x = y = z = 0;
        for(int i = 0; i < cloud_in.points.size(); i++) {
            x += cloud_in.points[i].x;
            y += cloud_in.points[i].y;
            z += cloud_in.points[i].z;
        }
        x /= cloud_in.points.size();
        y /= cloud_in.points.size();
        z /= cloud_in.points.size();
        return Eigen::Vector3d(x, y, z);
    }

    void labelLines(char axis) {
        ROS_ASSERT(lls.size() == 4);
        std::vector<std::pair<double, int>> dists_axis_mp;
        for(int i = 0; i < 4; i++) {
            Eigen::Vector3d centroid_i = getCentroid(lls[i].line_pts);
            std::cout << centroid_i.transpose() << std::endl;
            if(axis == 'z')
                dists_axis_mp.push_back(std::make_pair(centroid_i.z(), i));
            else if(axis == 'y')
                dists_axis_mp.push_back(std::make_pair(centroid_i.y(), i));
            else
                ROS_ASSERT(axis == 'z' || axis == 'y');
        }
        ROS_ASSERT(dists_axis_mp.size() == 4);
        std::sort(dists_axis_mp.begin(),
                  dists_axis_mp.end(),
                  std::greater<std::pair<double, int>>());
        for (int i = 0; i < 4; i++) {
            if(axis == 'z') {
                if(i <= 1)
                    lls[dists_axis_mp[i].second].labelZ = 'b';
                else
                    lls[dists_axis_mp[i].second].labelZ = 't';
            } else if (axis == 'y') {
                if(i <= 1)
                    lls[dists_axis_mp[i].second].labelY = 'l';
                else
                    lls[dists_axis_mp[i].second].labelY = 'r';
            } else {
                ROS_ASSERT(axis == 'z' || axis == 'y');
            }
        }
    }

    void publishLinesInOrder() {
        sensor_msgs::PointCloud2 line1_ros, line2_ros, line3_ros, line4_ros;
        ROS_ASSERT(lls.size() == 4);
        for(size_t i = 0; i < 4; i++) {
            char labelZ = lls[i].labelZ;
            char labelY = lls[i].labelY;
            if(labelZ == 'b' && labelY == 'l') {
                pcl::toROSMsg(lls[i].line_pts, line1_ros);
                line_1_pub.publish(line1_ros);
            }

            if(labelZ == 'b' && labelY == 'r') {
                pcl::toROSMsg(lls[i].line_pts, line2_ros);
                line_2_pub.publish(line2_ros);
            }

            if(labelZ == 't' && labelY == 'r') {
                pcl::toROSMsg(lls[i].line_pts, line3_ros);
                line_3_pub.publish(line3_ros);
            }

            if(labelZ == 't' && labelY == 'l') {
                pcl::toROSMsg(lls[i].line_pts, line4_ros);
                line_4_pub.publish(line4_ros);
            }
        }
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr
                plane(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *plane);

        pcl::PointCloud<pcl::PointXYZI> temp_plane(*plane);

        std::vector<pcl::PointCloud<pcl::PointXYZI>> lines_pts;
        std::vector<Eigen::VectorXf> lines_eqns;
        for (int i = 0; i < 4; i++){
            pcl::PointCloud<pcl::PointXYZI>::Ptr plane_ptr(plane);
            pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new
                                                                               pcl::SampleConsensusModelLine<pcl::PointXYZI>(plane_ptr));
            pcl::RandomSampleConsensus<pcl::PointXYZI> ransac_l(model_l);
            ransac_l.setDistanceThreshold(0.02);
            ransac_l.computeModel();
//            ROS_INFO_STREAM("Model coeff: " << ransac_l.model_coefficients_);
            std::vector<int> line_inliers;
            ransac_l.getInliers(line_inliers);

            if (line_inliers.empty()){
                break;
            } else {
                pcl::PointCloud<pcl::PointXYZI> line_i;
                pcl::copyPointCloud<pcl::PointXYZI>(*plane,
                                                    line_inliers,
                                                    line_i);
                lines_pts.push_back(line_i);
                lines_eqns.push_back(ransac_l.model_coefficients_);
            }
            pcl::PointCloud<pcl::PointXYZI> plane_no_line;
            remove_inliers(*plane_ptr, line_inliers, plane_no_line);
            *plane = plane_no_line;
        }
        ROS_ASSERT(lines_pts.size() == lines_eqns.size());
        if(lines_pts.size() == 4) {
            if(lines_pts[0].size() > 4 && lines_pts[1].size() > 4 &&
               lines_pts[2].size() > 4 && lines_pts[3].size() > 4) {
                int count_0s = 0;
                int count_1s = 0;
                for(int i = 0; i < 4; i++) {
                    Eigen::VectorXf line_i = lines_eqns[i];
                    Eigen::Vector3f line_i_direction(line_i[3], line_i[4], line_i[5]);
                    for(int j = i+1; j < 4; j++) {
                        Eigen::VectorXf line_j = lines_eqns[j];
                        Eigen::Vector3f line_j_direction(line_j[3], line_j[4], line_j[5]);
                        double dot_product = line_i_direction.dot(line_j_direction);
                        if(fabs(dot_product) > 0.95)
                            count_1s++;
                        if(fabs(dot_product) < 0.15)
                            count_0s++;
                    }
                }
                if(count_0s == 4 && count_1s == 2) {
                    ROS_WARN_STREAM("Recorded Frame no: " << ++good_frames);
                    lls.clear();
                    for(int i = 0; i < 4; i++) {
                        lineWithLabel ll;
                        ll.line_pts = lines_pts[i];
                        lls.push_back(ll);
                    }
                    labelLines('z');
                    labelLines('y');
                    publishLinesInOrder();
                } else {
                    ROS_WARN_STREAM("Incoherent angles b/w lines");
                }
            } else {
                ROS_WARN_STREAM("Insufficient Points");
            }
        } else {
            ROS_WARN_STREAM("Detected less than 4 lines");
        }
        sensor_msgs::PointCloud2 cloud_out_ros;
        pcl::toROSMsg(temp_plane, cloud_out_ros);

        cloud_pub.publish(cloud_out_ros);

    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_line_detection_node");
    lidarLineDetection lLD;
    ros::spin();
}