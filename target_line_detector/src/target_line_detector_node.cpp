#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

class targetLineDetector {
private:
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Publisher line_1_pub;
    ros::Publisher line_2_pub;
    ros::Publisher line_3_pub;
    ros::Publisher line_4_pub;

    std::vector<lineWithLabel> lls;

    int no_of_valid_obs;

    double ransac_threshold;
    double dot_prod_low_val;
    double dot_prod_high_val;
    int min_points_per_line;

    std::string node_name;
public:
    targetLineDetector(ros::NodeHandle n) {
        nh = n;
        node_name = ros::this_node::getName();
        cloud_sub = nh.subscribe("/cloud_in", 1, &targetLineDetector::cloudCallback, this);
        line_1_pub = nh.advertise<sensor_msgs::PointCloud2>("/line1_out", 1);
        line_2_pub = nh.advertise<sensor_msgs::PointCloud2>("/line2_out", 1);
        line_3_pub = nh.advertise<sensor_msgs::PointCloud2>("/line3_out", 1);
        line_4_pub = nh.advertise<sensor_msgs::PointCloud2>("/line4_out", 1);

        dot_prod_low_val = readParam<double>(nh, "dot_prod_low_val");
        dot_prod_high_val = readParam<double>(nh, "dot_prod_high_val");
        ransac_threshold = readParam<double>(nh, "ransac_threshold");
        min_points_per_line = readParam<int>(nh, "min_points_per_line");
        no_of_valid_obs = 0;
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name) {
        T ans;
        if (n.getParam(name, ans)) {
            ROS_INFO_STREAM("[" << node_name << "] " << "Loaded " << name << ": " << ans);
        }
        else {
            ROS_ERROR_STREAM("[" << node_name << "] " << " Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void remove_inliers(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                        std::vector<int> inliers_indices,
                        pcl::PointCloud<pcl::PointXYZI> &cloud_out) {

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

    void publishLines (pcl::PointCloud<pcl::PointXYZI> line1_pcl,
                       pcl::PointCloud<pcl::PointXYZI> line2_pcl,
                       pcl::PointCloud<pcl::PointXYZI> line3_pcl,
                       pcl::PointCloud<pcl::PointXYZI> line4_pcl,
                       ros::Time time_stamp, std::string frame_id) {
        sensor_msgs::PointCloud2 line1_ros, line2_ros, line3_ros, line4_ros;
        pcl::toROSMsg(line1_pcl, line1_ros);
        pcl::toROSMsg(line2_pcl, line2_ros);
        pcl::toROSMsg(line3_pcl, line3_ros);
        pcl::toROSMsg(line4_pcl, line4_ros);
        line1_ros.header.frame_id = line2_ros.header.frame_id =
        line3_ros.header.frame_id = line4_ros.header.frame_id = frame_id;
        line1_ros.header.stamp = line2_ros.header.stamp =
        line3_ros.header.stamp = line4_ros.header.stamp = time_stamp;
        line_1_pub.publish(line1_ros);
        line_2_pub.publish(line2_ros);
        line_3_pub.publish(line3_ros);
        line_4_pub.publish(line4_ros);
    }

    Eigen::Vector3d getCentroid(pcl::PointCloud<pcl::PointXYZI> cloud_in) {
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(cloud_in, centroid);
        return Eigen::Vector3d(centroid(0), centroid(1), centroid(2));
    }

    void labelLines(char axis) {
        ROS_ASSERT(lls.size() == 4);
        std::vector<std::pair<double, int>> dists_axis_mp;
        for(int i = 0; i < 4; i++) {
            Eigen::Vector3d centroid_i = getCentroid(lls[i].line_pts);
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

    void publishLinesInOrder(ros::Time time_stamp, std::string frame_id) {
        ROS_INFO_STREAM("[ " << node_name << " ] " << "Publishing LIDAR Lines");
        ROS_ASSERT(lls.size() == 4);
        sensor_msgs::PointCloud2 line1_ros, line2_ros, line3_ros, line4_ros;
        for(size_t i = 0; i < 4; i++) {
            char labelZ = lls[i].labelZ;
            char labelY = lls[i].labelY;
            if(labelZ == 'b' && labelY == 'l') {
                pcl::toROSMsg(lls[i].line_pts, line1_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line1_ros.header.stamp = time_stamp;
                    line1_ros.header.frame_id = frame_id;
                    line_1_pub.publish(line1_ros);
                }
            }

            if(labelZ == 'b' && labelY == 'r') {
                pcl::toROSMsg(lls[i].line_pts, line2_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line2_ros.header.stamp = time_stamp;
                    line2_ros.header.frame_id = frame_id;
                    line_2_pub.publish(line2_ros);
                }
            }

            if(labelZ == 't' && labelY == 'r') {
                pcl::toROSMsg(lls[i].line_pts, line3_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line3_ros.header.stamp = time_stamp;
                    line3_ros.header.frame_id = frame_id;
                    line_3_pub.publish(line3_ros);
                }
            }

            if(labelZ == 't' && labelY == 'l') {
                pcl::toROSMsg(lls[i].line_pts, line4_ros);
                if(lls[i].line_pts.points.size() > 0) {
                    line4_ros.header.stamp = time_stamp;
                    line4_ros.header.frame_id = frame_id;
                    line_4_pub.publish(line4_ros);
                }
            }
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        ros::Time time_stamp = cloud_msg->header.stamp;
        std::string frame_id = cloud_msg->header.frame_id;
        pcl::PointCloud<pcl::PointXYZI>::Ptr
                cloud_msg_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud_msg_pcl);
        int no_of_incoming_pts = cloud_msg_pcl->points.size();
        std::vector<pcl::PointCloud<pcl::PointXYZI>> lines_pts;
        std::vector<Eigen::VectorXf> lines_eqns;
        for (int i = 0; i < 4; i++) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(cloud_msg_pcl);
            pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new
            pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud_ptr));
            pcl::RandomSampleConsensus<pcl::PointXYZI> ransac_l(model_l);
            ransac_l.setDistanceThreshold(ransac_threshold);
            ransac_l.computeModel();
            std::vector<int> line_inliers;
            ransac_l.getInliers(line_inliers);
            if(!line_inliers.empty()) {
                pcl::PointCloud<pcl::PointXYZI> line_i;
                pcl::copyPointCloud<pcl::PointXYZI>(*cloud_msg_pcl,
                                                    line_inliers,
                                                    line_i);
                lines_pts.push_back(line_i);
                lines_eqns.push_back(ransac_l.model_coefficients_);
            } else {
                break;
            }
            pcl::PointCloud<pcl::PointXYZI> plane_no_line;
            remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
            *cloud_msg_pcl = plane_no_line;
        }
        if (lines_pts.size() == 4) {
            pcl::PointCloud<pcl::PointXYZI> line1_pcl = lines_pts[0];
            pcl::PointCloud<pcl::PointXYZI> line2_pcl = lines_pts[1];
            pcl::PointCloud<pcl::PointXYZI> line3_pcl = lines_pts[2];
            pcl::PointCloud<pcl::PointXYZI> line4_pcl = lines_pts[3];
            ulong no_pts_l1 = line1_pcl.points.size();
            ulong no_pts_l2 = line2_pcl.points.size();
            ulong no_pts_l3 = line3_pcl.points.size();
            ulong no_pts_l4 = line4_pcl.points.size();
            Eigen::VectorXf line1_eqn = lines_eqns[0];
            Eigen::Vector3d line1_direction = Eigen::Vector3d(line1_eqn[3],
                                                              line1_eqn[4],
                                                              line1_eqn[5]);
            Eigen::VectorXf line2_eqn = lines_eqns[1];
            Eigen::Vector3d line2_direction = Eigen::Vector3d(line2_eqn[3],
                                                              line2_eqn[4],
                                                              line2_eqn[5]);
            Eigen::VectorXf line3_eqn = lines_eqns[2];
            Eigen::Vector3d line3_direction = Eigen::Vector3d(line3_eqn[3],
                                                              line3_eqn[4],
                                                              line3_eqn[5]);
            Eigen::VectorXf line4_eqn = lines_eqns[3];
            Eigen::Vector3d line4_direction = Eigen::Vector3d(line4_eqn[3],
                                                              line4_eqn[4],
                                                              line4_eqn[5]);

            double angle1 = fabs(line1_direction.dot(line2_direction));
            double angle2 = fabs(line1_direction.dot(line3_direction));
            double angle3 = fabs(line1_direction.dot(line4_direction));
            std::vector<double> angles_arr;
            angles_arr.push_back(angle1);
            angles_arr.push_back(angle2);
            angles_arr.push_back(angle3);
            int countRA = 0;
            int countP = 0;
            for(int m = 0; m < angles_arr.size(); m++) {
                if(angles_arr[m] < dot_prod_low_val)
                    countRA++;
                if(angles_arr[m] > dot_prod_high_val)
                    countP++;
            }
            bool count_condition = (no_pts_l1 > min_points_per_line &&
                                    no_pts_l2 > min_points_per_line &&
                                    no_pts_l3 > min_points_per_line &&
                                    no_pts_l4 > min_points_per_line);
            bool angle_conditon = (countRA == 2 && countP == 1);
            if (count_condition &&
                angle_conditon) {
                lls.clear();
                for (int l = 0; l < 4; l++) {
                    lineWithLabel ll;
                    ll.line_pts = lines_pts[l];
                    lls.push_back(ll);
                }
                labelLines('z');
                labelLines('y');
                publishLinesInOrder(time_stamp, frame_id);
            }
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_detector_node");
    ros::NodeHandle nh("~");
    targetLineDetector tPD(nh);
    ros::spin();
}
