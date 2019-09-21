#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Velodyne.h>


class LidarEdgeDetector {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub_edges;

    sensor_msgs::PointCloud2 cloud_edges_out_ros;

    std::string lidar_input_topic_name;
    std::string lidar_output_topic_name;
    double edge_threshold;

public:
    LidarEdgeDetector(){
        lidar_input_topic_name = readParam<std::string>(nh, "lidar_input_topic_name");
        lidar_output_topic_name = lidar_input_topic_name+"/with_edges";
        edge_threshold = readParam<double>(nh, "edge_threshold");
        cloud_sub = nh.subscribe(lidar_input_topic_name, 10,
                &LidarEdgeDetector::callback, this);
        cloud_pub_edges = nh.advertise<sensor_msgs::PointCloud2>
                (lidar_output_topic_name, 10);
    }

    template <typename T>
    T readParam(ros::NodeHandle &n, std::string name){
        T ans;
        if (n.getParam(name, ans)){
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else{
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void detectEdges(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
        pcl::PointCloud<Velodyne::Point> cloud_in;
        pcl::fromROSMsg(*cloud_msg, cloud_in);
        Velodyne::Velodyne scan(cloud_in);
        scan.getRings();
        scan.intensityByRangeDiff();
        scan.normalizeIntensity();
        Velodyne::Velodyne thresholded_scan = scan.threshold(edge_threshold);
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_cloud_ptr(thresholded_scan.toPointsXYZI());
        pcl::toROSMsg(*xyz_cloud_ptr, cloud_edges_out_ros);
        cloud_edges_out_ros.header.frame_id = cloud_msg->header.frame_id;
        cloud_edges_out_ros.header.stamp = cloud_msg->header.stamp;
        cloud_pub_edges.publish(cloud_edges_out_ros);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        ros::Time begin = ros::Time::now();
        detectEdges(cloud_msg);
        ros::Time end = ros::Time::now();
        double time_elapsed = end.toSec() - begin.toSec();
        ROS_WARN_STREAM("Edge Detection Took: " << time_elapsed << " [s]");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_edge_detector_node");
    LidarEdgeDetector LED;
    ros::spin();
    return 0;
}