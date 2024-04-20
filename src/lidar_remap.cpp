#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class lidar_remap {
private:
    ros::NodeHandle node;  // ROS node handle
    ros::Subscriber sub;   // ROS subscriber for Odometry messages
    ros::Publisher pub;    // ROS publisher for publish to ROS topics

    dynamic_reconfigure::Server<first_project::parametersConfig> server;
    dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;


    std::string odom_param;

public:
    lidar_remap() {
        pub = node.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1000);

        f = boost::bind(&lidar_remap::reconfig_callback, this, _1, _2);
        server.setCallback(f);

        // Subscribe to the "/os_cloud_node/points" topic with a queue size of 1000, and specify the callback function
        sub = node.subscribe("/os_cloud_node/points", 1000, &lidar_remap::sub_callback, this);
    }

    // Callback function for processing incoming PointCloud2 messages
    void sub_callback(const sensor_msgs::PointCloud2& msg){
        sensor_msgs::PointCloud2 new_msg = msg;
        new_msg.header.frame_id = odom_param;

        pub.publish(new_msg);
    }

    void reconfig_callback(first_project::parametersConfig &config, uint32_t level) {
        odom_param = config.odom_param;
    }
};


// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_remap");  // Initialize the ROS node

    lidar_remap my_lidar_remap;  // Create an instance of the lidar_remap class
    ros::spin();  // Enter the ROS event loop

    return 0;
}
