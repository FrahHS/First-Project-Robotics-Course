#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class odom_to_tf {
private:
    ros::NodeHandle n;            // ROS node handle
    tf::TransformBroadcaster br;  // ROS transform broadcaster
    ros::Subscriber sub;          // ROS subscriber for Odometry messages

public:
    odom_to_tf() {
        // Subscribe to the "/input_odom" topic with a queue size of 1000, and specify the callback function
        sub = n.subscribe("/input_odom", 1000, &odom_to_tf::callback, this);
    }

    // Callback function for processing incoming Odometry messages
    void callback(const nav_msgs::Odometry& msg){
        ros::NodeHandle nh_private("~");  // Private node handle for accessing parameters

        std::string root_frame;  // Variable to store the name of the root frame
        std::string child_frame;  // Variable to store the name of the child frame

        // Get parameters for root_frame and child_frame from the parameter server
        nh_private.getParam("root_frame", root_frame);
        nh_private.getParam("child_frame", child_frame);

        // Create a transform object
        tf::Transform transform;

        // Extract position and orientation data from the Odometry message
        double x = msg.pose.pose.position.x;
        double y = msg.pose.pose.position.y;
        double z = 0;
        double theta = msg.pose.pose.orientation.z;

        // Set the translation component of the transform
        transform.setOrigin( tf::Vector3(x, y, z) );

        // Create a Quaternion for representing orientation
        tf::Quaternion q;
        q.setRPY(0, 0, theta);  // Set the roll, pitch, and yaw components of the Quaternion

        // Set the rotation component of the transform
        transform.setRotation(q);

        // Broadcast the transform with a time stamp to the "/odom" frame
        br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, root_frame, child_frame));
    }
};

// Main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_to_tf");  // Initialize the ROS node

    odom_to_tf my_odom_to_tf;  // Create an instance of the odom_to_tf class
    ros::spin();  // Enter the ROS event loop

    return 0;
}
