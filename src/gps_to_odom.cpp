#include <cmath>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"


#define WGS84_A 6378137.0         // Semi-major axis of the WGS84 ellipsoid (in meters)
#define WGS84_B 6356752.31424518  // Semi-minor axis of the WGS84 ellipsoid (in meters)
#define WGS84_E 0.0818191908      // First eccentricity of the WGS84 ellipsoid


// Definition of type to simplify the code
using Vector = std::vector<double>;

class gps_to_odom {
    double x;  // X ENU coordinate
    double y;  // Y ENU coordinate
    double z;  // Z ENU coordinate

private:
    ros::NodeHandle node;  // ROS node handle for ROS node functionalities
    ros::Subscriber sub;   // ROS subscriber for subscribing to ROS topics
    ros::Publisher pub;    // ROS publisher for publish to ROS topics

    double lat_r = 0;  // Reference latitude, will be set at the fist GPS coordinate received from the bag
    double lon_r = 0;  // Reference longitude, will be set at the fist GPS coordinate received from the bag
    double alt_r = 0;  // Reference altitude, will be set at the fist GPS coordinate received from the bag

    /** @brief Function to convert GPS coordinates to ECEF (Earth-Centered, Earth-Fixed).
     * 
     * * GPS coordinates must be in degrees
     *
     * \param lat Latitude to convert
     * \param lon Longitude to convert
     * \param alt Altitude to convert
     * \return Resultant size 3 ecef vector.
     *
     */
    Vector gps_to_ecef(double lat, double lon, double alt) {
        double slat = sin(lat * M_PI / 180.0);
        double clat = cos(lat * M_PI / 180.0);
        double slon = sin(lon * M_PI / 180.0);
        double clon = cos(lon * M_PI / 180.0);

        double n_long = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * slat * slat);
        
        Vector ecef(3);
        ecef[0] = (n_long + alt) * clat * clon;                       // ECEF X coordinate
        ecef[1] = (n_long + alt) * clat * slon;                       // ECEF Y coordinate
        ecef[2] = (n_long * (1.0 - WGS84_E * WGS84_E) + alt) * slat;  // ECEF Z coordinate

        return ecef;  // Return the ECEF coordinates
    }

    /** @brief Function to convert GPS coordinates to ENU (East-North-Up) coordinates.
     * 
     * * GPS coordinates must be in degrees
     *
     * \param lat Latitude to convert
     * \param lon Longitude to convert
     * \param alt Altitude to convert
     * \return Resultant size 3 enu vector.
     *
     */
    Vector gps_to_enu(double lat, double lon, double alt) {
        Vector ecef_p = gps_to_ecef(lat, lon, alt);  // Convert current GPS to ECEF

        // Convert reference GPS to ECEF
        Vector ecef_r = gps_to_ecef(lat_r, lon_r, alt_r);

        // Difference vector between current and reference ECEF coordinates
        Vector vector = {
            ecef_p[0] - ecef_r[0],
            ecef_p[1] - ecef_r[1],
            ecef_p[2] - ecef_r[2]
        };

        double slat_r = sin(lat_r  * M_PI / 180.0);
        double clat_r = cos(lat_r  * M_PI / 180.0);
        double slon_r = sin(lon_r  * M_PI / 180.0);
        double clon_r = cos(lon_r  * M_PI / 180.0);

        double east = -slon_r * vector[0] + clon_r * vector[1];
        double north = -slat_r * clon_r * vector[0] - slat_r * slon_r * vector[1] + clat_r * vector[2];
        double up = clat_r * clon_r * vector[0] + clat_r * slon_r * vector[1] + slat_r * vector[2];

        // Apply an additional rotation of 130 degrees around the z-axis
        Vector result(3, 0);
        result[0] = east;// * cos(130 * M_PI / 180) - north * sin(130 * M_PI / 180);
        result[1] = east;// * sin(130 * M_PI / 180) + north * cos(130 * M_PI / 180);
        result[2] = up;

        //return multiplyMatrixVector(matrix_r, vector);  // Return the result of matrix multiplication
        return result;
    }

public:
    gps_to_odom() {
        ros::NodeHandle nh_private("~");

        nh_private.getParam("lat_r", lat_r);
        nh_private.getParam("lon_r", lon_r);
        nh_private.getParam("alt_r", alt_r);

        pub = node.advertise<nav_msgs::Odometry>("gps_odom", 1000);
        sub = node.subscribe<sensor_msgs::NavSatFix>("fix", 1, &gps_to_odom::callback, this);  // Subscribe to NavSatFix topic
    }

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // LLA Conversion to radiant
        double latitude = msg->latitude;
        double longitude = msg->longitude;
        double altitude = msg->altitude;

        Vector enu = gps_to_enu(latitude, longitude, altitude);

        // Store previous pose for heading calculation
        static double prev_x = 0.0;
        static double prev_y = 0.0;

        // Calculate robot heading
        double dx = enu[0] - prev_x;
        double dy = enu[1] - prev_y;

        double heading = atan2(dy, dx);  // Calculate heading angle

        // Set previous pose for next iteration
        prev_x = enu[0];
        prev_y = enu[1];

        // Set ENU coordinates
        x = enu[0];
        y = enu[1];
        z = 0;

        // Setup the odometry message
        nav_msgs::Odometry odometry_msg;
        odometry_msg.header.stamp = msg->header.stamp;
        odometry_msg.header.frame_id = "world";
        odometry_msg.child_frame_id = "base_link";

        odometry_msg.pose.pose.position.x = x;
        odometry_msg.pose.pose.position.y = y;
        odometry_msg.pose.pose.position.z = z;

        // Set the orientation as a quaternion
        // 2D system soo 3 DOF: x, y, tetha(qz)
        double yaw = heading / 2.0;
        double qw = cos(yaw);
        double qx = 0.0;
        double qy = 0.0;
        double qz = sin(yaw);

        odometry_msg.pose.pose.orientation.w = qw;
        odometry_msg.pose.pose.orientation.x = qx;
        odometry_msg.pose.pose.orientation.y = qy;
        odometry_msg.pose.pose.orientation.z = qz;

        // Publish the odometry message
        pub.publish(odometry_msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");  // Initialize the ROS node

    gps_to_odom my_gps_to_odom;  // Create an instance of the gps_to_odom class
    ros::spin();  // Enter the ROS event loop

    return 0;
}
