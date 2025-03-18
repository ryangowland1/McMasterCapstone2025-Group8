#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <limits>
#include <cmath>

ros::Publisher path_pub;

// Define the starting position (x, y, z)
const float START_X = 0.0;
const float START_Y = -0.06;
const float START_Z = 0.0;
const float END_Y_OFFSET = 0.0;  // Adjust end point y to avoid aiming too high

int empty_count = 0;  // Tracks consecutive empty parking spot cycles

// Generate a smooth Bezier path
std::vector<geometry_msgs::PoseStamped> generateBezierPath(const Eigen::Vector3f& start, const Eigen::Vector3f& control, const Eigen::Vector3f& control2, const Eigen::Vector3f& end, int num_points) {
    std::vector<geometry_msgs::PoseStamped> path;
    for (int i = 0; i <= num_points; ++i) {
        float t = static_cast<float>(i) / num_points;
        float one_minus_t = 1.0 - t;

        // Quadratic Bezier curve formula
        // Eigen::Vector3f point = one_minus_t * one_minus_t * start +
                                // 2 * one_minus_t * t * control +
                                // t * t * end;

        //Explicit Cubic Bezier curve formula
        Eigen::Vector3f point = one_minus_t * one_minus_t * one_minus_t * start +
                                3 * one_minus_t * one_minus_t * t * control +
                                3 * one_minus_t * t * t * control2 +
                                t * t * t * end;

            
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map"; 
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = 0.0;  // Force Z to always be 0

        pose.pose.orientation.w = 1.0;  // Neutral orientation
        path.push_back(pose);
    }
    return path;
}

void parkingSpotsCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr parking_spots(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *parking_spots);

    if (parking_spots->empty()) {
        ROS_WARN("No parking spots received.");

        empty_count++; // Increase empty cycle counter
        if (empty_count >= 2) {
            // Publish an empty path to remove the old one
            nav_msgs::Path empty_path;
            empty_path.header.stamp = ros::Time::now();
            empty_path.header.frame_id = "map";
            path_pub.publish(empty_path);
            ROS_INFO("Published empty path to clear old parking path.");
        }
        return;
    }

    // Reset empty count since we found parking spots
    empty_count = 0;

    // Find the closest parking spot
    float min_distance = std::numeric_limits<float>::max();
    pcl::PointXYZRGB closest_spot;

    for (const auto& spot : parking_spots->points) {
        float distance = std::sqrt(std::pow(spot.x - START_X, 2) + 
                                   std::pow(spot.y - START_Y, 2) + 
                                   std::pow(spot.z - START_Z, 2));

        if (distance < min_distance) {
            min_distance = distance;
            closest_spot = spot;
        }
    }

    // Define start, control, and end points for the Bezier curve
    Eigen::Vector3f start(START_X, START_Y, START_Z);
    
    // Offset the end point's Y value
    Eigen::Vector3f end(closest_spot.x, closest_spot.y + END_Y_OFFSET, 0.0);

    // Compute a control point dynamically to make the curve tangent at the start
    Eigen::Vector3f control;
    float offset_distance = 0.5 * (end.x() - start.x());  // Adjust control point based on parking spot
    control.x() = start.x() + offset_distance;
    control.y() = start.y();
    control.z() = 0.0;  // Force control point to be on ground

    // Compute a control point dynamically to make the curve tangent at the end
    Eigen::Vector3f control2;
    control2.x() = end.x() - offset_distance;
    control2.y() = end.y();
    control2.z() = 0.0;  // Force control point to be on ground

    // Generate a smooth curved path
    int num_path_points = 20;  // More points for a smoother curve
    std::vector<geometry_msgs::PoseStamped> bezier_path = generateBezierPath(start, control, control2, end, num_path_points);

    // Create a nav_msgs::Path message
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "zed2_left_camera_frame"; 

    path_msg.poses = bezier_path;

    // Publish the path
    path_pub.publish(path_msg);
    ROS_INFO("Published smooth path to closest parking spot (%.2f, %.2f, %.2f)", 
             end.x(), end.y(), end.z());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "parking_path_planner");
    ros::NodeHandle nh;

    path_pub = nh.advertise<nav_msgs::Path>("/parking_path", 10);
    ros::Subscriber sub = nh.subscribe("/processed/parking_spots", 1, parkingSpotsCallback);

    ros::spin();
    return 0;
}
