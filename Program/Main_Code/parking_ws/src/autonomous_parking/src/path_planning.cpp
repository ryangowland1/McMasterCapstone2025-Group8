#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>

ros::Publisher path_pub;
nav_msgs::Path path;
geometry_msgs::Point last_point;
bool first_point = true;

void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (first_point)
    {
        last_point = msg->point;
        first_point = false;
        return;
    }

    geometry_msgs::Point current_point = msg->point;
    
    // Compute arc path between last_point and current_point
    double num_steps = 20; // Define how smooth the arc should be
    double cx = (last_point.x + current_point.x) / 2.0;
    double cy = (last_point.y + current_point.y) / 2.0;
    double radius = hypot(current_point.x - last_point.x, current_point.y - last_point.y) / 2.0;
    double start_angle = atan2(last_point.y - cy, last_point.x - cx);
    double end_angle = atan2(current_point.y - cy, current_point.x - cx);
    
    if (end_angle < start_angle) end_angle += 2 * M_PI;
    double angle_step = (end_angle - start_angle) / num_steps;

    for (int i = 0; i <= num_steps; ++i)
    {
        double theta = start_angle + i * angle_step;
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "odom";
        pose.pose.position.x = cx + radius * cos(theta);
        pose.pose.position.y = cy + radius * sin(theta);
        pose.pose.position.z = last_point.z + (current_point.z - last_point.z) * (i / num_steps);
        path.poses.push_back(pose);
    }
    
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";
    path_pub.publish(path);

    last_point = current_point;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;

    path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 10);
    ros::Subscriber point_sub = nh.subscribe("/input_points", 10, pointCallback);

    ros::spin();
    return 0;
}
