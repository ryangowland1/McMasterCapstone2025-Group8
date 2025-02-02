#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

ros::Publisher point_cloud_pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Create a PointCloud2 message
    sensor_msgs::PointCloud2 pointcloud;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Set up the PointCloud2 message
    pointcloud.header = scan_msg->header;
    pointcloud.height = 1;
    pointcloud.width = scan_msg->ranges.size();
    pointcloud.is_dense = true;
    pointcloud.is_bigendian = false;
    pointcloud.fields.resize(3);
    pointcloud.fields[0].name = "x";
    pointcloud.fields[1].name = "y";
    pointcloud.fields[2].name = "z";
    pointcloud.point_step = 3 * sizeof(float);
    pointcloud.row_step = pointcloud.point_step * pointcloud.width;
    pointcloud.data.resize(pointcloud.row_step * pointcloud.height);

    // Fill the point cloud data with laser scan readings
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        if (std::isinf(scan_msg->ranges[i]) || std::isnan(scan_msg->ranges[i])) {
            continue;  // Skip invalid data
        }

        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        pcl::PointXYZ point;
        point.x = scan_msg->ranges[i] * cos(angle);
        point.y = scan_msg->ranges[i] * sin(angle);
        point.z = 0.0f;  // Assuming the laser is 2D

        // Add the point to the PointCloud2 message
        sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
    }

    // Publish the PointCloud2 message
    point_cloud_pub.publish(pointcloud);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "laser_to_pointcloud");
    ros::NodeHandle nh;

    // Create a subscriber for the /scan topic
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, laserScanCallback);

    // Create a publisher for the point cloud data (Gazebo usually subscribes to /cloud)
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 10);

    // Spin to process incoming messages
    ros::spin();

    return 0;
}

