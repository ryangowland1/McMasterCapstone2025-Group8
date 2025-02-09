#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <iostream>

const float GREEN_THRESHOLD = 1.1f;
const float PLANE_DISTANCE_THRESHOLD = 0.02f;
const int STATISTICAL_NEIGHBORS = 50;
const float STANDARD_DEV_MULTIPLIER = 2.0f;
const int MAX_LINES = 5;

ros::Publisher green_cloud_pub;
ros::Publisher plane_cloud_pub;
ros::Publisher line_clouds_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : cloud->points) {
        if (point.g > GREEN_THRESHOLD * point.r && point.g > GREEN_THRESHOLD * point.b) {
            green_cloud->points.push_back(point);
        }
    }
    green_cloud->width = green_cloud->points.size();
    green_cloud->height = 1;
    green_cloud->is_dense = true;

    sensor_msgs::PointCloud2 green_cloud_msg;
    pcl::toROSMsg(*green_cloud, green_cloud_msg);
    green_cloud_msg.header = input_msg->header;
    green_cloud_pub.publish(green_cloud_msg);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(PLANE_DISTANCE_THRESHOLD);
    seg.setInputCloud(green_cloud);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : inliers->indices) {
        plane_cloud->points.push_back(green_cloud->points[idx]);
    }
    plane_cloud->width = plane_cloud->points.size();
    plane_cloud->height = 1;

    sensor_msgs::PointCloud2 plane_cloud_msg;
    pcl::toROSMsg(*plane_cloud, plane_cloud_msg);
    plane_cloud_msg.header = input_msg->header;
    plane_cloud_pub.publish(plane_cloud_msg);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(plane_cloud);
    float line_distance_threshold = 0.01f;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_clouds(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < MAX_LINES; ++i) {
        pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZRGB> line_seg;
        line_seg.setOptimizeCoefficients(true);
        line_seg.setModelType(pcl::SACMODEL_LINE);
        line_seg.setMethodType(pcl::SAC_RANSAC);
        line_seg.setDistanceThreshold(line_distance_threshold);
        line_seg.setInputCloud(plane_cloud);
        line_seg.segment(*line_inliers, *line_coefficients);

        if (line_inliers->indices.empty()) break;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setIndices(line_inliers);
        extract.setNegative(false);
        extract.filter(*line_cloud);

        for (size_t j = 0; j < line_cloud->points.size(); ++j) {
            uint8_t r = (i % 3 == 0) ? 255 : 0;
            uint8_t g = (i % 3 == 1) ? 255 : 0;
            uint8_t b = (i % 3 == 2) ? 255 : 0;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            line_cloud->points[j].rgb = *reinterpret_cast<float*>(&rgb);
            // add this line cloud's points to set of line clouds's points
            line_clouds->points.push_back(line_cloud->points[j]);
        }

        extract.setNegative(true);
        extract.filter(*plane_cloud);
        line_distance_threshold *= 1.1;
    }
    sensor_msgs::PointCloud2 line_clouds_msg;
    pcl::toROSMsg(*line_clouds, line_clouds_msg);
    line_clouds_msg.header = input_msg->header;
    line_clouds_pub.publish(line_clouds_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_processor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);
    
    green_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/green_cloud", 1);
    plane_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/plane_cloud", 1);
    line_clouds_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/line_cloud", 1);
    
    ros::spin();
    return 0;
}
