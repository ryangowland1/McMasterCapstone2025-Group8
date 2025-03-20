#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>


const float GREEN_THRESHOLD = 1.05f;
const float Z_THRESHOLD = -0.1f;
const float PLANE_DISTANCE_THRESHOLD = 0.02f;
const int MAX_LINES = 5;
const float PARKING_MIN_DIST = 0.43f;
const float PARKING_MAX_DIST = 0.45f;

ros::Publisher green_cloud_pub;
ros::Publisher plane_cloud_pub;
ros::Publisher line_clouds_pub;
ros::Publisher centroid_distance_pub;
ros::Publisher centroid_cloud_pub;
ros::Publisher parking_spots_pub; // New publisher for parking spots
ros::Publisher obstacle_cloud_pub; // New publisher for obstacles

void computeCentroidsAndPublishDistances(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& line_clouds, const std_msgs::Header& header) {
    std::vector<Eigen::Vector3f> centroids;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr parking_spots(new pcl::PointCloud<pcl::PointXYZRGB>); // Parking spots point cloud

    for (const auto& cloud : line_clouds) {
        if (cloud->empty()) continue;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        centroids.emplace_back(centroid.head<3>());

        pcl::PointXYZRGB centroid_point;
        centroid_point.x = centroid[0];
        centroid_point.y = centroid[1];
        centroid_point.z = centroid[2];
        centroid_point.r = 255;
        centroid_point.g = 0;
        centroid_point.b = 0;
        centroid_cloud->points.push_back(centroid_point);
    }

    sensor_msgs::PointCloud2 centroid_cloud_msg;
    pcl::toROSMsg(*centroid_cloud, centroid_cloud_msg);
    centroid_cloud_msg.header = header;
    centroid_cloud_pub.publish(centroid_cloud_msg);

    std_msgs::Float32MultiArray distances_msg;
    distances_msg.data.clear();

    for (size_t i = 0; i < centroids.size(); ++i) {
        for (size_t j = i + 1; j < centroids.size(); ++j) {
            float distance = (centroids[i] - centroids[j]).norm();
            distances_msg.data.push_back(distance);

            // Check if the distance is within the parking spot range
            if (distance >= PARKING_MIN_DIST && distance <= PARKING_MAX_DIST) {
                Eigen::Vector3f midpoint = (centroids[i] + centroids[j]) / 2.0f;

                pcl::PointXYZRGB parking_spot;
                parking_spot.x = midpoint[0];
                parking_spot.y = midpoint[1];
                parking_spot.z = midpoint[2];
                parking_spot.r = 0;   // Blue color for parking spot
                parking_spot.g = 0;
                parking_spot.b = 255;
                parking_spots->points.push_back(parking_spot);
            }
        }
    }

    if (!distances_msg.data.empty())
        centroid_distance_pub.publish(distances_msg);

    // Publish parking spots point cloud
    sensor_msgs::PointCloud2 parking_spots_msg;
    pcl::toROSMsg(*parking_spots, parking_spots_msg);
    parking_spots_msg.header = header;
    parking_spots_pub.publish(parking_spots_msg);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : cloud->points) {
        if (point.g > GREEN_THRESHOLD * point.r && point.g > GREEN_THRESHOLD * point.b) {
            green_cloud->points.push_back(point);
        }
    }

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

    sensor_msgs::PointCloud2 plane_cloud_msg;
    pcl::toROSMsg(*plane_cloud, plane_cloud_msg);
    plane_cloud_msg.header = input_msg->header;
    plane_cloud_pub.publish(plane_cloud_msg);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(plane_cloud);
    float line_distance_threshold = 0.01f;
    
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> line_clouds;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_line_clouds(new pcl::PointCloud<pcl::PointXYZRGB>);

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

        for (auto& point : line_cloud->points) {
            point.r = 255;
            point.g = 255;
            point.b = 0;
            all_line_clouds->points.push_back(point);
        }

        line_clouds.push_back(line_cloud);
        extract.setNegative(true);
        extract.filter(*plane_cloud);
        line_distance_threshold *= 1.1;
    }

    sensor_msgs::PointCloud2 line_clouds_msg;
    pcl::toROSMsg(*all_line_clouds, line_clouds_msg);
    line_clouds_msg.header = input_msg->header;
    line_clouds_pub.publish(line_clouds_msg);

    computeCentroidsAndPublishDistances(line_clouds, input_msg->header);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : cloud->points) {
        if (point.z > Z_THRESHOLD) {
            obstacle_cloud->points.push_back(point);
        }
    }

    sensor_msgs::PointCloud2 obstacle_cloud_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_cloud_msg);
    obstacle_cloud_msg.header = input_msg->header;
    obstacle_cloud_pub.publish(obstacle_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_processor");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);
    
    green_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/green_cloud", 1);
    plane_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/plane_cloud", 1);
    line_clouds_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/line_cloud", 1);
    centroid_distance_pub = nh.advertise<std_msgs::Float32MultiArray>("/processed/centroid_distances", 1);
    centroid_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/centroid_cloud", 1);
    parking_spots_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/parking_spots", 1);
    obstacle_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed/obstacle_cloud", 1);

    ros::spin();
    return 0;
}

