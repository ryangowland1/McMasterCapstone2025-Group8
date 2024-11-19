#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>

// Global configuration
const std::string INPUT_FILE = "Data/point_cloud_PCD_23310350_1080_23-10-2024-16-19-14.pcd";
const float GREEN_THRESHOLD = 1.2f;
const float PLANE_DISTANCE_THRESHOLD = 0.02f;          // Maximum distance from point to plane for inliers
const float INITIAL_LINE_DISTANCE_THRESHOLD = 0.01f;   // Starting distance for line inliers
const int STATISTICAL_NEIGHBORS = 50;                  // Number of neighbors for statistical filtering
const float STANDARD_DEV_MULTIPLIER = 2.0f;            // Standard deviation threshold multiplier
const int MAX_LINES = 5;                               // Maximum number of lines to try fitting
const float POINT_PROPORTION_THRESHOLD = 0.1f;         // Minimum proportion of points for a line

struct Point {
	float x, y, z;
	uint8_t r, g, b;
};

// Function to fit a plane and filter outliers
pcl::PointCloud<pcl::PointXYZRGB>::Ptr fitPlaneAndFilterOutliers(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(PLANE_DISTANCE_THRESHOLD);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.empty()) {
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (const auto& idx : inliers->indices) {
		plane_cloud->points.push_back(cloud->points[idx]);
	}
	plane_cloud->width = plane_cloud->points.size();
	plane_cloud->height = 1;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	sor.setInputCloud(plane_cloud);
	sor.setMeanK(STATISTICAL_NEIGHBORS);
	sor.setStddevMulThresh(STANDARD_DEV_MULTIPLIER);
	sor.filter(*filtered_cloud);

	return filtered_cloud;
}

// Function to fit multiple lines on the plane-filtered cloud
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fitMultipleLines(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud) {

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> line_clouds;
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(plane_cloud);

	float line_distance_threshold = INITIAL_LINE_DISTANCE_THRESHOLD;
	int detected_lines = 0;

	// needed as plane_cloud->points.size() is updated every iteration
	int total_points = plane_cloud->width;

	for (int i = 0; i < MAX_LINES; ++i) {
		pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(line_distance_threshold);
		seg.setInputCloud(plane_cloud);
		seg.segment(*inliers, *line_coefficients);

		// Reject any line that has less than POINT_PROPORTION_THRESHOLD * 100% of total points
		if (inliers->indices.size() < POINT_PROPORTION_THRESHOLD * total_points) {
			std::cerr << "No more lines can be fitted on the plane after " << detected_lines << " lines." << std::endl;
			break;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*line_cloud);

		line_clouds.push_back(line_cloud);
		detected_lines++;

		extract.setNegative(true);
		extract.filter(*plane_cloud);

		std::cout << "Line " << detected_lines << " coefficients: "
			<< line_coefficients->values[0] << " "
			<< line_coefficients->values[1] << " "
			<< line_coefficients->values[2] << " "
			<< line_coefficients->values[3] << " "
			<< line_coefficients->values[4] << " "
			<< line_coefficients->values[5] << std::endl;

		// Adjust line fitting threshold slightly for the next line to capture different lines better
		line_distance_threshold *= 1.1;  // Increase threshold slightly for next iteration
	}

	return line_clouds;
}

// Visualization function
void visualizeCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& original_cloud,
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& green_cloud,
	const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& line_clouds) {

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	int v1(0), v2(1), v3(2);
	viewer->createViewPort(0.0, 0.0, 0.33, 1.0, v1);
	viewer->createViewPort(0.33, 0.0, 0.66, 1.0, v2);
	viewer->createViewPort(0.66, 0.0, 1.0, 1.0, v3);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_original(original_cloud);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_green(green_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(original_cloud, rgb_original, "original_cloud", v1);
	viewer->addPointCloud<pcl::PointXYZRGB>(green_cloud, rgb_green, "green_cloud", v2);

	if (!line_clouds.empty()) {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> line1_color(line_clouds[0], 255, 0, 0); // Red
		viewer->addPointCloud<pcl::PointXYZRGB>(line_clouds[0], line1_color, "line_cloud1", v3);
	}
	if (line_clouds.size() > 1) {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> line2_color(line_clouds[1], 0, 255, 0); // Green
		viewer->addPointCloud<pcl::PointXYZRGB>(line_clouds[1], line2_color, "line_cloud2", v3);
	}
	if (line_clouds.size() > 2) {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> line3_color(line_clouds[2], 0, 0, 255); // Blue
		viewer->addPointCloud<pcl::PointXYZRGB>(line_clouds[2], line3_color, "line_cloud3", v3);
	}

	viewer->addText("Original Cloud", 10, 10, "original_label", v1);
	viewer->addText("Green Points", 10, 10, "green_label", v2);
	viewer->addText("Line-Fitted Points", 10, 10, "line_label", v3);

	viewer->initCameraParameters();
	viewer->resetCamera();

	std::cout << "Visualizing point clouds. Press 'q' to exit..." << std::endl;

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

int main() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(INPUT_FILE, *cloud) == -1) {
		std::cerr << "Couldn't read file " << INPUT_FILE << std::endl;
		return -1;
	}

	// Convert PCL cloud to vector of custom points
	std::vector<Point> points;
	for (const auto& pcl_point : cloud->points) {
		uint32_t rgb = *reinterpret_cast<const int*>(&pcl_point.rgb);
		Point p;
		p.x = pcl_point.x;
		p.y = pcl_point.y;
		p.z = pcl_point.z;
		p.r = (rgb >> 16) & 0xFF;
		p.g = (rgb >> 8) & 0xFF;
		p.b = rgb & 0xFF;
		points.push_back(p);
	}

	// Filter points based on green color dominance
	auto it = std::remove_if(points.begin(), points.end(),
		[](const Point& p) {
			return !(p.g > GREEN_THRESHOLD * p.r && p.g > GREEN_THRESHOLD * p.b);
		});
	points.erase(it, points.end());

	// Convert filtered points back to PCL cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	green_cloud->width = points.size();
	green_cloud->height = 1;
	green_cloud->points.resize(green_cloud->width * green_cloud->height);

	for (size_t i = 0; i < points.size(); ++i) {
		green_cloud->points[i].x = points[i].x;
		green_cloud->points[i].y = points[i].y;
		green_cloud->points[i].z = points[i].z;
		uint32_t rgb = ((uint32_t)points[i].r << 16 | (uint32_t)points[i].g << 8 | (uint32_t)points[i].b);
		green_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
	}

	// Fit plane and remove outliers
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_filtered_cloud = fitPlaneAndFilterOutliers(green_cloud);

	// Fit multiple lines on the plane-filtered points
	auto line_clouds = fitMultipleLines(plane_filtered_cloud);

	// Visualize all stages
	visualizeCloud(cloud, green_cloud, line_clouds);

	return 0;
}