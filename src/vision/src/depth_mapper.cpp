#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>       
#include <sensor_msgs/Image.h>                                                                                                                                             
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#include <iostream>
#include <cmath>
#include <vector>

const float RESOLUTION = 0.05;
const int WIDTH = 3000;
const int HEIGHT = 3000;
const float ORIGIN_X = -(WIDTH*RESOLUTION)/2.0;
const float ORIGIN_Y = -(HEIGHT*RESOLUTION)/2.0;

struct BotPosition {
    double x, y, z;
    double yaw, roll, pitch;
};

struct GridCell {
    int x, y;
};

struct OccupancyGrid {
    double resolution;
    double origin_x, origin_y;
    int width, height;
};

// Global variables
BotPosition current_bot_pose;
nav_msgs::OccupancyGrid occupancy_grid;
std::vector<GridCell> obstacle_cells;
std::vector<GridCell> non_obstacle_cells;
ros::Publisher debug_obstacle_pub;
ros::Publisher debug_non_obstacle_pub;

pcl::PointXYZ cloudPointToGlobalPoint(const pcl::PointXYZ& cloud_point, const BotPosition& bot_pos) {
    pcl::PointXYZ global_point;

    global_point.x = bot_pos.x + cloud_point.x*sin(bot_pos.roll) + cloud_point.z*cos(bot_pos.roll);
    global_point.y = bot_pos.y + cloud_point.z*sin(bot_pos.roll) - cloud_point.x*cos(bot_pos.roll);
    global_point.z = cloud_point.y;

    return global_point;
}

GridCell globalPointToMapIndex(const pcl::PointXYZ& global_point, const OccupancyGrid& grid) {
    int x = static_cast<int>(round((global_point.x - grid.origin_x)/grid.resolution));
    int y = static_cast<int>(round((global_point.y - grid.origin_y)/grid.resolution));

    return {x, y};
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    std::cout << "hello noobs" << std::endl;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert ROS PointCloud2 message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Create debug point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_non_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Set the frame IDs
    debug_obstacle_cloud->header.frame_id = "map";
    debug_non_obstacle_cloud->header.frame_id = "map";

    // Create OccupancyGrid struct for conversion
    OccupancyGrid grid = {
        RESOLUTION,
        ORIGIN_X,
        ORIGIN_Y,
        WIDTH,
        HEIGHT
    };

    // Clear previous data
    obstacle_cells.clear();
    non_obstacle_cells.clear();
    
    // Process each point in the cloud
    for (const auto& point : cloud->points) {
        // Skip invalid points
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // Convert point to global frame
        pcl::PointXYZ global_point = cloudPointToGlobalPoint(point, current_bot_pose);

        // Convert global point to grid cell
        GridCell cell = globalPointToMapIndex(global_point, grid);

        // Check if cell is within grid bounds
        if (cell.x >= 0 && cell.x < WIDTH && cell.y >= 0 && cell.y < HEIGHT) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = global_point.x;
            colored_point.y = global_point.y;
            colored_point.z = global_point.z;

            // Categorize points based on height (z-coordinate)
            if (global_point.z > 2.0) {  // Points higher than 2 meters
                non_obstacle_cells.push_back(cell);
                // Green for non-obstacles
                colored_point.r = 255;
                colored_point.g = 0;
                colored_point.b = 0;
                debug_non_obstacle_cloud->points.push_back(colored_point);
            } else {  // Points lower than 2 meters
                obstacle_cells.push_back(cell);
                // Red for obstacles
                colored_point.r = 0;
                colored_point.g = 0;
                colored_point.b = 255;
                debug_obstacle_cloud->points.push_back(colored_point);
            }
        }
    }

    // Update occupancy grid
    for (const auto& cell : obstacle_cells) {
        int index = cell.y * WIDTH + cell.x;
        if (index >= 0 && index < occupancy_grid.data.size()) {
            occupancy_grid.data[index] = 100;  // 100 represents occupied cell
        }
    }

    for (const auto& cell : non_obstacle_cells) {
        int index = cell.y * WIDTH + cell.x;
        if (index >= 0 && index < occupancy_grid.data.size()) {
            occupancy_grid.data[index] = 0;  // 0 represents free cell
        }
    }

    // Publish debug point clouds
    debug_obstacle_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    debug_non_obstacle_cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());

    sensor_msgs::PointCloud2 obstacle_msg;
    sensor_msgs::PointCloud2 non_obstacle_msg;
    
    pcl::toROSMsg(*debug_obstacle_cloud, obstacle_msg);
    pcl::toROSMsg(*debug_non_obstacle_cloud, non_obstacle_msg);

    debug_obstacle_pub.publish(obstacle_msg);
    debug_non_obstacle_pub.publish(non_obstacle_msg);
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_bot_pose.x = msg->pose.pose.position.x;
    current_bot_pose.y = msg->pose.pose.position.y;
    current_bot_pose.z = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3(q).getRPY(current_bot_pose.roll, current_bot_pose.pitch, current_bot_pose.yaw);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper_node");
    ros::NodeHandle nh;

    // Initialize occupancy grid
    occupancy_grid.header.frame_id = "base_link";
    occupancy_grid.info.resolution = RESOLUTION;
    occupancy_grid.info.width = WIDTH;
    occupancy_grid.info.height = HEIGHT;
    occupancy_grid.info.origin.position.x = ORIGIN_X;   
    occupancy_grid.info.origin.position.y = ORIGIN_Y;
    occupancy_grid.info.origin.position.z = 0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data.resize(WIDTH * HEIGHT, -1);

    // Set up subscribers and publishers
    ros::Subscriber pc_sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);
    ros::Subscriber image_sub = nh.subscribe("/zed2i/zed_node/depth/depth_registered", 1, depthImageCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odometryCallback);
    ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);
    
    // Add debug publishers
    debug_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/obstacle_points", 1);
    debug_non_obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("/debug/non_obstacle_points", 1);

    ros::Rate rate(10);
    while (ros::ok()) {
        occupancy_grid.header.stamp = ros::Time::now();
        grid_pub.publish(occupancy_grid);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}