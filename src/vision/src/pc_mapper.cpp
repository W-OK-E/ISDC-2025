#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

ros::Publisher map_pub;

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

const double resolution = 0.06;

const double P_PRIOR = 0.5;  
const double P_OCC = 0.7;    
const double P_FREE = 0.3;   
const double L_OCC = log(P_OCC / (1 - P_OCC));
const double L_FREE = log(P_FREE / (1 - P_FREE));
const double L_PRIOR = log(P_PRIOR / (1 - P_PRIOR));
const double L_MIN = -5;  
const double L_MAX = 5;   

pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr unoccupied_cloud(new pcl::PointCloud<pcl::PointXYZ>);

pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> occupied_octree(static_cast<float>(resolution)); 
vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> occupied_centers;

pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> unoccupied_octree(static_cast<float>(resolution)); 
vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> unoccupied_centers;

BotPosition current_bot_pose;
OccupancyGrid grid;

bool cloud_received = false;
bool odom_received = false;

vector<double> log_odds_map;
vector<int> hit_count;
vector<double> update_sum;

pcl::PointXYZ cloudPointToGlobalPoint(const pcl::PointXYZ& cloud_point, const BotPosition& bot_pos) {
    pcl::PointXYZ global_point;

    global_point.x = bot_pos.x + cloud_point.x*sin(bot_pos.yaw) + cloud_point.z*cos(bot_pos.yaw);
    global_point.y = bot_pos.y + cloud_point.z*sin(bot_pos.yaw) - cloud_point.x*cos(bot_pos.yaw);
    global_point.z = cloud_point.y;

    return global_point;
}

GridCell globalPointToPixelIndex(const pcl::PointXYZ& global_point, const OccupancyGrid& grid) {
    int x = static_cast<int>(round((global_point.x - grid.origin_x)/grid.resolution));
    int y = static_cast<int>(round((global_point.y - grid.origin_y)/grid.resolution));

    return {x, y};
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *current_cloud);

    occupied_cloud->clear();
    unoccupied_cloud->clear();
    
    for (int i = current_cloud->size() - 1; i >= 0; i--) {
        pcl::PointXYZ cloud_point;

        uint8_t r = current_cloud->points[i].r;
        uint8_t g = current_cloud->points[i].g;
        uint8_t b = current_cloud->points[i].b;

        float x = current_cloud->points[i].x;
        float y = current_cloud->points[i].y;
        float z = current_cloud->points[i].z;

        cloud_point.x = x;
        cloud_point.y = y;
        cloud_point.z = z;

        if (x < 6  && x > -6 && z < 8) {
            if ((r > 150 && g > 150 && b > 150 && z < 8)) {
                if (i%1 == 0)
                    occupied_cloud->points.push_back(cloud_point);
            } 
            else 
            {
                if (i%2 == 0)
                    unoccupied_cloud->points.push_back(cloud_point);
            }
        }
    }

    occupied_octree.deleteTree();
    occupied_octree.setInputCloud(occupied_cloud);
    occupied_octree.addPointsFromInputCloud();

    unoccupied_octree.deleteTree();
    unoccupied_octree.setInputCloud(unoccupied_cloud);
    unoccupied_octree.addPointsFromInputCloud();

    cloud_received = true;
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
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

    odom_received = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_mapper");
    ros::NodeHandle nh;

    ROS_INFO("PointCloud Mapper Node Started.");

    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed2i/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1, true);

    ros::Rate loop_rate(100);

    grid.width = 1500;
    grid.height = 1000;
    grid.resolution = resolution;
    grid.origin_x = -(grid.width/2)*grid.resolution;
    grid.origin_y = -(grid.height/2)*grid.resolution;

    nav_msgs::OccupancyGrid map;
    map.info.resolution = grid.resolution;
    map.info.width = grid.width;
    map.info.height = grid.height;
    map.info.origin.position.x = grid.origin_x;
    map.info.origin.position.y = grid.origin_y;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(grid.width * grid.height, -1);

    log_odds_map.resize(grid.width * grid.height, L_PRIOR);
    hit_count.resize(grid.width * grid.height, 0);
    update_sum.resize(grid.width * grid.height, 0.0);

    while(ros::ok()) {
        if (odom_received && cloud_received) {

            fill(hit_count.begin(), hit_count.end(), 0);
            fill(update_sum.begin(), update_sum.end(), 0.0);

            unoccupied_centers.clear();
            unoccupied_octree.getOccupiedVoxelCenters(unoccupied_centers);

            occupied_centers.clear();
            occupied_octree.getOccupiedVoxelCenters(occupied_centers);

            for (const auto& point : unoccupied_centers) {
                pcl::PointXYZ global_point = cloudPointToGlobalPoint(point, current_bot_pose);
                GridCell indices = globalPointToPixelIndex(global_point, grid);

                if (indices.x >= 0 && indices.x < grid.width && indices.y >= 0 && indices.y < grid.height) {
                    int index = indices.y * grid.width + indices.x;
                    update_sum[index] += L_FREE;
                    hit_count[index]++;
                }
            }

            for (const auto& point : occupied_centers) {
                pcl::PointXYZ global_point = cloudPointToGlobalPoint(point, current_bot_pose);
                GridCell indices = globalPointToPixelIndex(global_point, grid);

                if (indices.x >= 0 && indices.x < grid.width && indices.y >= 0 && indices.y < grid.height) {
                    int index = indices.y * grid.width + indices.x;
                    update_sum[index] += L_OCC;
                    hit_count[index]++;
                }
            }

            for (size_t i = 0; i < log_odds_map.size(); ++i) {
                if (hit_count[i] > 0) {
                    double avg_update = update_sum[i] / hit_count[i];
                    log_odds_map[i] = max(L_MIN, min(L_MAX, log_odds_map[i] + avg_update));
                }
            }

            for (size_t i = 0; i < map.data.size(); ++i) {
                double prob = 1 - (1 / (1 + exp(log_odds_map[i])));
                if (prob > 0.7) {
                    map.data[i] = 100;
                } else if (prob < 0.3) {
                    map.data[i] = 0;
                } else {
                    map.data[i] = -1;
                }
            }

            map.header.stamp = ros::Time::now();
            map.header.frame_id = "map";
            map_pub.publish(map);   

            // Reset flags
            cloud_received = false;
            odom_received = false;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}