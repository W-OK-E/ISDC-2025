#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

using namespace cv;
using namespace std;

class DepthToCostmap {
public:
    DepthToCostmap() {
        // Subscribe to the depth image topic
        depth_sub_ = nh_.subscribe("/zed2i/zed_node/depth/depth_registered", 1, &DepthToCostmap::depthCallback, this);
        // Publisher for occupancy grid
        odom_sub_ = nh_.subscribe("/odom",1,&DepthToCostmap::odomCallback,this);
        occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
        occupancyGrid.header.stamp = ros::Time::now();
        occupancyGrid.header.frame_id = "map"; // Adjust frame_id as necessary
        occupancyGrid.info.resolution = 0.1; // Set resolution (meters per cell)
        occupancyGrid.info.width = 1500;
        occupancyGrid.info.height = 1500;
        occupancyGrid.info.origin.position.x = 750; // Adjust origin as necessary
        occupancyGrid.info.origin.position.y = -750;
        occupancyGrid.info.origin.position.z = 0;
        occupancyGrid.info.origin.orientation.w = 1.0;

        ROS_INFO("WELL BUMMER");
        occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

        for(int y = 0;y<1000;y++)
        {
            for(int x = 0;x<100;x++)
            {
                occupancyGrid.data[x + y*1000] = -1;
            }
        }
        // Optionally, initialize all values to -1 (unknown)
        // std::fill(occupancyGrid.data.begin(), occupancyGrid.data.end(), -1);

        
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher occupancy_grid_pub_;

    geometry_msgs::Point point_in_base_link;

    nav_msgs::OccupancyGrid occupancyGrid;

        // Set the header


    int start_x, start_y, new_x = 0, new_y = 0, delta_x  = 0,delta_y = 0,map_x = 0,map_y = 0;
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        
        try {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            Mat depthImage = cv_ptr->image;

            // Create a costmap (occupancy grid)
            Mat costmap(depthImage.size(), CV_32FC1, Scalar(0));
            // cout<<"Dimension of the Costmap is: "<<costmap.size()<<endl;
            // Parameters for depth normalization
            float minDepth = 5.0; // Minimum depth to consider for safe space
            float maxDepth = 8.0; // Maximum depth to consider for obstacles
            for (int y = 0; y < depthImage.rows; ++y) {
                for (int x = 0; x < depthImage.cols; ++x) {
                    float depth = depthImage.at<float>(y, x);
                    if(depth <minDepth)
                        minDepth = depth;
                    if(depth > maxDepth)
                        maxDepth = depth;   
                }
            }
            //Addin
            // Process each pixel
            for (int y = 0; y < depthImage.rows; ++y) {
                for (int x = 0; x < depthImage.cols; ++x) {
                    float depth = depthImage.at<float>(y, x);
                    float cost = normalizeDepth(depth, minDepth, maxDepth);

                    costmap.at<float>(y,x) = cost;
                }
            }

            // Publish the occupancy grid
            publishOccupancyGrid(costmap);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
        }
    }

    float normalizeDepth(float depth, float mindepth, float maxdepth)
    {
        return (depth/(maxdepth - mindepth));
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) 
    {

        new_x = msg->pose.pose.position.x;
        new_y = msg->pose.pose.position.y;
        delta_x = new_x - start_x;
        delta_y = new_y - start_y;
        start_x = new_x;
        start_y = new_y;
        // point_in_base_link.x = start_x;
        // point_in_base_link.y = start_y;
        map_x += delta_x - 320;
        map_y += delta_y - 160;
    }

    void publishOccupancyGrid(const Mat& costmap) {
        // Set the header
        geometry_msgs::TransformStamped transformStamped;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        
        try 
        {
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time::now());


            // Transform the point to map frame
            geometry_msgs::Point point_in_map;

            tf2::doTransform(point_in_base_link, point_in_map, transformStamped);
            start_y = static_cast<int>(point_in_map.x);
            start_x = static_cast<int>(point_in_map.y);
        }
        catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        }
        occupancyGrid.header.stamp = ros::Time::now();
        ROS_INFO("PUBLISHING OCCUPANCY GRID");

        // Fill occupancy grid data
        for (int y = map_y; y < map_y + costmap.rows; ++y) {
            for (int x = map_x; x < map_x + costmap.cols; ++x) {
        // Ensure indices are within bounds
                if (y < 0 || y >= occupancyGrid.info.height || 
                    x < 0 || x >= occupancyGrid.info.width) {
                    std::cerr << "Index out of bounds: y = " << y << ", x = " << x << std::endl;
                    continue; // Skip this iteration if out of bounds
                }

                float cost = costmap.at<float>(y - map_y, x - map_x);
                int ind = y * occupancyGrid.info.width + x; // Use occupancyGrid width

                occupancyGrid.data[ind] = cost * 100; // Use the correct index
            }
        }
        for (int y = 1300; y < 1495 ; ++y) {
            for (int x = 1300; x < 1495; ++x){
                 int ind = y * occupancyGrid.info.width + x;
                occupancyGrid.data[ind] = 100;
            }
        }
        occupancy_grid_pub_.publish(occupancyGrid);
        ROS_INFO("Occupancy grid published.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_to_costmap_node");
    DepthToCostmap dtcm;
    ros::spin();
    return 0;
}

