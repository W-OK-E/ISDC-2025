#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>

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
        occupancyGrid.info.width = 1000;
        occupancyGrid.info.height = 1000;
        occupancyGrid.info.origin.position.x = 0; // Adjust origin as necessary
        occupancyGrid.info.origin.position.y = 0;
        occupancyGrid.info.origin.position.z = 0;
        occupancyGrid.info.origin.orientation.w = 1.0;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher occupancy_grid_pub_;

    nav_msgs::OccupancyGrid occupancyGrid;

        // Set the header


    int start_x, start_y;
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            Mat depthImage = cv_ptr->image;

            // Create a costmap (occupancy grid)
            Mat costmap(depthImage.size(), CV_32FC1, Scalar(0));

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
            // Process each pixel
            for (int y = 0; y < depthImage.rows; ++y) {
                for (int x = 0; x < depthImage.cols; ++x) {
                    float depth = depthImage.at<float>(y, x);

                    // Check for valid depth value
                    if (depth > 0) {
                        // Normalize depth to cost
                        float cost = normalizeDepth(depth, minDepth, maxDepth);
                        costmap.at<float>(y, x) = cost;
                    } else {
                        // Mark unknown depth as high cost
                        costmap.at<float>(y, x) = 1.0; // High cost for unknown
                    }
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
        start_x = msg->pose.pose.position.x;
        start_y = msg->pose.pose.position.y;
    }

    void publishOccupancyGrid(const Mat& costmap) {
        nav_msgs::OccupancyGrid occupancyGrid;

        // Set the header
        occupancyGrid.header.stamp = ros::Time::now();

        // Resize the data vector


        // Fill occupancy grid data
        for (int y = start_y; y < start_y + costmap.rows; ++y) {
            for (int x = start_x; x < start_x + costmap.cols; ++x) {
                float cost = costmap.at<float>(y-start_y, x - start_x);
                // if (cost < 0.5) {
                //     occupancyGrid.data[y * costmap.cols + x] = 0; // Free
                // } else {
                //     occupancyGrid.data[y * costmap.cols + x] = 100; // Occupied
                // }
                occupancyGrid.data[y * costmap.cols + x] = cost*100;
            }
        }

        // Publish the occupancy grid
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

