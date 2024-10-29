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
        occupancyGrid.info.resolution = 0.5; // Set resolution (meters per cell)
        occupancyGrid.info.width = 1500;
        occupancyGrid.info.height = 1500;
        occupancyGrid.info.origin.position.x = -750 * 0.05; // Adjust origin as necessary
        occupancyGrid.info.origin.position.y = -750 * 0.05;
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


    struct BotPosition {
        double x, y, z;
        double yaw, roll, pitch;
    };
    BotPosition bot_pose;
    int start_x, start_y, new_x = 0, new_y = 0, delta_x  = 0,delta_y = 0,map_x = 0,map_y = 0;
    
    geometry_msgs::Point globalPointToPixelIndex(const geometry_msgs::Point& global_point, const nav_msgs::OccupancyGrid grid) 
    {
        int x = static_cast<int>(round((global_point.x - grid.info.origin.position.x)/grid.info.resolution));
        int y = static_cast<int>(round((global_point.y - grid.info.origin.position.y)/grid.info.resolution));

        geometry_msgs::Point p;
        p.x = x;
        p.y = y;

        return p;
    }
    // geometry_msgs::Point depth_to_global(vector<int> depth_point, const BotPosition& bot_pos) 
    // {
    //     geometry_msgs::Point global_point;
    //     global_point.x = bot_pos.x + depth_point.x*sin(bot_pos.yaw) + depth_point.z*cos(bot_pos.yaw);
    //     global_point.y = bot_pos.y + depth_point.z*sin(bot_pos.yaw) - depth_point.x*cos(bot_pos.yaw);

    //     return global_point;
    // }
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        
        try {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            Mat depthImage = cv_ptr->image;

            // Create a costmap (occupancy grid)
            Mat costmap(depthImage.size(), CV_32FC1, Scalar(0));
            cout<<"Dimension of the Costmap is: "<<costmap.size()<<endl;
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
        bot_pose.x = msg->pose.pose.position.x;
        bot_pose.y = msg->pose.pose.position.y;
        bot_pose.z = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
            );
        tf2::Matrix3x3(q).getRPY(bot_pose.roll, bot_pose.pitch, bot_pose.yaw);

        new_x = msg->pose.pose.position.x;
        new_y = msg->pose.pose.position.y;
        delta_x = abs(new_x - start_x);
        delta_y = abs(new_y - start_y);
        start_x = new_x;
        start_y = new_y;
        // point_in_base_link.x = start_x;
        // point_in_base_link.y = start_y;
        map_x += delta_x - 320;
        map_y += delta_y - 160;
    }

    void odomCallback2(const nav_msgs::Odometry::ConstPtr& msg)
    {
        float bot_x = msg->
    }
    void publishOccupancyGrid(const Mat& costmap) {
        // Set the header
        geometry_msgs::TransformStamped transformStamped;

        occupancyGrid.header.stamp = ros::Time::now();
        ROS_INFO("Publishing Grid");
        for (int y = 0; y < costmap.rows; ++y) {
            for (int x = 0; x < costmap.cols; ++x) {
        // Ensure indices are within bounds
                if (y < 0 || y >= occupancyGrid.info.height || 
                    x < 0 || x >= occupancyGrid.info.width) {
                    // std::cerr << "Index out of bounds: y = " << y << ", x = " << x << std::endl;
                    continue; // Skip this iteration if out of bounds
                }
                geometry_msgs::Point global_point;
                global_point.x = x;
                global_point.y = y;
                // geometry_msgs::Point p_ind = globalPointToPixelIndex(global_point, occupancyGrid);
                float cost = costmap.at<float>(y , x );
                // int ind = global_point.y * occupancyGrid.info.width + global_point.x; // Use occupancyGrid width
                int ind = y * occupancyGrid.info.width + x; 
                occupancyGrid.data[ind] = cost * 100; // Use the correct index
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

