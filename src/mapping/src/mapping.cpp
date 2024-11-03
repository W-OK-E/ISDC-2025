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
        resolution_ = 0.01;
        width_ = 1500;
        height_ = 1500;
        origin_x = -(width_/2) * resolution_;
        origin_y = -(height_/2) * resolution_;

        occupancyGrid.info.resolution = resolution_; // Set resolution (meters per cell)
        occupancyGrid.info.width = width_;
        occupancyGrid.info.height = height_;
        occupancyGrid.info.origin.position.x = origin_x; // Adjust origin as necessary
        occupancyGrid.info.origin.position.y = origin_y;
        occupancyGrid.info.origin.position.z = 0;
        occupancyGrid.info.origin.orientation.w = 1.0;

        ROS_INFO("WELL BUMMER");
        occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);
        std::fill(occupancyGrid.data.begin(), occupancyGrid.data.end(), -1); //Initializing the Grid

        
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
    int delta_x  = 0,delta_y = 0,map_x = 0,map_y = 0,origin_x = 0,origin_y = 0;
    float resolution_ = 1, width_ = 0, height_ = 0;
    geometry_msgs::Point new_pos, old_pos;
    
    geometry_msgs::Point globalPointToPixelIndex(const geometry_msgs::Point& global_point, const nav_msgs::OccupancyGrid grid) 
    {
        int x = static_cast<int>(round((global_point.x - grid.info.origin.position.x)/grid.info.resolution));
        int y = static_cast<int>(round((global_point.y - grid.info.origin.position.y)/grid.info.resolution));

        geometry_msgs::Point p;
        p.x = x;
        p.y = y;

        return p;
    }

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

        occupancyGrid.header.stamp = ros::Time::now();

        cout<<"CHange in x:"<<delta_x<<endl;
        int origin_x = 750-320;// + delta_x;//half of the image's columns, 
        int origin_y = 750-180;//half of the image's rows ;
        for (int x = 0; x < depthImage.cols; ++x) {
            for (int y = 0; y < depthImage.rows; ++y) {
        // Ensure indices are within bounds
                if (y + delta_y < 0 || y + delta_y >= occupancyGrid.info.height || 
                    x + delta_x < 0 || x + delta_x >= occupancyGrid.info.width) {
                    // std::cerr << "Index out of bounds: y = " << y << ", x = " << x << std::endl;
                    continue; // Skip this iteration if out of bounds
                }
                // geometry_msgs::Point p_ind = globalPointToPixelIndex(global_point, occupancyGrid);
                float cost = normalizeDepth(depthImage.at<float>(y , x ),minDepth,maxDepth);
                // int ind = global_point.y * occupancyGrid.info.width + global_point.x; // Use occupancyGrid width
                // cout<<x+delta_x<<" "<<y+delta_y<<endl;
                int ind = (y + origin_y) + (origin_x + x) * width_ ; 
                // cout<<"Index:"<<ind<<endl;
                occupancyGrid.data[ind] = cost * 100; // Use the correct index
            }
        }


        occupancy_grid_pub_.publish(occupancyGrid);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
        }
    }

    float normalizeDepth(float depth, float mindepth, float maxdepth)
    {
        return (depth/(maxdepth - mindepth));
    }

    geometry_msgs::Point worldToGrid(geometry_msgs::Point P) const {

        P.x = static_cast<int>((P.x - origin_x) / resolution_);
        P.y = static_cast<int>((P.y - origin_y) / resolution_);
        return P;
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
        
        new_pos = msg->pose.pose.position;
        cout<<"Inside Odom "<<msg->pose.pose.position.x<<" "<<msg->pose.pose.position.y<<endl;
        delta_x = int((abs(new_pos.x) - abs(old_pos.x))*215);
        delta_y = abs(new_pos.y - old_pos.y);
        
        old_pos = new_pos;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_to_costmap_node");
    DepthToCostmap dtcm;
    ros::spin();
    return 0;
}





