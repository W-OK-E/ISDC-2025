#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

ros::Publisher pub;
tf2_ros::Buffer tf_buffer;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
{
    try {
        // Get transform from input cloud frame to map
        geometry_msgs::TransformStamped transform_stamped = 
            tf_buffer.lookupTransform("map", input_cloud->header.frame_id, 
                                    ros::Time(0), ros::Duration(1.0));
        
        // Convert quaternion from transform
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        
        // Extract roll, pitch, yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // Create new quaternion with only yaw (zeroing roll and pitch)
        tf2::Quaternion q_modified;
        q_modified.setRPY(0, 0, yaw);
        
        // Create modified transform
        geometry_msgs::TransformStamped modified_transform = transform_stamped;
        modified_transform.transform.rotation.x = q_modified.x();
        modified_transform.transform.rotation.y = q_modified.y();
        modified_transform.transform.rotation.z = q_modified.z();
        modified_transform.transform.rotation.w = q_modified.w();

        // Convert to Eigen matrix directly
        Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
        
        // Set rotation
        tf2::Matrix3x3 rot_matrix(q_modified);
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                transform_matrix(i,j) = rot_matrix[i][j];
            }
        }
        
        // Set translation
        transform_matrix(0,3) = modified_transform.transform.translation.x;
        transform_matrix(1,3) = modified_transform.transform.translation.y;
        transform_matrix(2,3) = modified_transform.transform.translation.z;
        
        // Transform point cloud
        sensor_msgs::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(transform_matrix, *input_cloud, transformed_cloud);

        std::cout << "hello" << std::endl;
        
        // Update frame_id and publish
        transformed_cloud.header.frame_id = "map";
        transformed_cloud.header.stamp = input_cloud->header.stamp;
        pub.publish(transformed_cloud);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_tf");
    ros::NodeHandle nh;
    
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    pub = nh.advertise<sensor_msgs::PointCloud2>("flattened_cloud", 1);
    ros::Subscriber sub = nh.subscribe("/zed2i/zed_node/point_cloud/cloud_registered", 1, pointCloudCallback);
    
    ros::spin();
    return 0;
}