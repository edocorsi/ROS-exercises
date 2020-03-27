/* ---------------------------------------------------------------
 * 
 * Node turtle_pose_to_rviz
 * Edoardo Corsi
 * March 2020
 * 
 * ----------------------------------------------------------------
 */

// Includes
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include "turtlesim/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

// Global variables
geometry_msgs::PoseStamped point_msg;
nav_msgs::Odometry odom_msg;
tf2::Quaternion turtle_quaternion;

// Topic messages callback
void pose_callback(const turtlesim::Pose::ConstPtr& pose_msg)
{
    // Create turtle quaternion from roll/pitch/yaw (in radians)
    turtle_quaternion.setRPY(0,0,pose_msg->theta);
    
    // Compose message to the topic /turtle_pose_to_display/pose
    point_msg.header.frame_id = "/world_frame";
    point_msg.header.stamp = ros::Time::now();
    point_msg.pose.position.x = pose_msg->x;
    point_msg.pose.position.y = pose_msg->y;
    
    point_msg.pose.orientation.x = turtle_quaternion[0];
    point_msg.pose.orientation.y = turtle_quaternion[1];
    point_msg.pose.orientation.z = turtle_quaternion[2];
    point_msg.pose.orientation.w = turtle_quaternion[3];
    
    // Compose message to the topic /turtle_pose_to_display/odom
    odom_msg.header.frame_id = "/world_frame";
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.position.x = pose_msg->x;
    odom_msg.pose.pose.position.y = pose_msg->y;
    
    odom_msg.pose.pose.orientation.x = turtle_quaternion[0];
    odom_msg.pose.pose.orientation.y = turtle_quaternion[1];
    odom_msg.pose.pose.orientation.z = turtle_quaternion[2];
    odom_msg.pose.pose.orientation.w = turtle_quaternion[3];
    
}

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_pose_to_rviz");
    ros::NodeHandle node("~");
    
     // Specify the desidered rate to run in Hz. The node run at 100 Hz.
    ros::Rate rate(100); 
    
    // Create Subscriber and Publishers 
    ros::Subscriber sub_pose = node.subscribe("/turtle1/pose", 1000, &pose_callback);

    ros::Publisher pub_pose = node.advertise<geometry_msgs::PoseStamped>("/turtle_pose_to_display/pose",100);
    ros::Publisher pub_odom = node.advertise<nav_msgs::Odometry>("/turtle_pose_to_display/odom",100);
     
    while (ros::ok()) 	// Keep spinning loop until user presses Ctrl+C
	{
        // Publish messages 
        pub_pose.publish(point_msg);
        pub_odom.publish(odom_msg);             

        // Consume available callbacks
        ros::spinOnce();
        // Wait the remaining time to let us hit our 100 Hz publish rate
        rate.sleep();       

    }

    return 0;
    
}
