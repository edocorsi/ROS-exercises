/* ---------------------------------------------------------------
 * 
 * Node control_turtle_pose
 * Edoardo Corsi
 * March 2020
 * 
 * ----------------------------------------------------------------
 */

// Includes
#include <iostream>
#include <math.h>
#include <time.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

// Structs
struct POSE
{
  float x;
  float y;
  float theta;
};

// Global variables
struct POSE robot_pose;
geometry_msgs::Twist cmd_msg;
ros::Publisher pub_vel;
float kp_theta = 1;

// Topic messages callback
void pose_callback(const turtlesim::Pose::ConstPtr& pose_msg)
{
    robot_pose.x = pose_msg->x;
    robot_pose.y = pose_msg->y;
    robot_pose.theta = pose_msg->theta;
}

// Function that creates random number receiving in input the range 
float create_random_number(int min, int max)
{
    float rand_number;
    rand_number = (rand() % (max*10 + 1))/10.0*2.0 + min;
    ROS_INFO("rand number generated: %f\n", rand_number);
    return rand_number;
}

// Function that allows to avoid obstacle
void avoid_obstacle()
{
    geometry_msgs::Twist cmd_msg;

    // Reorient turtle upwards
    ROS_INFO("correcting theta");
    
    // Specify the desidered rate of the internal control in Hz
    ros::Rate internal_rate(100);
    
    while (fabs(robot_pose.theta - M_PI/2.0) > 0.1)
    {
        cmd_msg.angular.z = kp_theta*(M_PI/2 - robot_pose.theta);
        pub_vel.publish(cmd_msg);

        // Consume available callbacks
        ros::spinOnce();
        // Wait the remaining time to let us hit our 100 Hz publish rate
        internal_rate.sleep();
    }

    // Move the robot upwards
    ROS_INFO("theta aligned, now moving away");
    cmd_msg.angular.z = 0;
    cmd_msg.linear.x = 2;
    pub_vel.publish(cmd_msg);
}

// Main function
int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_controller");
    ros::NodeHandle nh("~");

    // Specify the desidered rate to run in Hz. The node run at 1 Hz.
    ros::Rate rate(1);

    srand(time(NULL));

    // Create Subscriber and Publisher 
    ros::Subscriber robot_pose_sub = nh.subscribe("/turtle1/pose", 20, &pose_callback);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    sleep(1); // wait for adv-sub

    while(ros::ok())
    {
        // Limit to upper half
        if (robot_pose.y <= 6.0)
        {
            ROS_INFO("WARNING: collision alert");
            avoid_obstacle();
        }
        // Move randomly
        else
        {
            ROS_INFO("Moving randomly");
            // Create random control
            cmd_msg.linear.x = create_random_number(-2, 2);
            cmd_msg.angular.z = create_random_number(-2, 2);

            // Send command
            pub_vel.publish(cmd_msg);
        }
    
    // Consume available callbacks
    ros::spinOnce();
    // Wait the remaining time to let us hit our 100 Hz publish rate
    rate.sleep();
    }

    return 0;
}
