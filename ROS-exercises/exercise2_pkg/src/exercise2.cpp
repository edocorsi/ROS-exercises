/* ---------------------------------------------------------------
 * 
 * Node controller_node
 * Edoardo Corsi
 * March 2020
 * 
 * ----------------------------------------------------------------
 */

// Includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "exercise2_pkg/robot_velocity.h"

// Global variables
float min_long_vel;
float max_long_vel;
float min_yaw_rate;
float max_yaw_rate;
float input_long_vel;
float input_yaw_rate;

// Topic messages callback
void controller_callback(const exercise2_pkg::robot_velocity::ConstPtr& msg)
{
    input_long_vel = msg->longitudinal_velocity;
    input_yaw_rate = msg->yaw_rate;
}

// Function that checks limits to apply saturation
float saturation_function(float v_min, float v_max, float v)
{
    if (v > v_max)
        
        v = v_max;
        
    else if (v < v_min)
        
        v = v_min;
    
    return v;        
}

// Main function
int main(int argc, char **argv)
{
	ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh("~");
    
    geometry_msgs::Twist cmd_msg;
    
    // Specify the desidered rate to run in Hz. The node run at 100 Hz.
    ros::Rate rate(100);

    // Receive minimum saturation value for longitudinal velocity as parameter 
    nh.getParam("sat_min_long_vel", min_long_vel);
    ROS_INFO("Found parameter: min saturation value for longitudinal velocity set to %.3f m/s", min_long_vel);
     
    // Receive maximum saturation value for longitudinal velocity as parameter 
    nh.getParam("sat_max_long_vel", max_long_vel);
	ROS_INFO("Found parameter: max saturation value for longitudinal velocity set to %.3f m/s", max_long_vel);
	
    // Receive minimum saturation value for yaw rate as parameter 
	nh.getParam("sat_min_yaw_rate", min_yaw_rate);
	ROS_INFO("Found parameter: min saturation value for yaw rate set to %.3f rad/s", min_yaw_rate);
		
    // Receive maximum saturation value for yaw rate as parameter 
	nh.getParam("sat_max_yaw_rate", max_yaw_rate);
	ROS_INFO("Found parameter: max saturation value for yaw rate set to %.3f rad/s", max_yaw_rate);
	
    // Create Subscriber and Publisher
    ros::Subscriber sub_contr = nh.subscribe("/control_velocity",100, &controller_callback);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000); 
    
    while (ros::ok()) 	// Keep spinning loop until user presses Ctrl+C
	{
        cmd_msg.linear.x = saturation_function(min_long_vel, max_long_vel, input_long_vel);
        cmd_msg.angular.z = saturation_function(min_yaw_rate, max_yaw_rate, input_yaw_rate);
        
        // Publish the message
        pub_vel.publish(cmd_msg);
        
        // Consume available callbacks
        ros::spinOnce();
       
        // Wait the remaining time to let us hit our 100 Hz publish rate
        rate.sleep();       

    }
	return 0;
}
