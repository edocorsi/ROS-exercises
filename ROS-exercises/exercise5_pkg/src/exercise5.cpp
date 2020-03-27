/* ---------------------------------------------------------------
 * 
 * Node turtle_avoidance
 * Edoardo Corsi
 * March 2020
 * 
 * ----------------------------------------------------------------
 */

// Includes
#include <math.h>  
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

using namespace std;

// Structs
struct force_components
{
    float x_components;
    float y_components;
};

// Global variables
// Publisher
ros::Publisher vel_pub;

// Robot variables
float x_pos;
float y_pos;
float theta_pos;

// Target variables
float distance_target_x;
float distance_target_y;
float distance_target_x_norm;
float distance_target_y_norm;
float distance_target;

float radius_of_influence = 1;

// Variables of first convex component of Cobs
float distance_obs_first_x;
float distance_obs_first_y;
float distance_obs_first;
float x_obs_first;
float y_obs_first;

// Variables of second convex component of Cobs
float distance_obs_second_x;
float distance_obs_second_y;
float distance_obs_second;
float x_obs_second;
float y_obs_second;

// Forces variables
force_components repulsive_force_total;
force_components attractive_force;
force_components total_force;
int gamma_val = 2;
float kr = 3;
float ka = 1;

// Control variables
float kp_lin = 0.5;
float kp_yaw_rate = 2;
float threshold_ang = 0.02, threshold_dist = 0.2;

// Topic message callback
void pose_callback(const turtlesim::Pose::ConstPtr& pose_msg, const string &topic)
{
    if(topic.compare("/turtle1/pose") == 0)
    {        
        x_pos = pose_msg->x;
        y_pos = pose_msg->y;
        theta_pos = pose_msg->theta;
    }
    else if(topic.compare("/turtle2/pose") == 0)
    {
        x_obs_first = pose_msg->x;
        y_obs_first = pose_msg->y;
    }
    else
    {
        x_obs_second = pose_msg->x;
        y_obs_second = pose_msg->y;
    }
    
//     ROS_INFO(" x_obs_first is %.3f", x_obs_first);
//     ROS_INFO(" y_obs_first is %.3f", y_obs_first);
//     
//     ROS_INFO(" x_obs_second is %.3f", x_obs_second);
//     ROS_INFO(" y_obs_second is %.3f", y_obs_second);
//     
//     ROS_INFO(" pos x is %.3f", x_pos);
//     ROS_INFO(" pos y is %.3f", y_pos);
//     ROS_INFO(" pos theta is %.3f", theta_pos);
    
}

// Function that computes distances
void compute_distances(float x_target, float y_target)
{
    // Compute distance from target configuration on x and y components
    distance_target_x = (x_target - x_pos);
    distance_target_y = (y_target - y_pos);
    
    // Compute distance from target configuration 
    distance_target = sqrt(pow(distance_target_x,2) + pow(distance_target_y,2));
    
    // Compute distance from target configuration on x nd y components normalized
    distance_target_x_norm = distance_target_x / distance_target;
    distance_target_y_norm = distance_target_y / distance_target;
   
    // Compute distances from Cobs first convex component on x and y components
    distance_obs_first_x = (x_pos - x_obs_first);
    distance_obs_first_y = (y_pos - y_obs_first);

    // Compute distances from Cobs second convex component on x and y components
    distance_obs_second_x = (x_pos - x_obs_second);
    distance_obs_second_y = (y_pos - y_obs_second);
    
    // Compute distance from obstacles
    distance_obs_first = sqrt(pow(distance_obs_first_x,2) + pow(distance_obs_first_y,2));
    distance_obs_second = sqrt(pow(distance_obs_second_x,2) + pow(distance_obs_second_y,2));
    
//     ROS_INFO(" distance target x is %.3f", distance_target_x);
//     ROS_INFO(" distance_target y is %.3f", distance_target_y);
//     ROS_INFO(" distance_target is %.3f", distance_target);
//      
//     ROS_INFO(" distance_target_x_norm is %.3f", distance_target_x_norm);
//     ROS_INFO(" distance_target_y_norm is %.3f", distance_target_y_norm);
//     
//     ROS_INFO(" distance_obs_first_x is %.3f", distance_obs_first_x);
//     ROS_INFO(" distance_obs_first_y is %.3f", distance_obs_first_y);
//     
//     ROS_INFO(" distance_obs_second_x is %.3f", distance_obs_second_x);
//     ROS_INFO(" distance_obs_second_y is %.3f", distance_obs_second_y);
// 
//     ROS_INFO(" distance_obs_first is %.3f", distance_obs_first);
//     ROS_INFO(" distance_obs_second is %.3f", distance_obs_second);
}

// Function that computes repulsive force
void compute_repulsive_forces()
{
    float repulsive_force_first = 0;
    float repulsive_force_second = 0;
    
    // Compute repulsive force from the first convex component of Cobs
    if (distance_obs_first >=  radius_of_influence)
    {
        repulsive_force_first = 0;
    }
    else
    {
        repulsive_force_first = (kr / pow(distance_obs_first,2)) * pow((1/distance_obs_first - 1/radius_of_influence),gamma_val-1);
    }
        
    repulsive_force_total.x_components += repulsive_force_first * distance_obs_first_x;
    repulsive_force_total.y_components += repulsive_force_first * distance_obs_first_y;
    
    // Compute repulsive force from the second convex component of Cobs
    if (distance_obs_second >=  radius_of_influence)
    {
        repulsive_force_second = 0;
    }
    else
    {
        repulsive_force_second = (kr / pow(distance_obs_second,2)) * pow((1/distance_obs_second - 1/radius_of_influence),gamma_val-1);
    }
    
    repulsive_force_total.x_components += repulsive_force_second * distance_obs_second_x;
    repulsive_force_total.y_components += repulsive_force_second * distance_obs_second_y;
    
//     ROS_INFO(" repulsive_force_total.x_components is %.3f", repulsive_force_total.x_components);
//     ROS_INFO(" repulsive_force_total.y_components is %.3f", repulsive_force_total.y_components);
}

// Function that computes attractive force
void compute_attractive_forces()
{
    if (distance_target > 1)
    {
        attractive_force.x_components = ka * distance_target_x;
        attractive_force.y_components = ka * distance_target_y;
    }
    else
    {
        attractive_force.x_components = ka * distance_target_x_norm;
        attractive_force.y_components = ka * distance_target_y_norm;        
    }
//     ROS_INFO(" attractive_force.x_components is %.3f", attractive_force.x_components);
//     ROS_INFO("attractive_force.y_components is %.3f", attractive_force.y_components);
}

// Function that allows to align the robot to the target
void alignToTarget()
{
    geometry_msgs::Twist cmd;

    ros::Rate loopRate(200);
    float err_angle;

    do
    {
        err_angle = atan2(total_force.y_components, total_force.x_components) - theta_pos;
                
        cmd.angular.z = kp_yaw_rate * err_angle;
                    
        vel_pub.publish(cmd);          
        
        ros::spinOnce();
        loopRate.sleep();
        
    } while(fabs(err_angle) > threshold_ang);

    // stop turtle
    cmd.linear.x = 0;
    cmd.angular.z = 0;
    vel_pub.publish(cmd);

    //ROS_INFO("aligned!");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_avoidance");
	ros::NodeHandle nh("~");

    // The node run at 50 Hz
    ros::Rate rate(50); 
    
    // Definition of local variables
    float total_force_value;
    float x_target = 0, y_target = 0;
    geometry_msgs::Twist cmd_msg;

    // Get parameters from launch file
    nh.getParam("x_target", x_target);
    nh.getParam("y_target", y_target);
    nh.getParam("radius_of_influence", radius_of_influence);
    
    // Create Subscribers        
    ros::Subscriber sub_pose = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 1, boost::bind(pose_callback, _1, "/turtle1/pose"));  
	ros::Subscriber sub_c_obs_first = nh.subscribe<turtlesim::Pose>("/turtle2/pose", 1, boost::bind(pose_callback, _1, "/turtle2/pose"));  
   	ros::Subscriber sub_C_obs_second = nh.subscribe<turtlesim::Pose>("/turtle3/pose", 1, boost::bind(pose_callback,_1,"/turtle3/pose"));
    
    vel_pub = nh.advertise< geometry_msgs::Twist>("/turtle1/cmd_vel",1000);   
    
    sleep(2); // wait for adv-sub
	ros::spinOnce(); // Call the Callbacks to get current position of the robot and of the obstacles
	sleep(2);
    
    compute_distances(x_target, y_target);
    
    ROS_INFO("obs1: x,y=%.2f,%.2f", x_obs_first, y_obs_first);
    ROS_INFO("obs2: x,y=%.2f,%.2f", x_obs_second, y_obs_second);
    ROS_INFO("The radius of influence of the obstacles is %f", radius_of_influence);
    ROS_INFO("target: x,y=%.2f,%.2f", x_target, y_target);

    ROS_INFO("Move robot towards target");

    while(distance_target > threshold_dist)
    {
        // Sum of repulsive forces
        repulsive_force_total.x_components = 0;
        repulsive_force_total.y_components = 0;
    
        compute_repulsive_forces();
        
        // Attractive force
        attractive_force.x_components = 0;
        attractive_force.y_components = 0;
        
        compute_attractive_forces();
        
        // Compute total force as sum of repulsive and attractive forces
        total_force.x_components = attractive_force.x_components + repulsive_force_total.x_components;
        total_force.y_components = attractive_force.y_components + repulsive_force_total.y_components;
        
        total_force_value = sqrt(pow(total_force.x_components,2) + pow(total_force.y_components,2)); 
        
        // Align to theta reference
        alignToTarget();

        // Move the robot forward
        cmd_msg.linear.x = kp_lin*total_force_value;
        vel_pub.publish(cmd_msg);
        
        ros::spinOnce();
        rate.sleep();
        
        compute_distances(x_target, y_target);

    }
    
    ROS_INFO("turtle1 reached target point!");
    
	return 0;
}
