/* ---------------------------------------------------------------
 * 
 * Node turtle_multi_node
 * Edoardo Corsi
 * March 2020
 * 
 * ----------------------------------------------------------------
 */

// Includes
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "exercise4_pkg/RobotStatus.h"

using namespace std;

// Structs
struct STATE
{
  float x;
  float y;
  float theta;
  bool is_ready;
  bool is_arrived;
};

// Global variables
// Variables of the team
ros::Subscriber team_status_sub;
ros::Publisher team_status_pub;
ros::Publisher vel_pub;
bool team_ready = false;
bool team_arrived = false;
map<string, STATE> robots_state;

int robot_to_move;
const int robot_number = 4;
string robot_name = "turtle1";
float orientation;

// Control variables
float threshold_ang = 0.1;
float threshold_dist = 0.2;
float k_yaw_rate = 0.5;
float k_long_vel = 0.5;

// Function that publishes the information that the node is ready
void publish_ready_status(string robot_name)
{
	exercise4_pkg::RobotStatus status_msg;

	status_msg.header.stamp = ros::Time::now();
	status_msg.robot_id = robot_name;
	status_msg.is_ready = true;
    status_msg.is_arrived = false;
    status_msg.x_initial = robots_state[robot_name].x;
    status_msg.y_initial = robots_state[robot_name].y;

	team_status_pub.publish(status_msg);
}

// Function that waits that all robots are ready
void wait_for_team()
{
	ros::Rate loopRate(1);

	// Wait until all robots are ready ...
	while (!team_ready)
    {
		publish_ready_status(robot_name);
		ros::spinOnce();
		loopRate.sleep();
	}
	
}

// Function that computes the barycenter of the initial conditions
void compute_barycenter(float* barycenter)
{
    float bar_x = 0, bar_y = 0;
        
    for(auto & robot : robots_state)
    {
        bar_x += robot.second.x;
        bar_y += robot.second.y;
    }

    barycenter[0] = bar_x/robot_number;
    barycenter[1] = bar_y/robot_number;

    ROS_INFO("barycenter computed = %.2f,%.2f", barycenter[0], barycenter[1]);
}

// Function that returns the orientation reference from target point
float get_orientation_ref(float x_target, float y_target, string name)
{
    float err_x;
    float err_y;
    float theta_ref;
    
    err_x = (x_target - robots_state[name].x);
    err_y = (y_target - robots_state[name].y);
    theta_ref = atan2f(err_y,err_x);
    
    return theta_ref;

}

// Function that returns the distance error from target point
float get_distance_err(float x_target, float y_target, string name)
{
    float err_x;
    float err_y;
    float err_dist;

    err_x = (x_target - robots_state[name].x);
    err_y = (y_target - robots_state[name].y);
    err_dist = sqrt(pow(err_x,2) + pow(err_y,2));

    return err_dist;
}

// Function that checks if all robots are ready to go and if all robots are arrived at the target
void team_status_callback(const exercise4_pkg::RobotStatus::ConstPtr& status_msg)
{
	robots_state[status_msg->robot_id].is_ready = status_msg->is_ready;
   	robots_state[status_msg->robot_id].is_arrived = status_msg->is_arrived;
   	robots_state[status_msg->robot_id].x = status_msg->x_initial;
   	robots_state[status_msg->robot_id].y = status_msg->y_initial;

    // Check if all robots have sent their positions
    if(!team_ready)
    {
        int ready_counter = 0;
        
        for(auto & robot : robots_state)
        {
            if(robot.second.is_ready)
                ready_counter ++;
        }

        if(ready_counter == robot_number)
        {
            team_ready = true;
        }
    }
    
    // Check if all robots are arrived to average initial conditions
    if(!team_arrived)
    {
        int robot_to_move_counter = 0;
        
        for(auto & robot : robots_state)
        {
            if(robot.second.is_arrived)
                robot_to_move_counter ++;
        }
        
        if(robot_to_move_counter == robot_number)
        {
            team_arrived = true;
            ROS_INFO("All robots are arrived!");
        }
        
    robot_to_move = (robot_to_move_counter + 1);
    }
}

// Topic message callback
void pose_callback(const turtlesim::Pose::ConstPtr& pose_msg)
{
    robots_state[robot_name].x = pose_msg->x;
    robots_state[robot_name].y = pose_msg->y;
    robots_state[robot_name].theta = pose_msg->theta;

}

// Function that moves the robot towards the target
void move_to_target(float ref_angle, float x_target, float y_target, string name)
{
    float err_angle, err_dist;
    exercise4_pkg::RobotStatus status_msg;
    
    geometry_msgs::Twist cmd_msg;
        
    ros::Rate loopRate(100);

    do
    {
        err_angle = ref_angle - robots_state[robot_name].theta;
        
        cmd_msg.linear.x = 0;
        cmd_msg.angular.z = k_yaw_rate * err_angle;
                    
        vel_pub.publish(cmd_msg);          
        
        ros::spinOnce();
        loopRate.sleep();
        
    } while(fabs(err_angle) > threshold_ang);

    ROS_INFO("%s : angle error is %.3f", name.c_str(), err_angle);
    sleep(2);

    do
    {
        err_dist = get_distance_err(x_target, y_target, name);
        
        cmd_msg.linear.x = k_long_vel * err_dist;
        cmd_msg.angular.z = 0;

        vel_pub.publish(cmd_msg);          
        
        ros::spinOnce();
        loopRate.sleep();
        
    } while(err_dist > threshold_dist);

    ROS_INFO("%s : distance error is %.3f", name.c_str(), err_dist);
    sleep(2);
    
    // Publish the message to the team that the robot has reached the target point
    status_msg.header.stamp = ros::Time::now();
    status_msg.robot_id = robot_name;
    status_msg.is_ready = true;
    status_msg.is_arrived = true;
    status_msg.x_initial = robots_state[robot_name].x;
    status_msg.y_initial = robots_state[robot_name].y;

    team_status_pub.publish(status_msg);
    
    ROS_INFO("%s is arrived!", name.c_str());
    
}

// Function that manages team's movement
void manage_team(float x_target, float y_target)
{
    ros::Rate loopRate(1);
   
    string str_robot_to_move;
        
    while(ros::ok()) 	// Keep spinning loop until user presses Ctrl+C
	{
        ros::spinOnce();
        str_robot_to_move = "turtle" + to_string(robot_to_move);
        
        if(robot_name.compare(str_robot_to_move) == 0)
        {
            move_to_target(orientation, x_target, y_target, str_robot_to_move);
        }
        
        loopRate.sleep();
	}
}

// Main function
int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "turtle_multi"); 
	ros::NodeHandle node("~");
   
    // Get robot name as parameter from launch file
	node.getParam("robot_name", robot_name);
    
    float barycenter[2];
	
	// Publish and subscribe to team status messages
	team_status_pub = node.advertise<exercise4_pkg::RobotStatus>("/team_status", 10);
	team_status_sub = node.subscribe("/team_status", 20, &team_status_callback);

    // Create Publisher and Subscriber
	vel_pub = node.advertise< geometry_msgs::Twist>("/" + robot_name + "/cmd_vel",1000);   
    ros::Subscriber sub_pose = node.subscribe("/" + robot_name + "/pose",100, &pose_callback);
      
    sleep(2); // wait for adv-sub
	ros::spinOnce(); // Call the Callbacks to get current position
	sleep(2);
    
	publish_ready_status(robot_name);

	wait_for_team();
    
    // Compute barycenter
    compute_barycenter(barycenter);
    
    // Compute angle reference signal from the target point
    orientation = get_orientation_ref(barycenter[0], barycenter[1], robot_name);
    
    // Move the robot
    manage_team(barycenter[0], barycenter[1]);
    
	return 0;
}
