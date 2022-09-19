#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ugv_planner/DVHcontrol.h>


using namespace std;

ros::Subscriber velocity_cmdsub;
ros::Subscriber jump_cmdsub;
ros::Publisher  odom_pub;
//tf::TransformBroadcaster broadcaster;

ugv_planner::DVHcontrol velocity_command;
nav_msgs::Odometry last_odom;

double p_init_x, p_init_y, p_init_z;
double max_height, min_height;
double time_resolution = 0.01;

bool rcv_cmd = false;

bool in_air  = false;
double up_speed = 0;

void initParams()
{
	p_init_x = 1.5;
	p_init_y = -0.5;
	//p_init_z = 0.8;
	//max_height = 0.8;
	min_height = 0.2;
}

void rcvVelCmdCallBack(const ugv_planner::DVHcontrol cmd)
{	
	         rcv_cmd 	= true;
	velocity_command    = cmd;
}

void normyaw(double& y)
{
  if (y > M_PI)
  {
    y-=2*M_PI;
  }
  else if (y < -M_PI)
  {
    y+=2*M_PI;
  }
}

void rcvJmpCmdCallBack(const std_msgs::Empty jcmd)
{
	rcv_cmd = true;
	if(in_air == false)
	{
		in_air  = true;
		up_speed = 4.0;
	}
}

void pubOdom()
{	
	nav_msgs::Odometry new_odom;

	new_odom.header.stamp       = ros::Time::now();
	new_odom.header.frame_id    = "world";

	if(rcv_cmd)
	{
		
		double vel_left   = velocity_command.left_wheel_vel;
		double vel_right  = velocity_command.right_wheel_vel;
		double vel_vertical = velocity_command.vetical_vel;

		double roll_speed = 0.5 * ( velocity_command.left_wheel_vel + velocity_command.right_wheel_vel );
	    new_odom.twist.twist.angular.y = roll_speed;

		Eigen::Quaterniond    q(	last_odom.pose.pose.orientation.w,
							    	last_odom.pose.pose.orientation.x,
							    	last_odom.pose.pose.orientation.y,
							    	last_odom.pose.pose.orientation.z  );

		Eigen::Matrix3d       R(q);
		Eigen::Vector2d       lvel(last_odom.twist.twist.linear.x,last_odom.twist.twist.linear.y);
		double last_yaw 	= atan2(R.col(0)[1],R.col(0)[0]);                  
		double last_x   	= last_odom.pose.pose.position.x;                  
		double last_y  	    = last_odom.pose.pose.position.y;                  
		double last_z  	    = last_odom.pose.pose.position.z;
		double last_a       = last_odom.twist.twist.angular.x;

		


		new_odom.pose.pose.position.x  = last_x + 0.5 * (vel_left + vel_right) * cos(last_yaw) * time_resolution ;
		new_odom.pose.pose.position.y  = last_y + 0.5 * (vel_left + vel_right) * sin(last_yaw) * time_resolution ;
		new_odom.pose.pose.position.z  = last_z + vel_vertical * time_resolution;
		new_odom.pose.pose.position.z  = max(min_height , min(new_odom.pose.pose.position.z , max_height));
		new_odom.twist.twist.linear.x  = 0.5 * (vel_left + vel_right) * cos(last_yaw);
		new_odom.twist.twist.linear.y  = 0.5 * (vel_left + vel_right) * sin(last_yaw);
		new_odom.twist.twist.linear.z  = vel_vertical;
		                 double omega  = (vel_right - vel_left) / 0.5;
		new_odom.twist.twist.angular.z = omega;

		if(in_air)
		{
			up_speed -= 9.8 * 0.01;
			double new_a        = last_a + up_speed *  0.01; 
			if(new_a <= 0){ new_a = 0; in_air = false;}   
			new_odom.twist.twist.angular.x = new_a;  
		}

		double yaw = last_yaw + omega * time_resolution;
		normyaw(yaw);

		//cout << "omega = "<< omega <<endl;
		Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);
		Eigen::Vector3d yC(-sin(yaw), cos(yaw), 0);
		Eigen::Vector3d zC(0, 0, 1);
		Eigen::Matrix3d R2;
		R2.col(0) = xC;
		R2.col(1) = yC;
		R2.col(2) = zC;
		Eigen::Quaterniond q2(R2);
		new_odom.pose.pose.orientation.w = q2.w();
		new_odom.pose.pose.orientation.x = q2.x();
		new_odom.pose.pose.orientation.y = q2.y();
		new_odom.pose.pose.orientation.z = q2.z();
	}
	else
	{
		new_odom = last_odom;
	}

	last_odom = new_odom;
    odom_pub.publish(new_odom);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "ugv_kinematic_model_node");
    ros::NodeHandle nh( "~" );

	
    nh.param("max_height", max_height, 1.0);
    nh.param("max_height", p_init_z, 1.0);

	initParams();
	
		jump_cmdsub	 = nh.subscribe( "jump_command", 1, rcvJmpCmdCallBack );
    velocity_cmdsub  = nh.subscribe( "command", 1, rcvVelCmdCallBack );
    	   odom_pub  = nh.advertise<nav_msgs::Odometry>("odometry", 1);
				 
	last_odom.header.stamp    = ros::Time::now();
	last_odom.header.frame_id = "world";                      
	last_odom.pose.pose.position.x = p_init_x;
	last_odom.pose.pose.position.y = p_init_y;
	last_odom.pose.pose.position.z = p_init_z;

	last_odom.pose.pose.orientation.w = 1;
	last_odom.pose.pose.orientation.x = 0;
	last_odom.pose.pose.orientation.y = 0;
	last_odom.pose.pose.orientation.z = 0;

	last_odom.twist.twist.linear.x = 0.0;
	last_odom.twist.twist.linear.y = 0.0;
	last_odom.twist.twist.linear.z = 0.0;

	last_odom.twist.twist.angular.x = 0.0;
	last_odom.twist.twist.angular.y = 0.0;
	last_odom.twist.twist.angular.z = 0.0;

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
		pubOdom();                   
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}