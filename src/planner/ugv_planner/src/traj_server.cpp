
#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/Bspline.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "ugv_planner/DVHcontrol.h"
#include "ugv_planner/traj_visualization.h"
#include "ugv_planner/Polynome.h"
#include "minco_opt/minco.hpp"

#include <minco_opt/poly_traj_utils.hpp>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>

using namespace ugv_planner;
using namespace std;

ros::Publisher control_cmd_pub;
ros::Publisher jump_cmd_pub;
ros::Publisher despoint_vis_pub;
ros::Publisher despoint_pub;

const double pi = 3.1415926535;
double dt = 0.1;
double t_cur;

bool has_traj = false;
bool has_odom = false;

Trajectory trajectory;
minco::MinJerkOpt jerkOpt_;
double traj_duration;
ros::Time start_time;
int traj_id;

vector<double> jump_times;
int jump_index = 0;

//odometry on real time
Eigen::Vector3d  odometry_pos;
Eigen::Vector3d  odometry_vel;
double           odometry_yaw;


void tempRenderAPoint(Eigen::Vector3d pt, Eigen::Vector3d color)
{
    visualization_msgs::Marker sphere;
    sphere.header.frame_id  = "world";
    sphere.header.stamp     = ros::Time::now();
    sphere.type             = visualization_msgs::Marker::SPHERE;
    sphere.action           = visualization_msgs::Marker::ADD;
    sphere.id               = 1;
    sphere.pose.orientation.w   = 1.0;
    sphere.color.r              = color(0);
    sphere.color.g              = color(1);
    sphere.color.b              = color(2);
    sphere.color.a              = 0.8;
    sphere.scale.x              = 0.2;
    sphere.scale.y              = 0.2;
    sphere.scale.z              = 0.2;
    sphere.pose.position.x      = pt(0);
    sphere.pose.position.y      = pt(1);
    sphere.pose.position.z      = pt(2);
    despoint_vis_pub.publish(sphere);
    
}

void rcvJtCallBack(std_msgs::Float64MultiArray msg)
{
  jump_times.clear();
  double time;
  for(auto t : msg.data)
  {
    time = t;
    jump_times.push_back(time);
  }
}

void rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
  if(has_odom == false){ cout <<"[TRAJ_SERVER] has odometry "<<endl; }
  has_odom = true;
  odometry_pos[0] = msg->pose.pose.position.x;
  odometry_pos[1] = msg->pose.pose.position.y;
  odometry_pos[2] = msg->pose.pose.position.z;

  
  odometry_vel[0] = msg->twist.twist.linear.x;
  odometry_vel[1] = msg->twist.twist.linear.y;
  odometry_vel[2] = msg->twist.twist.linear.z;

  Eigen::Quaterniond q( msg->pose.pose.orientation.w,
			                  msg->pose.pose.orientation.x,
		                  	msg->pose.pose.orientation.y,
		                  	msg->pose.pose.orientation.z );
  Eigen::Matrix3d R(q);
  odometry_yaw = atan2(R.col(0)[1],R.col(0)[0]);
  
}


void polynomialTrajCallback(ugv_planner::PolynomeConstPtr msg)
{
  // parse pos traj
  Eigen::MatrixXd posP(3, msg -> pos_pts.size() - 2);
  Eigen::VectorXd T(msg -> t_pts.size());
  Eigen::MatrixXd initS, tailS;

  for (int i = 1; i < msg -> pos_pts.size() - 1 ;i++)
  {
    posP(0, i-1) = msg->pos_pts[i].x;
    posP(1, i-1) = msg->pos_pts[i].y;
    posP(2, i-1) = msg->pos_pts[i].z;
  }
  for (int i=0; i<msg->t_pts.size();i++)
  {
    T(i) = msg->t_pts[i];
  }
  
  initS.setZero(3, 3);
  tailS.setZero(3, 3);
  initS.col(0) = Eigen::Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
  initS.col(1) = Eigen::Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
  initS.col(2) = Eigen::Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
  tailS.col(0) = Eigen::Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
  tailS.col(1) = Eigen::Vector3d::Zero();
  tailS.col(2) = Eigen::Vector3d::Zero();
  jerkOpt_.reset(initS, msg->pos_pts.size()-1);
  jerkOpt_.generate(posP, tailS, T);

  trajectory    = jerkOpt_.getTraj();
  traj_duration = trajectory.getTotalDuration();

  start_time  = msg -> start_time;
  traj_id     = msg -> traj_id;
  jump_index  = 0;

  if(has_traj == false){ cout <<"[TRAJ_SERVER] has trajectory "<<endl; }
  has_traj = true;
}

double err_yaw( double des_yaw, double odom_yaw)
{
  if(des_yaw - odom_yaw >= pi)
    return (des_yaw - odom_yaw) - 2 * pi;
  else if(des_yaw - odom_yaw <= -pi)
    return 2 * pi + (des_yaw - odom_yaw);
  else
    return (des_yaw - odom_yaw); 
}

void cmdCallback(const ros::TimerEvent &e)
{
    // no publishing before receive traj
    if ((!has_traj) || (!has_odom))
      return;
    
    ros::Time time_now  = ros::Time::now();
    t_cur               = (time_now - start_time).toSec();

    ugv_planner::DVHcontrol cmd;
    cmd.header.frame_id = "world";
    if (t_cur < traj_duration && t_cur >= 0.0)
    {
      Eigen::Vector3d des_pos   = trajectory.getPos(t_cur);
      Eigen::Vector3d des_vel   = trajectory.getVel(t_cur);
      Eigen::Vector3d des_acc   = trajectory.getAcc(t_cur);
      Eigen::Vector2d des_velxy   = Eigen::Vector2d(des_vel(0), des_vel(1));
      tempRenderAPoint(des_pos, Eigen::Vector3d(0.1,0.2,0.9) );

      double des_yawdot  =  (des_acc(1) * des_vel(0) - des_acc(0) * des_vel(1)) / (des_velxy.squaredNorm());

      Eigen::Vector3d err_pos   = des_pos - odometry_pos;

      double des_yaw     = atan2(err_pos(1), err_pos(0));
      Eigen::Vector3d err_vel   = des_vel - odometry_vel;
      Eigen::Vector2d err_posxy = Eigen::Vector2d(err_pos(0), err_pos(1));
      Eigen::Vector2d err_velxy = Eigen::Vector2d(err_vel(0), err_vel(1));
      Eigen::Vector2d forward_dir = Eigen::Vector2d( cos(odometry_yaw) , sin(odometry_yaw) );

      double ef  = forward_dir.dot(err_posxy);
      double def = forward_dir.dot(err_velxy);

      double forward_speed = des_velxy.norm() + ( 5 * ef + 0.5 * def );
      double ori_speed     = des_yawdot + (4 * err_yaw(des_yaw , odometry_yaw) );
      double vertical_speed = des_vel(2) + (2 * (des_pos(2) - odometry_pos(2)));
      if(ori_speed > 5)
      {
        cout<<"dyd = " << des_yawdot <<" | dy = " << des_yaw << "  | odom_yaw = " << odometry_yaw <<endl;
      }

      double L = 0.5;
      cmd.left_wheel_vel  =  (forward_speed - L * ori_speed) / 2;
      cmd.right_wheel_vel =  (forward_speed + L * ori_speed) / 2;
      cmd.vetical_vel     =  vertical_speed;

      control_cmd_pub.publish(cmd);
    }
    else if(t_cur >= traj_duration)
    {
      cmd.left_wheel_vel  =  0;
      cmd.right_wheel_vel =  0;
      cmd.vetical_vel     =  0;

      jump_times.clear();
      control_cmd_pub.publish(cmd);
    }

    if(jump_index < jump_times.size() && abs( t_cur - jump_times[jump_index]) < 0.005)
    {
      std_msgs::Empty jcmd;
      jump_cmd_pub.publish(jcmd);
      jump_index ++;
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber traj_sub      = nh.subscribe("trajectory_topic", 10, polynomialTrajCallback);
  ros::Subscriber odom_sub      = nh.subscribe("odom", 1, rcvOdomCallBack );
  ros::Subscriber jt_sub        = nh.subscribe("jt_array", 1, rcvJtCallBack );

  despoint_pub      = nh.advertise<geometry_msgs::PoseStamped>("despoint", 20); 
  despoint_vis_pub  = nh.advertise<visualization_msgs::Marker>("point/vis", 20); 
  control_cmd_pub   = nh.advertise<ugv_planner::DVHcontrol>("controller_cmd", 20); 
  jump_cmd_pub      = nh.advertise<std_msgs::Empty>("jump_command", 20); 

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);


  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}