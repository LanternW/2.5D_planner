#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <bspline_opt/uniform_bspline.h>
#include "grid_map/global_map_manager.h"
#include <ros/ros.h>
#include <ugv_planner/traj_visualization.h>
#include <ugv_planner/quickhull.hpp>
#include <ugv_planner/bezier_base.h>
#include <ugv_planner/lan_bezier_optimizer.h>
#include <minco_opt/traj_opt.h>
#include "ugv_planner/Polynome.h"
#include <ugv_planner/corridor.h>

namespace ugv_planner
{

 

  class UGVPlannerManager
  {
    // SECTION stable
  public:
    UGVPlannerManager();
    ~UGVPlannerManager();
    void init(ros::NodeHandle& nh);

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    void targetRcvCallback(const geometry_msgs::PoseStamped target_info);
    void odomRcvCallback(const nav_msgs::Odometry odom);
    void despRcvCallback(const geometry_msgs::PoseStamped desp);

    void globalReplan();
    void PublishTraj();

    ////// SMC
    vector<Eigen::Vector3d> sortVertices(vector<Eigen::Vector3d> vertices);
    vector<PolygonCorridor> corridors;
    vector<double> time_allocation;
    void generateOCC();
    void generateSMC(vector<Eigen::Vector3d> path);
    PolygonCorridor cluster2Corridor(vector<Eigen::Vector3d> grids_set ,Eigen::Vector3d seed_pt );
    Eigen::Vector3d findClosestEdgeNormal(PolygonCorridor corridor, Eigen::Vector3d pt);
    void generateCurveByWps(vector<Eigen::Vector3d> waypoints );
    void generateCurveByOptimizer(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, int seg);
    void generateFinalMincoTraj(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, vector<Eigen::Vector3d> path, bool use_path_wp = false);
    void trajPlanning(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, vector<Eigen::Vector3d> path);
    void checkJps();
    void pubJps();   
    vector<double> timeAllocate(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);



  private:

    ros::Subscriber target_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber des_pos_sub;
    ros::Publisher  traj_pub;
    ros::Publisher  jps_pub;
    ros::Publisher  target_pub;
    TrajVisualization vis_render;
    Bernstein* bezier_basis;
    Eigen::MatrixXd bezier_coeff;
    double time_duration;

    ros::NodeHandle nh;



    bool has_odom;
    bool has_target;

    geometry_msgs::PoseStamped    target;        //the global target
    Eigen::Vector3d               target_pos;    // = target
    Eigen::Vector3d               now_pos;       // = rt_odometry
    Eigen::Vector3d               now_vel;       // = rt_odometry'
    Eigen::Vector3d               now_acc;       // = rt_odometry''
    Eigen::Vector3d               des_pos;       // updated from traj server
    nav_msgs::Odometry            rt_odometry;   //the odometry on real time

    double p_max_vel;
    double p_max_acc;
    ros::Time traj_start_time;
    ros::Time traj_end_time;

    lanBezierOptimizer* lan_bezier_optimizer;
    unique_ptr<TrajOpt> minco_traj_optimizer;
    Trajectory final_trajectory;
  public:

    LanGridMapManager::Ptr global_map_manager;

  };
} 

#endif