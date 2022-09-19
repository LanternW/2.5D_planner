#pragma once
#include <grid_map/global_map_manager.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <chrono>
#include <thread>

#include "minco.hpp"


namespace ugv_planner {

class TrajOpt {
 public:
  ros::NodeHandle nh_;
  ros::Publisher debug_pub, debug_wp_pub;

  LanGridMapManager::Ptr grid_map_manager;
  bool pause_debug_ = true;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_;
  // collision avoiding and dynamics paramters
  double pok_, vmax_, amax_, vmax_z_ , amax_z_;
  double rhoP_, rhoV_, rhoA_;
  double rhoTracking_, rhosVisibility_;
  double clearance_d_, tolerance_d_, theta_clearance_;
  // SE3 dynamic limitation parameters
  double thrust_max_, thrust_min_;
  double omega_max_, omega_yaw_max_;
  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // Minimum Jerk Optimizer
  minco::MinJerkOpt jerkOpt_;
  Eigen::MatrixXd initS_;
  Eigen::MatrixXd finalS_;
  // weight for each vertex
  Eigen::VectorXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;
  double sum_T_;

  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;
  double tracking_dur_;
  double tracking_dist_;
  double tracking_dt_;

  // polyH utils
  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;
  
  void drawDebug(Trajectory end_path);
  void drawDebugWp(std::vector<Eigen::Vector3d> end_path);
  void deleteX_(){delete[] x_;}

 public:
  TrajOpt() {}
  ~TrajOpt() {}

  void setParam(ros::NodeHandle& nh)
  {
    nh_ = nh;
    nh.param("optimization/K", K_, 8);
    nh.param("optimization/pok", pok_, 0.3);
    nh.param("optimization/vmax", vmax_, 3.0);
    nh.param("optimization/amax", amax_, 4.0);
    nh.param("optimization/vmaxz", vmax_z_, 0.2);
    nh.param("optimization/amaxz", amax_z_, 0.4);
    nh.param("optimization/rhoT", rhoT_, 1000.0);
    nh.param("optimization/rhoP", rhoP_, 1000.0);
    nh.param("optimization/rhoV", rhoV_, 1000.0);
    nh.param("optimization/rhoA", rhoA_, 1000.0);
    nh.param("optimization/pause_debug", pause_debug_, false);
    debug_pub = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_path", 10);
    debug_wp_pub = nh.advertise<visualization_msgs::Marker>("/traj_opt/debug_path_wp", 10);

  }
  void setEnvironment(const LanGridMapManager::Ptr& mapPtr)
  {
      grid_map_manager = mapPtr;
  }
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const std::vector<Eigen::Vector3d>& path,
                     int N,
                     Trajectory& traj,
                     bool is_continuing);

  void addTimeIntPenalty(double& cost);
  bool grad_cost_p(const Eigen::Vector3d& p,
                   Eigen::Vector3d& gradp,
                   double& costp);
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_a(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& costa);
  
  bool grad_cost_d(const Eigen::Vector3d& p,
                   const Eigen::Vector3d& v, 
                   Eigen::Vector3d& gradp,
                   Eigen::Vector3d& gradv,
                   double& costp,
                   double& costv);

};

}  // namespace rm_planner