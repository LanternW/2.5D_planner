#ifndef _TRAJ_VISUALIZATION_H_
#define _TRAJ_VISUALIZATION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "traj_utils/Bspline.h"
#include "grid_map/global_map_manager.h"
#include "bspline_opt/uniform_bspline.h"
#include <Eigen/Eigen>

#include <ugv_planner/bezier_base.h>
#include <ugv_planner/corridor.h>
#include <minco_opt/poly_traj_utils.hpp>
#include <vector>

namespace ugv_planner {

    class TrajVisualization {

    public:
        TrajVisualization();
        ~TrajVisualization();
        void init(ros::NodeHandle &nh);
        void visTarget(geometry_msgs::PoseStamped& target_info);


        void renderPoints(std::vector<Eigen::Vector3d> pts, Eigen::Vector3d color, int type, double scale, int id);
        void renderAPoint(Eigen::Vector3d pt, Eigen::Vector3d color);

        //void renderSMC(vector<Eigen::Vector3d> vertices);
        void renderSMC(vector<PolygonCorridor> corridors);
        void visBezierTrajectory(Bernstein* bezier_basis, Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);
        void visPolynomialTrajectory(Trajectory traj, Eigen::Vector3d color, int id);




    private:
        ros::NodeHandle nh;
        ros::Publisher  point_vis_pub;
        ros::Publisher  trajectory_vis_pub;
        ros::Publisher  target_vis_pub;
        ros::Publisher  pvpair_vis_pub;
        ros::Publisher  polygons_vis_pub;

    };
}


#endif
