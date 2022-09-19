#include <ugv_planner/traj_visualization.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Eigen>

namespace ugv_planner {


    TrajVisualization::TrajVisualization(){}
    TrajVisualization::~TrajVisualization(){}


    void TrajVisualization::init(ros::NodeHandle &nh)
    {
        this->nh = nh;
        trajectory_vis_pub  = nh.advertise<visualization_msgs::Marker>("trajectory/vis", 2);
        target_vis_pub      = nh.advertise<visualization_msgs::Marker>("target/vis", 20);
        point_vis_pub       = nh.advertise<visualization_msgs::Marker>("point2/vis", 20);
        pvpair_vis_pub      = nh.advertise<visualization_msgs::Marker>("pvpair/vis", 20);
        polygons_vis_pub    = nh.advertise<visualization_msgs::Marker>("polyhedron_corridor/vis",2);
        //polygons_vis_pub    = nh.advertise<geometry_msgs::PolygonStamped>("polyhedron_corridor/vis",20);
    }


    void TrajVisualization::visTarget(geometry_msgs::PoseStamped& target_info)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id  = "world";
        sphere.header.stamp     = ros::Time::now();
        sphere.type             = visualization_msgs::Marker::SPHERE;
        sphere.action           = visualization_msgs::Marker::ADD;
        sphere.id               = 1;

        sphere.pose.orientation.w   = 1.0;
        sphere.color.r              = 1.0;
        sphere.color.g              = 0.2;
        sphere.color.b              = 0.2;
        sphere.color.a              = 0.7;
        sphere.scale.x              = 0.5;
        sphere.scale.y              = 0.5;
        sphere.scale.z              = 0.5;
        sphere.pose.position.x      = target_info.pose.position.x;
        sphere.pose.position.y      = target_info.pose.position.y;
        sphere.pose.position.z      = target_info.pose.position.z;

        target_vis_pub.publish(sphere);
    }

    void TrajVisualization::visPolynomialTrajectory(Trajectory traj, Eigen::Vector3d color, int id = 10)
    {
        visualization_msgs::Marker traj_vis;
        traj_vis.header.stamp       = ros::Time::now();
        traj_vis.header.frame_id    = "world";
        traj_vis.id = id;
        traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
        traj_vis.scale.x = 0.05;
        traj_vis.scale.y = 0.05;
        traj_vis.scale.z = 0.05;
        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;

        traj_vis.color.a = 1.0;
        traj_vis.color.r = color(0);
        traj_vis.color.g = color(1);
        traj_vis.color.b = color(2);
        geometry_msgs::Point pt;
        Eigen::Vector3d pos;

        double t_duration = traj.getTotalDuration();
        for(double t = 0; t < t_duration; t += 0.05)
        {
            pos = traj.getPos(t);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            // pt.z = 0.05;
            traj_vis.points.push_back(pt);
        }
        trajectory_vis_pub.publish(traj_vis);
    }

    void TrajVisualization::visBezierTrajectory(Bernstein* bezier_basis , Eigen::MatrixXd polyCoeff, Eigen::VectorXd time)
    {   
        visualization_msgs::Marker traj_vis, control_pts;

        int cpts_id = 0;
        int traj_id = 1000;
        control_pts.header.stamp    = traj_vis.header.stamp       = ros::Time::now();
        control_pts.header.frame_id = traj_vis.header.frame_id    = "world";

        control_pts.id = cpts_id;
        traj_vis.id = traj_id;
        traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
        control_pts.type = visualization_msgs::Marker::SPHERE_LIST;

        control_pts.action = traj_vis.action = visualization_msgs::Marker::ADD;
        traj_vis.scale.x = 0.05;
        traj_vis.scale.y = 0.05;
        traj_vis.scale.z = 0.05;
        control_pts.scale.x = 0.1;
        control_pts.scale.y = 0.1;
        control_pts.scale.z = 0.1;

        control_pts.pose.orientation.x = 0.0;
        control_pts.pose.orientation.y = 0.0;
        control_pts.pose.orientation.z = 0.0;
        control_pts.pose.orientation.w = 1.0;

        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;

        traj_vis.color.a = 1.0;
        traj_vis.color.r = 1.0;
        traj_vis.color.g = 1.0;//max(0.0, 1 - rgb / 5.0);
        traj_vis.color.b = 1.0;
        control_pts.color.a = 1.0;
        control_pts.color.r = 1.0;
        control_pts.color.g = 0.0;
        control_pts.color.b = 0.0;


        double traj_len = 0.0;
        int count = 0;
        Eigen::Vector3d cur, pre;
        cur.setZero();
        pre.setZero();
        
        traj_vis.points.clear();

        Eigen::Vector3d state;
        geometry_msgs::Point pt, ctl_pt;

        int segment_num  = polyCoeff.rows();
        for(int i = 0; i < segment_num; i++ ){

            for(int j = 0; j < 8; j++)
            {
                ctl_pt.x = polyCoeff(i, j)    * time(i) ;
                ctl_pt.y = polyCoeff(i, j+8)  * time(i) ;
                ctl_pt.z = polyCoeff(i, j+16) * time(i) ;
                control_pts.points.push_back(ctl_pt);
            }
            for (double t = 0.0; t < 1.0; t += 0.01 / time(i), count += 1){
                state = bezier_basis -> getPosFromBezier( polyCoeff, t, i );
                cur(0) = pt.x = time(i) * state(0);
                cur(1) = pt.y = time(i) * state(1);
                cur(2) = pt.z = time(i) * state(2);
                traj_vis.points.push_back(pt);

                if (count) traj_len += (pre - cur).norm();
                pre = cur;
            }
        }
            trajectory_vis_pub.publish(traj_vis);
            trajectory_vis_pub.publish(control_pts);

    }


    void TrajVisualization::renderPoints(vector<Eigen::Vector3d> pts, Eigen::Vector3d color , int type, double scale, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id  = "world";
        sphere.header.stamp     = ros::Time::now();
        if(type == 0)
            sphere.type             = visualization_msgs::Marker::LINE_STRIP;
        else if(type == 1)
            sphere.type             = visualization_msgs::Marker::SPHERE_LIST;
        sphere.action           = visualization_msgs::Marker::ADD;
        sphere.id               = id;

        sphere.pose.orientation.w   = 1.0;
        sphere.color.r              = color(0);
        sphere.color.g              = color(1);
        sphere.color.b              = color(2);
        sphere.color.a              = 0.8;
        sphere.scale.x              = scale;
        sphere.scale.y              = scale;
        sphere.scale.z              = scale;
        geometry_msgs::Point pt;
        Eigen::Vector3d ptv;
        for(int i = 0 ; i < pts.size(); i++)
        {
            ptv = pts[i];
            pt.x = ptv(0);
            pt.y = ptv(1);
            pt.z = ptv(2);
            sphere.points.push_back(pt);
        }
        //sphere.pose.position.x      = pt(0);
        //sphere.pose.position.y      = pt(1);
        //sphere.pose.position.z      = pt(2);

        point_vis_pub.publish(sphere);
    }

    void TrajVisualization::renderAPoint(Eigen::Vector3d pt, Eigen::Vector3d color)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id  = "world";
        sphere.header.stamp     = ros::Time::now();
        sphere.type             = visualization_msgs::Marker::SPHERE;
        sphere.action           = visualization_msgs::Marker::ADD;
        sphere.id               = 46;

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

        point_vis_pub.publish(sphere);
    }

/*
    void TrajVisualization::renderSMC(vector<Eigen::Vector3d> vertices)
    {
        static int id = 0;
        geometry_msgs::PolygonStamped polyhedron_stm;
        geometry_msgs::Polygon polyhedron;
        geometry_msgs::Point32 pt;
        

        for(int i = 0 ; i < vertices.size(); i++)
        {
            pt.x = vertices[i](0);
            pt.y = vertices[i](1);
            pt.z = vertices[i](2);
            polyhedron.points.push_back(pt);
        }

        polyhedron_stm.header.frame_id = "world";
        polyhedron_stm.polygon         = polyhedron;
        polyhedron_stm.id              = id;
        id ++;
        polygons_vis_pub.publish(polyhedron_stm);
    }
*/


    void TrajVisualization::renderSMC(vector<PolygonCorridor> corridors)
    {
        visualization_msgs::Marker smc;
        smc.header.frame_id  = "world";
        smc.header.stamp     = ros::Time::now();
        smc.type             = visualization_msgs::Marker::SPHERE_LIST;
        smc.action           = visualization_msgs::Marker::ADD;
        smc.id               = 4;

        smc.pose.orientation.w   = 1.0;
        smc.color.r              = 0.9;
        smc.color.g              = 0.9;
        smc.color.b              = 0.9;
        smc.color.a              = 1.0;
        smc.scale.x              = 0.02;
        smc.scale.y              = 0.02;
        smc.scale.z              = 0.02;
        geometry_msgs::Point pt;
        Eigen::Vector3d ptv;
        vector<Eigen::Vector3d> vtcs;
        
        for(int i = 0 ; i < corridors.size(); i++)
        {
            vtcs = corridors[i].getVertices();
            if(vtcs.size() > 2)
            {   
                for(int j = 0 ; j < vtcs.size()-1 ;j++)
                {
                    
                    for(double t = 0 ; t <= 1.0; t += 0.005)
                    {
                        ptv = t * vtcs[j] + (1.0-t) * vtcs[j+1];
                        pt.x = ptv(0);
                        pt.y = ptv(1);
                        pt.z = 0.0;
                        smc.points.push_back(pt);
                    }
                }
                    for(double t = 0 ; t <= 1.0; t += 0.005)
                    {
                        ptv = t * vtcs[0] + (1.0-t) * vtcs[vtcs.size()-1];
                        pt.x = ptv(0);
                        pt.y = ptv(1);
                        pt.z = 0.0;
                        smc.points.push_back(pt);
                    }
            }
        }

        polygons_vis_pub.publish(smc);
    }


}


