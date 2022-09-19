// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" 
#include <cstdlib>
#include <ctime>

//#define DEBUG_TRAJ8

// #define CONSIDER_JUMPING

const double pi = 3.1415926535;
using namespace quickhull;

namespace ugv_planner
{
  UGVPlannerManager::UGVPlannerManager() {

    has_odom                = false;
    has_target              = false;
    p_max_vel               = 1.0;
    p_max_acc               = 2.0;

    now_pos                 = now_vel = now_acc = Eigen::Vector3d(0,0,0);
    bezier_basis            = new Bernstein(3); 
    bezier_basis -> setFixedOrder(7);

    lan_bezier_optimizer   = new lanBezierOptimizer();

    global_map_manager.reset(new LanGridMapManager);
    global_map_manager   -> init(nh);

    minco_traj_optimizer.reset(new TrajOpt());

    corridors.clear();
    ROS_INFO("trajectory planner is ready."); 
  }

  UGVPlannerManager::~UGVPlannerManager() {}

  void UGVPlannerManager::init(ros::NodeHandle& nh)
  {
    this->nh       = nh;
    target_sub     = nh.subscribe("/move_base_simple/goal", 1, &UGVPlannerManager::targetRcvCallback, this);
    odom_sub       = nh.subscribe("/viscar/odom",1 ,&UGVPlannerManager::odomRcvCallback, this);
    des_pos_sub    = nh.subscribe("/ugv_traj_server/despoint",1 ,&UGVPlannerManager::despRcvCallback, this);
    traj_pub       = nh.advertise<ugv_planner::Polynome>("trajectory",3);
    jps_pub        = nh.advertise<std_msgs::Float64MultiArray>("jt_array",3);
    target_pub     = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);


    minco_traj_optimizer -> setParam(nh);
    minco_traj_optimizer -> setEnvironment(global_map_manager);


    global_map_manager -> setParam(nh);


    vis_render.init(nh);
  }




  void UGVPlannerManager::despRcvCallback(const geometry_msgs::PoseStamped desp)
  {
    des_pos(0) = desp.pose.position.x;
    des_pos(1) = desp.pose.position.y;
    des_pos(2) = desp.pose.position.z;
  }

  void UGVPlannerManager::targetRcvCallback(const geometry_msgs::PoseStamped target_info)
  { 
    if(has_target == false){
        ROS_INFO("Get target");
        has_target = true;
    }
    target     = target_info;
    target_pos(0) = target.pose.position.x;
    target_pos(1) = target.pose.position.y;
    target_pos(2) = 0;
    vis_render.visTarget(target);

    cout<< "g = " << global_map_manager -> getGapByPosW3(target_pos) <<endl;
    //traj plan
    globalReplan();

  
    //Eigen::Vector3d posw, grad;
    //posw(0) = target_pos(0);
    //posw(1) = target_pos(1);
    //posw(2) = 0;

    //grad = global_map_manager -> calcESDF_Grid(posw);
    //cout<<grad<<endl;

//////

    //Eigen::Vector3d posw;
    //posw(0) = target_pos(0);
    //posw(1) = target_pos(1);
    //posw(2) = 0;
    ////cout<< global_map_manager -> getHeightByPosW3(posw)<<endl;
    //vector<Eigen::Vector3d> step_cluster;
    //vector<PolygonCorridor> step_corridors;
    //cout<<"-----------------"<<endl;
    //step_cluster = global_map_manager -> convexNClusterInflation(global_map_manager -> posW2Index( global_map_manager -> point324(posw)) );
    //PolygonCorridor step_corridor = cluster2Corridor(step_cluster, posw);
    //vector<Eigen::Vector3d> vts = step_corridor.getVertices();
    //cout<<endl<<endl<<"seed:\n"<<posw<<endl<<endl;
    //for(Eigen::Vector3d vt : vts)
    //{
    //  cout<<vt<<endl<<endl;
    //}
    //step_corridors.push_back(step_corridor);
    //vis_render.renderSMC(step_corridors);
 

  }

  void UGVPlannerManager::odomRcvCallback(const nav_msgs::Odometry odom)
  {
      static Eigen::Vector3d last_vel(0,0,0);
      if(has_odom == false){
        ROS_INFO("Get odometry");
        has_odom = true;
      }
      rt_odometry = odom;
      now_pos(0) = odom.pose.pose.position.x;
      now_pos(1) = odom.pose.pose.position.y;
      now_pos(2) = odom.pose.pose.position.z;
      
      now_vel(0) = odom.twist.twist.linear.x;
      now_vel(1) = odom.twist.twist.linear.y;
      now_vel(2) = odom.twist.twist.linear.z;

      now_acc    = (now_vel - last_vel) * 100;
      last_vel   = now_vel;
  }


  void UGVPlannerManager::PublishTraj()
  {
      ugv_planner::Polynome poly;
      Eigen::MatrixXd poses = final_trajectory.getPositions();
      Eigen::VectorXd ts    = final_trajectory.getDurations();

      for (int i = 0; i < poses.cols(); i++)
      {
        geometry_msgs::Point temp;
        temp.x = poses(0, i);
        temp.y = poses(1, i);
        temp.z = poses(2, i);
        poly.pos_pts.push_back(temp);
      }
      for (int i = 0; i < ts.size(); i++)
      {
        poly.t_pts.push_back(ts(i));
      }
      poly.init_v.x = 0;
      poly.init_v.y = 0;
      poly.init_v.z = 0;
      poly.init_a.x = 0;
      poly.init_a.y = 0;
      poly.init_a.z = 0;
      poly.start_time = ros::Time::now();
      poly.traj_id = 1;

      traj_pub.publish(poly);
  }

  vector<Eigen::Vector3d> UGVPlannerManager::sortVertices(vector<Eigen::Vector3d> vertices)
  {
      Eigen::Vector3d center;

      center = 0.5 * vertices[0] + 0.5 * vertices[1];

      for(int i = 0 ; i < vertices.size() ; i++)
      {
         vertices[i] -= center;
      }

      std::sort(vertices.begin(), vertices.end(), [](Eigen::Vector3d a, Eigen::Vector3d b) {

        double ang1 = atan2(a(1), a(0));
        double ang2 = atan2(b(1), b(0));
        if(ang1 < 0) ang1 += 2 * pi;
        if(ang2 < 0) ang2 += 2 * pi;
        return ang1 < ang2;   
    });

    std::sort(vertices.begin(), vertices.end(), [](Eigen::Vector3d a, Eigen::Vector3d b) {

        double ang1 = atan2(a(1), a(0));
        double ang2 = atan2(b(1), b(0));
        if(ang1 < 0) ang1 += 2 * pi;
        if(ang2 < 0) ang2 += 2 * pi;
        return ang1 < ang2;   
    });

//    std::sort(vertices.begin(), vertices.end(), [](Eigen::Vector3d a, Eigen::Vector3d b) {
//
//        double ang1 = atan2(a(1), a(0));
//        double ang2 = atan2(b(1), b(0));
//        if(ang1 < 0) ang1 += 2 * pi;
//        if(ang2 < 0) ang2 += 2 * pi;
//        return ang1 < ang2;   
//    });

      for(int i = 0 ; i < vertices.size() ; i++)
      {
         vertices[i] += center;
      }
      return vertices;
  }


  void UGVPlannerManager::generateOCC()
  {
    std::cout<<"[OCC] gene begin" <<std::endl;
    vector<Eigen::Vector3d> vertices;
    vector<Eigen::Vector3d> vertices_;
    QuickHull<double> quickHull;
    vector<Vector3<double>> grids_set;
    vector<Eigen::Vector3d> grids_set_;
    corridors.clear();

    Eigen::Vector3d np;
    int addr;
    for(int i = 0 ; i <  global_map_manager -> map_index_xmax ; i++)
    {
      for(int j = 0 ; j <  global_map_manager -> map_index_ymax ; j++)
      {
        addr = global_map_manager -> toAddress(i,j);
        if(global_map_manager -> is_occupiedI(Eigen::Vector2i(i,j), 0) == false ){continue;}
        np = global_map_manager -> point423(global_map_manager -> index2PosW(Eigen::Vector2i(i,j)));

        bool in_cor = false;
        if(corridors.size() != 0)
        {
          for (auto corridor : corridors)
          {
            if(corridor.isPointInPolygon(np))
            {
              in_cor = true;
              break;
            } 
          }
         
        }

        if(corridors.size() == 0 || in_cor == false)
        { 
          
            vertices.clear();
            vertices_.clear();
            grids_set.clear();
            grids_set_.clear();

            Eigen::Vector3d seed_pt = np;
            grids_set_ = global_map_manager -> convexNClusterInflation(global_map_manager -> posW2Index(global_map_manager ->point324(seed_pt) ));
            //grids_set_ = global_map_manager -> convexClusterInflation(global_map_manager -> posW2Index(Eigen::Vector4d(target_pos(0), target_pos(1),0,1)) );
            std::cout<<"[cluster size = ]" <<grids_set_.size()<<std::endl;
            for(int i = 0 ; i < grids_set_.size(); i++)
            {
              grids_set.push_back(Vector3<double>(grids_set_[i](0), grids_set_[i](1), 0));
            }
            auto hull = quickHull.getConvexHull(grids_set, true, false); 
            auto vertexBuffer = hull.getVertexBuffer();
            int vertex_num = (int)vertexBuffer.size();

            for(int i = 0; i < vertex_num; i++)
            {
                Eigen::Vector3d vertex(vertexBuffer[i].x, vertexBuffer[i].y, vertexBuffer[i].z);  
                vertices.push_back(vertex);
            } 
            for(int i = 0 ; i < vertices.size(); i++)
            {
              if(  (vertices[i] - seed_pt).norm() > global_map_manager -> getResolution() )
              {
                vertices_.push_back(vertices[i]);
              }
            }
            vertices_ = sortVertices(vertices_);

            //vis_render.renderSMC(vertices_);
            PolygonCorridor new_corridor(vertices_);
            if(i != 0)
            {
              new_corridor.setSeed(np);
            }
            corridors.push_back(new_corridor);
            vis_render.renderSMC(corridors);
        }
      }
    }
  }

  PolygonCorridor UGVPlannerManager::cluster2Corridor(vector<Eigen::Vector3d> grids_set_  ,Eigen::Vector3d seed_pt)
  {
      vector<Eigen::Vector3d> vertices;
      vector<Eigen::Vector3d> vertices_;
      QuickHull<double> quickHull;
      vector<Vector3<double>> grids_set;
      std::cout<<"[cluster size = ]" <<grids_set_.size()<<std::endl;
      for(int i = 0 ; i < grids_set_.size(); i++)
      {
        grids_set.push_back(Vector3<double>(grids_set_[i](0), grids_set_[i](1), 0));
      }
      auto hull = quickHull.getConvexHull(grids_set, true, false); 
      auto vertexBuffer = hull.getVertexBuffer();
      int vertex_num = (int)vertexBuffer.size();

      for(int i = 0; i < vertex_num; i++)
      {
          Eigen::Vector3d vertex(vertexBuffer[i].x, vertexBuffer[i].y, vertexBuffer[i].z);  
          vertices.push_back(vertex);
      }
      //vertices = sortVertices(vertices);
      for(int i = 0 ; i < vertices.size(); i++)
      {
        if(  (vertices[i] - seed_pt).norm() > 5*global_map_manager -> getResolution() )
        {
          vertices_.push_back(vertices[i]);
        }
      }
      vertices_ = sortVertices(vertices_);
      PolygonCorridor new_corridor(vertices_);
      return new_corridor;
  }


  Eigen::Vector3d UGVPlannerManager::findClosestEdgeNormal(PolygonCorridor corridor, Eigen::Vector3d pt)
  {
    Eigen::MatrixXd A = corridor.getA();
    int edge_count = A.rows();
    int min_dist_row = 0;
    double min_dist = 1e10;

    double a,b,c,d, dist;
    for(int i = 0 ; i < edge_count; i++)
    {
      a = A(i,0);
      b = A(i,1);
      c = A(i,2);
      d = sqrt(a*a + b*b);
      dist = abs(a*pt(0) + b*pt(1) + c) / d;
      if(dist < min_dist)
      {
        min_dist_row = i;
        min_dist = dist;
      }
    }
    a = A(min_dist_row,0);
    b = A(min_dist_row,1);
    Eigen::Vector3d edge_normal = Eigen::Vector3d(-a, -b, 0);
    edge_normal.normalize();
    cout<<"edge_normal = " << edge_normal<<endl;
    return edge_normal;
  }

  void UGVPlannerManager::generateSMC(vector<Eigen::Vector3d> path)
  {
    std::cout<<"[SMC] gene begin, path size = " <<path.size()<<std::endl;
    //vector<Eigen::Vector3d> vertices;
    //vector<Eigen::Vector3d> vertices_;
    //QuickHull<double> quickHull;
    //vector<Vector3<double>> grids_set;
    vector<Eigen::Vector3d> grids_set_;
    corridors.clear();

    // path.reserve()
    for(int i = path.size() - 1 ; i >= 0 ; i--)
    {
      if(corridors.size() == 0 || (corridors.size() != 0 && !corridors.back().isPointInPolygon(path[i])) )
      { 
          //vertices.clear();
          //vertices_.clear();
          //grids_set.clear();
          grids_set_.clear();

          Eigen::Vector3d seed_pt = path[i];
          grids_set_ = global_map_manager -> convexClusterInflation(global_map_manager -> posW2Index(global_map_manager ->point324(seed_pt) ));
          PolygonCorridor new_corridor = cluster2Corridor(grids_set_, seed_pt);
          //grids_set_ = global_map_manager -> convexClusterInflation(global_map_manager -> posW2Index(Eigen::Vector4d(target_pos(0), target_pos(1),0,1)) );
          //std::cout<<"[cluster size = ]" <<grids_set_.size()<<std::endl;
          //for(int i = 0 ; i < grids_set_.size(); i++)
          //{
          //  grids_set.push_back(Vector3<double>(grids_set_[i](0), grids_set_[i](1), 0));
          //}
          //auto hull = quickHull.getConvexHull(grids_set, true, false); 
          //auto vertexBuffer = hull.getVertexBuffer();
          //int vertex_num = (int)vertexBuffer.size();
//
          //for(int i = 0; i < vertex_num; i++)
          //{
          //    Eigen::Vector3d vertex(vertexBuffer[i].x, vertexBuffer[i].y, vertexBuffer[i].z);  
          //    vertices.push_back(vertex);
          //}
          ////vertices = sortVertices(vertices);
          //for(int i = 0 ; i < vertices.size(); i++)
          //{
          //  if(  (vertices[i] - seed_pt).norm() > global_map_manager -> getResolution() )
          //  {
          //    vertices_.push_back(vertices[i]);
          //  }
          //}
          //vertices_ = sortVertices(vertices_);
//
          ////vis_render.renderSMC(vertices_);
          //PolygonCorridor new_corridor(vertices_);
          if(i != 0)
          {
            new_corridor.setSeed(path[i-1]);
          }
          corridors.push_back(new_corridor);
          vis_render.renderSMC(corridors);
      }
      else
      {
          //std::cout<<"point in the front corridor" <<std::endl;
          //std::cout<<"PIFC,value = \n" << corridors.back().getDotnum(path[i]) << "\n sum = "<<corridors.back().getDotnum(path[i]).norm()<<std::endl;
          
      }
      //ros::Duration(0.1).sleep();
    }



  }

  //早期验证性函数，生成过waypoints的贝塞尔直线段
  void UGVPlannerManager::generateCurveByWps(vector<Eigen::Vector3d> waypoints )
  {
    int seg = waypoints.size() - 1;
    Eigen::VectorXd time     = Eigen::VectorXd::Ones(seg);
    time_allocation.clear();
    for(int i = 0 ; i < seg ; i++)
    {
      time_allocation.push_back(time(i));
    }
    Eigen::MatrixXd bezier_c = Eigen::MatrixXd::Zero(seg, 3*8);
    for(int i = 0 ; i < seg; i++)
    {
        Eigen::Vector3d local_startpt = waypoints[i];
        Eigen::Vector3d local_endpt = waypoints[i+1];
        for(int j = 0 ; j < 8; j++)
        {
            Eigen::Vector3d control_pt = local_startpt + (double(j)/8) * (local_endpt - local_startpt);
            bezier_c(i,j)      = control_pt(0);
            bezier_c(i,j + 8)  = control_pt(1);
            bezier_c(i,j + 16) = control_pt(2);
        }
    }
    //std::cout<<bezier_c<<std::endl;
    vis_render.visBezierTrajectory(bezier_basis, bezier_c, time );
  }


  vector<double> UGVPlannerManager::timeAllocate(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
  {
    double max_vel = 1.5;
    vector<Eigen::Vector3d> waypoints;
    vector<double> time_allocation;
    waypoints.push_back(start_pt);
    for(int i = 1; i < corridors.size(); i++)
    {
      waypoints.push_back(corridors[i].getSeed());
    }
    waypoints.push_back(end_pt);

    for(int i = 0; i < waypoints.size() - 1 ; i++)
    {
      time_allocation.push_back( (waypoints[i+1] - waypoints[i]).norm() / max_vel  );
    }
    return time_allocation;
  }

  void UGVPlannerManager::generateCurveByOptimizer(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, int seg)
  {
      Eigen::MatrixXd Qo_u = bezier_basis->getMQM_u();
      Eigen::MatrixXd Qo_l = bezier_basis->getMQM_l();
      Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(2,3);
      Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2,3);
      Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2,3);
      pos.row(0) = start_pt;
      pos(0,2) = 0;
      pos(1,2) = 0;
      pos.row(1) = end_pt;    
      vel.row(0) << 0.0, 0.0, 0.0;
      acc.row(0) << 0.0, 0.0, 0.0;

      Eigen::VectorXd bezier_time;

      
      time_allocation.clear();
      for(int i = 0 ; i < seg ; i++)
      {time_allocation.push_back(1.0);}
      
      //time_allocation = timeAllocate(start_pt, end_pt); 

      ros::Time time_before_optimization = ros::Time::now();
        
      int error_code = lan_bezier_optimizer -> bezierCurveGeneration(
                corridors,time_allocation, Qo_u, Qo_l, pos, vel, acc, 7 , 3, 1.5, 3.0);

      ros::Time time_after_optimization = ros::Time::now();
      std::cout<<"[PLANNER MANAGER] optimize done! cost " << time_after_optimization.toSec() - time_before_optimization.toSec() <<" s"<<std::endl;
      std::cout<<"[PLANNER MANAGER] optimizer return value: " << error_code << std::endl;

      bezier_coeff = lan_bezier_optimizer -> getPolyCoeff();
      bezier_time  = lan_bezier_optimizer -> getPolyTime();

      time_allocation = timeAllocate(start_pt, end_pt); 
      time_duration = 0.0;
      for(int i = 0 ; i < time_allocation.size(); i++)
      {
        time_duration += time_allocation[i];
      }
      time_duration   = bezier_time.sum();


      vis_render.visBezierTrajectory( bezier_basis, bezier_coeff, bezier_time );
  }


  void UGVPlannerManager::generateFinalMincoTraj(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, vector<Eigen::Vector3d> path, bool use_path_wp)
  {
    Eigen::MatrixXd initS;
    start_pt(2) = global_map_manager -> getGapByPosW3(start_pt);
    end_pt(2)   = global_map_manager -> getGapByPosW3(end_pt);

    initS.setZero(3, 3);
    initS.col(0) = start_pt;
    initS.col(1) = Eigen::Vector3d(0, 0, 0);
    initS.col(2) = Eigen::Vector3d(0, 0, 0);

    double space_resolution = 0.3;
    /// get waypoints
    vector<Eigen::Vector3d> waypoints;
    Eigen::Vector3d last_pos, pos;
    double dt = 0.001;
    double mileage = 0.0 , lastwp_mileage = 0.0;

    // corridor-based method
    if(!use_path_wp)
    {
      int current_seg   = 0;
      int segment_num   = bezier_coeff.rows();
      last_pos = start_pt;
      for(double t = 0.0; current_seg < segment_num; current_seg++)
      {
        t = 0.0;
        while(t <= 1.0)
        {
            pos    = bezier_basis -> getPosFromBezier( bezier_coeff, t, current_seg );
            pos(2) = global_map_manager -> getGapByPosW3(pos);
            mileage += (pos - last_pos).norm();
            if(mileage - lastwp_mileage >= space_resolution)
            {
              waypoints.push_back(pos);
              lastwp_mileage  = mileage;
            }
            last_pos = pos;
            t += dt;
        }
      }
      waypoints.push_back(end_pt);
    }

    // esdf-based method
    else
    {
      int di = space_resolution / global_map_manager -> p_grid_resolution;
      for(int i = path.size() - 1 ; i >= 0 ; i -= di)
      {
          pos = path[i];
          pos(2) = global_map_manager -> getGapByPosW3(pos);
          waypoints.push_back(pos);
      }
      waypoints.push_back(end_pt);
    }
    
    // vis_render.renderPoints(waypoints, Eigen::Vector3d(0,0,0.9),1, 0.1, 200);
    int pieceN = waypoints.size();

    global_map_manager -> jps.clear();
    global_map_manager -> publishESDFMap();
    if (minco_traj_optimizer -> generate_traj(initS, waypoints, pieceN, final_trajectory, false))
    {
      vis_render.visPolynomialTrajectory(final_trajectory, Eigen::Vector3d(1,1,0), 7);

#ifdef CONSIDER_JUMPING
      checkJps();
      global_map_manager -> publishESDFMap();
      if(global_map_manager -> jps.size() == 0)
      {
        cout << "[minco optimizer]: optimization success." << endl;
        PublishTraj();
        minco_traj_optimizer -> deleteX_();
        vis_render.visPolynomialTrajectory(final_trajectory, Eigen::Vector3d(0,1,0), 10);
      }
      else
      {
        if (minco_traj_optimizer -> generate_traj(initS, waypoints, pieceN, final_trajectory, true))
        {
          cout << "[minco optimizer]: optimization success." << endl;
          checkJps();
          pubJps();
          PublishTraj();
          vis_render.visPolynomialTrajectory(final_trajectory, Eigen::Vector3d(0,1,0), 10);
        }
        else
        {
          ROS_WARN("[minco optimizer]: optimization failed in step direction cast.");
          pubJps();
          PublishTraj();
        }
      }
#else
    cout << "[minco optimizer]: optimization success." << endl;
    PublishTraj();
#endif

    }
    else
    {
      ROS_WARN("[minco optimizer]: optimization failed.");
    }
    
  }


  void UGVPlannerManager::pubJps()
  {
    std_msgs::Float64MultiArray jps;
    for(auto jp : global_map_manager -> jps)
    {
      jps.data.push_back(jp.takeoff_time);
    }
    jps_pub.publish(jps);
  }

  void UGVPlannerManager::checkJps()
  {
    global_map_manager -> jps.clear();
    Eigen::Vector3d pos, posa, posj,vel_dir;
    vector<Eigen::Vector3d> jps_vis;
    vector<Eigen::Vector3d> step_cluster;
    vector<PolygonCorridor> step_corridors;
    double t_duration = final_trajectory.getTotalDuration();
    double h1,h2;
    for(double t = 0.2; t < t_duration - 0.05; t += 0.05)
    {
        pos = final_trajectory.getPos(t);
        posa = final_trajectory.getPos(t + 0.05);
        posj = final_trajectory.getPos(t - 0.2);
        h1 = global_map_manager -> getHeightByPosW3(pos);
        h2 = global_map_manager -> getHeightByPosW3(posa);
        step_cluster.clear();       
        if(h2 - h1 > 0.09)
        {
          posa(2) = 0;
          step_cluster = global_map_manager -> convexNClusterInflation(global_map_manager -> posW2Index(global_map_manager ->point324(posa)) );
          PolygonCorridor step_corridor = cluster2Corridor(step_cluster, posa);
          step_corridors.push_back(step_corridor);
          vis_render.renderSMC(step_corridors);
          vel_dir = findClosestEdgeNormal(step_corridor, posa);
          global_map_manager -> jps.push_back(JumpPoint( Eigen::Vector2d(pos(0),pos(1)) , Eigen::Vector2d(vel_dir(0),vel_dir(1)) , t-0.2) );
          jps_vis.push_back(pos);
          jps_vis.push_back(posj);
          t += 0.05;
        }
    }
    vis_render.renderPoints(jps_vis, Eigen::Vector3d(1,0,0), 1, 0.1, 120);
  }  


  void UGVPlannerManager::trajPlanning(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt , vector<Eigen::Vector3d> path)
  {
    //  generateCurveByOptimizer(start_pt, end_pt, corridors.size()); // trr method
     generateFinalMincoTraj(start_pt, end_pt,path, true);
  }

  void UGVPlannerManager::globalReplan()
  {
    vector<Eigen::Vector3d> path_0 = global_map_manager -> AstarPathSearch( now_pos, target_pos);
    vis_render.renderPoints(path_0, Eigen::Vector3d(0.6,0.6,0.1),0, 0.05, 1);

    //_polyhedronGenerator -> corridorIncreGeneration(path_0, _poly_array_msg);
    if (path_0.size() > 0)
    {
      // corridor based method
      // generateSMC(path_0);
      // generateOCC();

      // esdf based method
      trajPlanning( now_pos, target_pos, path_0);
    }
    else
    {
      ROS_WARN(" No path! ");
    }
  }












} //namespace