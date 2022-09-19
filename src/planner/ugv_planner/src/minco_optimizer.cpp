#include <minco_opt/traj_opt.h>

#include <minco_opt/lbfgs_raw.hpp>

namespace ugv_planner {

static double rhoP_tmp_;

// SECTION  variables transformation and gradient transmission
static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}
static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}
static void forwardT(const double& t, Eigen::Ref<Eigen::VectorXd> vecT) {
  vecT.setConstant(expC2(t));
}
static void addLayerTGrad(const double& t,
                          const Eigen::Ref<const Eigen::VectorXd>& gradT,
                          double& gradt) {
  gradt = gradT.sum() * gdT2t(t);
}

// !SECTION variables transformation and gradient transmission

// SECTION object function
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  const double& t = x[0];
  double& gradt = grad[0];
  Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
  Eigen::Map<const Eigen::MatrixXd> tailS(x + obj.dim_t_ + 3 * obj.dim_p_, 3, 3);
  Eigen::Map<Eigen::MatrixXd> gradP(grad + obj.dim_t_, 3, obj.dim_p_);
  Eigen::Map<Eigen::MatrixXd> gradtailS(grad + obj.dim_t_ + 3 * obj.dim_p_, 3, 3);
  Eigen::VectorXd T(obj.N_);
  forwardT(t, T);
  obj.jerkOpt_.generate(P, tailS, T);
  double cost = obj.jerkOpt_.getTrajJerkCost();
  obj.jerkOpt_.calGrads_CT();
  obj.addTimeIntPenalty(cost);
  obj.jerkOpt_.calGrads_PT();
  obj.jerkOpt_.gdT.array() += obj.rhoT_;
  cost += obj.rhoT_ * T.sum();
  addLayerTGrad(t, obj.jerkOpt_.gdT, gradt);
  gradP = obj.jerkOpt_.gdP;
  // gradtailS = obj.jerkOpt_.gdTail;

  // make the tailS near
  // cost += (tailS - obj.finalS_).squaredNorm();
  // gradtailS += 2 * (tailS - obj.finalS_);

  return cost;
}


// !SECTION object function
static inline int earlyExit(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  //if (obj.pause_debug_) {
  if (false) {
    const double& t = x[0];
    Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
    Eigen::Map<const Eigen::MatrixXd> tailS(x + obj.dim_t_ + 3 * obj.dim_p_, 3, 3);
    Eigen::VectorXd T(obj.N_);
    forwardT(t, T);
    obj.jerkOpt_.generate(P, tailS, T);
    auto traj = obj.jerkOpt_.getTraj();
    obj.drawDebug(traj);

    // std::vector<Eigen::Vector3d> int_waypts;
    // for (const auto& piece : traj) {
    //   const auto& dur = piece.getDuration();
    //   for (int i = 0; i < obj.K_; ++i) {
    //     double t = dur * i / obj.K_;
    //     int_waypts.push_back(piece.getPos(t));
    //   }
    // }
    // obj.drawDebugWp(int_waypts);
    // obj.visPtr_->visualize_pointcloud(int_waypts, "int_waypts");

    // NOTE pause
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  return k > 1e3;
}

bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
                            const std::vector<Eigen::Vector3d>& path,
                            const int N,
                            Trajectory& traj,
                            bool is_continuing) {

  N_ = N;
  
  
  dim_t_ = 1;
  dim_p_ = N_ - 1;
  
  if( !is_continuing )
  {
    x_ = new double[dim_t_ + 3 * dim_p_ + 9];
  }
  double& t = x_[0];
  Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_);
  Eigen::Map<Eigen::MatrixXd> tailS(x_ + dim_t_ + 3 * dim_p_, 3, 3);

  // NOTE set boundary conditions
  initS_ = iniState;
  double tempNorm = initS_.col(1).norm();
  initS_.col(1) *= tempNorm > vmax_ ? (vmax_ / tempNorm) : 1.0;
  tempNorm = initS_.col(2).norm();
  initS_.col(2) *= tempNorm > amax_ ? (amax_ / tempNorm) : 1.0;
  finalS_.setZero(3, 3);
  finalS_.col(0) = path.back();
  tailS = finalS_;
  // set initial guess
  double path_len = 0.0;
  for (auto i=0;i<path.size()-1;i++)
  {
    path_len+=(path[i]-path[i+1]).norm();
  }
  double T0 = 1.0 * path_len / vmax_ / N_;

  t = logC2(T0);
 
  int size_path = path.size();

  for (int i = 0; i < N_ - 1; ++i) {
    P.col(i) = path[i];
  }
  jerkOpt_.reset(initS_, N_);
  // NOTE optimization
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1e-5;
  lbfgs_params.min_step = 1e-20;
  lbfgs_params.delta = 1e-5;
  lbfgs_params.line_search_type = 0;
  double minObjectiveXY , minObjectiveZ;

  rhoP_tmp_ = rhoP_;

  auto opt_ret1 = lbfgs::lbfgs_optimize(dim_t_ + 3 * dim_p_ + 9, 
                                       x_, 
                                       &minObjectiveXY,
                                       &objectiveFunc, nullptr,
                                       &earlyExit, this, &lbfgs_params);

  std::cout << "\033[32m"
            << "ret: " << opt_ret1 << "\033[0m" << std::endl;
  if (pause_debug_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  if (opt_ret1 < 0) {
    if(is_continuing)
    {
      delete[] x_;
    }
    return false;
  }
  Eigen::VectorXd T(N_);
  forwardT(t, T);
  jerkOpt_.generate(P, tailS, T);
  traj = jerkOpt_.getTraj();
  if(is_continuing)
  {
    delete[] x_;
  }
  
  return true;
}


void TrajOpt::addTimeIntPenalty(double& cost) {
  Eigen::Vector3d pos, vel, acc, jer;
  Eigen::Vector3d grad_tmp, grad_tmp_p, grad_tmp_v ;
  double cost_tmp, cost_tmp_p, cost_tmp_v;
  Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
  double s1, s2, s3, s4, s5;
  double step, alpha;
  Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
  double gradViolaPt, gradViolaVt, gradViolaAt;
  double omg;

  int innerLoop;
  for (int i = 0; i < N_; ++i) {
    const auto& c = jerkOpt_.b.block<6, 3>(i * 6, 0);
    step = jerkOpt_.T1(i) / K_;
    s1 = 0.0;
    innerLoop = K_ + 1;

    for (int j = 0; j < innerLoop; ++j) {
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      alpha = 1.0 / K_ * j;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;

      omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

    
      if (grad_cost_p(pos, grad_tmp, cost_tmp)) {
        
        gradViolaPc = beta0 * grad_tmp.transpose();
        gradViolaPt = alpha * grad_tmp.dot(vel);
        jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
        jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaPt);
        cost += omg * step * cost_tmp;
      }
      

      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        gradViolaVc = beta1 * grad_tmp.transpose();
        gradViolaVt = alpha * grad_tmp.dot(acc);
        jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
        jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaVt);
        cost += omg * step * cost_tmp;
      }
      if (grad_cost_a(acc, grad_tmp, cost_tmp)) {
        gradViolaAc = beta2 * grad_tmp.transpose();
        gradViolaAt = alpha * grad_tmp.dot(jer);
        jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
        jerkOpt_.gdT(i) += omg * (cost_tmp / K_ + step * gradViolaAt);
        cost += omg * step * cost_tmp;
      }

      if (grad_cost_d(pos,vel, grad_tmp_p, grad_tmp_v, cost_tmp_p, cost_tmp_v)) {
        
        gradViolaPc = beta0 * grad_tmp_p.transpose();
        gradViolaPt = alpha * grad_tmp_p.dot(vel);
        jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaPc * cost_tmp_v;
        //jerkOpt_.gdT(i) += omg * (cost_tmp_p / K_ + step * gradViolaPt ) * cost_tmp_v;

        gradViolaVc = beta1 * grad_tmp_v.transpose();
        gradViolaVt = alpha * grad_tmp_v.dot(acc);
        jerkOpt_.gdC.block<6, 3>(i * 6, 0) += omg * step * gradViolaVc * cost_tmp_p;
        jerkOpt_.gdT(i) += omg * (cost_tmp_v / K_ + step * gradViolaVt ) * cost_tmp_p;
        cost += omg * step * cost_tmp_p * cost_tmp_v;
      }

      s1 += step;
    }
  }
}

bool TrajOpt::grad_cost_p(const Eigen::Vector3d& p,
                          Eigen::Vector3d& gradp,
                          double& costp) {
  
  // costp = gridmapPtr_->getCostWithGrad(p, gradp);
  double esdf = 0 ;
  double z_cost = 0;
  Eigen::Vector3d gp(Eigen::Vector3d::Zero());
  //double step_esdf = 0;
  //Eigen::Vector3d sgp(Eigen::Vector3d::Zero());  
  Eigen::Vector3d z_grid(Eigen::Vector3d::Zero()); 
  costp = 0;
  gradp = Eigen::Vector3d::Zero();
  //grid_map_manager -> getGradCostP(p,costp,gradp);


  esdf = grid_map_manager -> calcESDF_Cost(p);
  gp   = grid_map_manager -> calcESDF_Grid(p);

  //step_esdf =  grid_map_manager -> calcStepESDF_Cost(p);
  //sgp       =  grid_map_manager -> calcStepESDF_Grid(p);


  grid_map_manager -> calcZ_CostGrad(p,z_cost ,z_grid);
  costp +=  rhoP_tmp_ * z_cost;
  gradp +=  rhoP_tmp_ * z_grid;

  //if ( (esdf       > grid_map_manager -> truncation_distance || esdf == 0) &&
  //     (step_esdf  > grid_map_manager -> truncation_distance || step_esdf == 0))
  //{
  if ( esdf > grid_map_manager -> truncation_distance || esdf == 0 )
  {
    return true;
  }
  else
  {    
    costp +=  rhoP_tmp_ * pow(grid_map_manager -> truncation_distance - esdf, 3);
    gradp +=  rhoP_tmp_ * 3 * pow(grid_map_manager -> truncation_distance - esdf, 2) * gp;


    //costp +=  rhoP_tmp_ * pow(grid_map_manager -> truncation_distance - step_esdf, 3);
    //gradp +=  rhoP_tmp_ * 3 * pow(grid_map_manager -> truncation_distance - step_esdf, 2) * sgp;

    return true;
  }
  
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  
  gradv = Eigen::Vector3d::Zero();
  costv = 0;
  double vpen  = v.squaredNorm() - vmax_ * vmax_;
  double vzpen = v(2)*v(2) - vmax_z_ * vmax_z_;   // vz_max = 0.2
  if(vpen > 0 || vzpen > 0)
  {
    if (vzpen > 0) {
      gradv += rhoV_ * 6 * vzpen * vzpen * Eigen::Vector3d(0,0,v(2));
      costv += rhoV_ * vzpen * vzpen * vzpen;
    }
    if (vpen > 0) {
      gradv += rhoV_ * 6 * vpen * vpen * v;
      costv += rhoV_ * vpen * vpen * vpen;
    }
    return true;
  }

  return false;

  //double vpen  = v.squaredNorm() - vmax_ * vmax_;
  //if (vpen > 0) {
  //  gradv = rhoV_ * 6 * vpen * vpen * v;
  //  costv = rhoV_ * vpen * vpen * vpen;
  //  return true;
  //}
  //return false;
}

bool TrajOpt::grad_cost_a(const Eigen::Vector3d& a,
                          Eigen::Vector3d& grada,
                          double& costa) {

  grada = Eigen::Vector3d::Zero();
  costa = 0;
  double apen  = a.squaredNorm() - amax_ * amax_;
  double azpen = a(2)*a(2) - amax_z_ * amax_z_;
  if(apen > 0 || azpen > 0)
  {
    if (azpen > 0) {
      grada += rhoA_ * 6 * azpen * azpen * Eigen::Vector3d(0,0,a(2));
      costa += rhoA_ * azpen * azpen * azpen;
    }
    if (apen > 0) {
      grada += rhoA_ * 6 * apen * apen * a;
      costa += rhoA_ * apen * apen * apen;
    }
    return true;
  }

  return false;

  //double apen = a.squaredNorm() - amax_ * amax_;
  //if (apen > 0) {
  //  grada = rhoA_ * 6 * apen * apen * a;
  //  costa = rhoA_ * apen * apen * apen;
  //  return true;
  //}
  //return false;
}

bool TrajOpt::grad_cost_d(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& v, 
                           Eigen::Vector3d& gradp,
                           Eigen::Vector3d& gradv,
                           double& costp,
                           double& costv)
{
  if( grid_map_manager -> getStepDirCostAndGrad(p,v,gradp, gradv,costp,costv ) ) 
  {
    //std::cout<< "cost_v = "<<costv<<" | cost_p = "<<costp<<std::endl
    //         << "gradv = "<<gradv<<" | grad_v = "<<gradv<<std::endl;

    costv *= 1000;
    gradv *= 1000;
    return true;
  }
  return false;

}

void TrajOpt::drawDebug(Trajectory end_path)
{
  int id = 0;
  visualization_msgs::Marker sphere, line_strip;
  sphere.header.frame_id = line_strip.header.frame_id = "world";
  sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
  sphere.id = id;
  line_strip.id = id + 1000;
  id++;

  sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  sphere.color.r = line_strip.color.r = 1;
  sphere.color.g = line_strip.color.g = 0;
  sphere.color.b = line_strip.color.b = 1;
  sphere.color.a = line_strip.color.a = 1;
  sphere.scale.x = 0.05;
  sphere.scale.y = 0.05;
  sphere.scale.z = 0.05;
  line_strip.scale.x = 0.05 / 2;
  geometry_msgs::Point pt;

  double dur = end_path.getDurations().sum();
  for (double i = 0; i < dur - 1e-4; i+=0.1)
  {
    Eigen::Vector3d dur_p = end_path.getPos(i);
    pt.x = dur_p(0);
    pt.y = dur_p(1);
    pt.z = dur_p(2);
    line_strip.points.push_back(pt);
  }
  debug_pub.publish(line_strip);
}

void TrajOpt::drawDebugWp(std::vector<Eigen::Vector3d> front_path)
{
  int id = 0;
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id++;
  // kino_pub_.publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = 0;
  mk.color.g = 1;
  mk.color.b = 0;
  mk.color.a = 1;

  mk.scale.x = 0.075;
  mk.scale.y = 0.075;
  mk.scale.z = 0.075;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(front_path.size()); i++) {
    pt.x = front_path[i](0);
    pt.y = front_path[i](1);
    pt.z = 0;
    mk.points.push_back(pt);
  }
  debug_wp_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

}  // namespace rm_planner