#ifndef _CORRIDOR_H_
#define _CORRIDOR_H_

#include <stdlib.h>

#include <Eigen/Eigen>
#include <ros/ros.h>


namespace ugv_planner
{

  class PolygonCorridor
  {
  public:
    PolygonCorridor(vector<Eigen::Vector3d> vertices_)
    {
      vertices = vertices_;
      int vertices_num = vertices_.size();
      n = vertices_num;
      A = Eigen::MatrixXd::Zero(n, 3);

      Eigen::Vector3d line_coff;
      Eigen::Vector3d p1,p2,p3;
      for(int i = 0 ; i < vertices_num ; i++)
      {

          p1 = vertices_[(i % vertices_num)];
          p2 = vertices_[((i+1) % vertices_num)];
          p3 = vertices_[((i+2) % vertices_num)];
          line_coff(0) = p2(1) - p1(1);
          line_coff(1) = p1(0) - p2(0);
          line_coff(2) = p2(0)*p1(1) - p1(0)*p2(1);

          p3(2) = 1;
          if( line_coff.transpose() * p3 > 0 )
          {
            line_coff *= -1;
          }
          A.row(i) = line_coff.transpose();
      }
    }

    bool isPointInPolygon(Eigen::Vector3d pos)
    {
      pos(2) = 1;  //p = [x,y,1]'
      Eigen::VectorXd b = A * pos;
      // b 《 0
      for(int i = 0 ; i < n ; i ++)
      {
        if(b[i] > 0 ){return false;}
      }
      return true;
    }

    Eigen::VectorXd getDotnum(Eigen::Vector3d pos)
    {
      pos(2) = 1;  //p = [x,y,1]'
      Eigen::VectorXd b = A * pos;

      return b;
    }

    void setSeed(Eigen::Vector3d s)
    {
      seed = s;
    }

    Eigen::Vector3d getSeed(){return seed;}
    int getConstraintNum(){return n;}
    Eigen::MatrixXd getA(){return A;}
    vector<Eigen::Vector3d> getVertices(){return vertices;}

  private:
    vector<Eigen::Vector3d> vertices;
    Eigen::Vector3d seed;
    Eigen::MatrixXd A;                     //Ax < 0 means the point 'x' in polygon
    int n;
  };
}

#endif