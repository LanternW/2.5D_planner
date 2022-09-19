#ifndef LOCAL_MAP_MANAGER_H
#define LOCAL_MAP_MANAGER_H


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <string.h>

#define inf 1>>20
using namespace std;

namespace ugv_planner
{
    struct GridNode;
    typedef GridNode* GridNodePtr;  /*    GridNode: DT for A* search    */

    class GridNode
    {   
    public:
        int id;        // 1--> open set, -1 --> closed set, 0 --> unknown
        Eigen::Vector3d coord; 
        Eigen::Vector2i index;
        
        double cost;
        GridNodePtr father;

        GridNode(Eigen::Vector2i _index, Eigen::Vector3d _coord){  
            id = 0;
            index = _index;
            coord = _coord;

            cost  = 0.0;
            father = NULL;
        }

        GridNode(Eigen::Vector3d _coord, double _cost, Eigen::Vector2i _index, GridNodePtr _father){  
            id = 0;
            index = _index;
            coord = _coord;

            cost  = _cost;
            father = _father;
        }
        GridNode(Eigen::Vector3d _coord, double _cost, Eigen::Vector2i _index, GridNodePtr _father, int _id){  
            id = _id;
            index = _index;
            coord = _coord;

            cost  = _cost;
            father = _father;
        }

        GridNode(){};
        ~GridNode(){};
    };

    class JumpPoint
    {
    public:
       JumpPoint(Eigen::Vector2d _takeoff_pos, Eigen::Vector2d _velocity_dir, double _takeoff_time)
       {
           takeoff_pos  = _takeoff_pos;
           velocity_dir = _velocity_dir;
           takeoff_time = _takeoff_time;
       }
    public:
       double takeoff_time;
       Eigen::Vector2d takeoff_pos;
       Eigen::Vector2d velocity_dir; 
    };

    class GirdInformation
    {
    public:
        GirdInformation(){is_occupied = false; is_occupied_flate = false,gap = 10000.0; height = 0; roughness = 0.0; esdf_var = 1e10; step_esdf_var = 1e10;esdf_g = Eigen::Vector3d(0,0,0);}
    public:
        bool   is_occupied;             //可通行性
        bool   is_occupied_flate;
        double gap;                     //落差
        double height;                  //
        double roughness;               //粗糙度
        double esdf_var;                //as its name
        double step_esdf_var;                //as its name
        Eigen::Vector3d esdf_g;         //as its name , maybe useless
    };

    class LanGridMapManager
    {
        public:
            LanGridMapManager();
            ~LanGridMapManager();

            void rcvGlobalMapHandler(const sensor_msgs::PointCloud2& global_map);
            void rcvOdomHandler(const nav_msgs::Odometry odom);

            void init(ros::NodeHandle& node_handler);

            // 是否被占用
            bool is_occupied(Eigen::Vector4d posw, int flate);
            bool is_occupiedI(Eigen::Vector2i index, int flate);
            bool is_occupied_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2, int flate);

            // 主要用于占用珊格的椭球生成
            bool is_not_occupied(Eigen::Vector4d posw, int flate);
            bool is_not_occupiedI(Eigen::Vector2i index, int flate);
            bool is_not_occupied_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2, int flate);

            bool is_step(Eigen::Vector4d posw);
            bool is_stepI(Eigen::Vector2i index);
            bool is_not_step_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2);

            bool isBoundaryOcc(int index_x , int index_y);
            bool isBoundaryStep(int index_x , int index_y);

            double getGapByI(Eigen::Vector2i index);
            double getGapByPosW3(Eigen::Vector3d posw)
            {
                return getGapByI(posW2Index(point324(posw)));
            }

            double getResolution(){return p_grid_resolution;}

            bool indexInMap(int index_x, int index_y);
            bool indexInMap(Eigen::Vector2i index)
            {
                return indexInMap(index(0), index(1));
            }
            bool posInMap(Eigen::Vector4d pos);  //posM
            bool posWInMap(Eigen::Vector4d pos);

            double getHeightByI(Eigen::Vector2i index);
            double getHeightByPosW3(Eigen::Vector3d posw)
            {
                return getHeightByI(posW2Index(point324(posw)));
            }
            
            Eigen::Vector4d point324(Eigen::Vector3d point3)
            {
                Eigen::Vector4d point4;
                point4.block(0,0,3,1) = point3;
                point4(3) = 1;
                return point4;
            }
            Eigen::Vector3d point423(Eigen::Vector4d point4)
            {
                Eigen::Vector3d point3;
                point3 = point4.block(0,0,3,1);
                return point3;
            }

            int toAddress(int x, int y){return x * grid_map.info.width + y ;}
            bool isAddressOutOfMap(int addr){return (addr >= map_index_xmax * map_index_ymax) || (addr < 0);}
            Eigen::Vector2i posM2Index(Eigen::Vector4d pos);
            Eigen::Vector2i posW2Index(Eigen::Vector4d pos);
            Eigen::Vector4d index2PosW(Eigen::Vector2i index);
            Eigen::Vector4d posW2posM(Eigen::Vector4d posw)
            {
                posw(1) = -posw(1);
                Eigen::Vector4d posm  = Tm_w * posw;
                return posm;
            }


            Eigen::Vector4d posM2posW(Eigen::Vector4d posm)
            {
                Eigen::Vector4d posw  = Tm_w.inverse() * posm;
                posw(1) = - posw(1);
                return posw;
            }
            
            vector<Eigen::Vector3d> AstarPathSearch(Eigen::Vector3d start, Eigen::Vector3d end);
    

            ////// convex cluster generate
            bool checkConvexity(vector<Eigen::Vector3d> C , Eigen::Vector3d pos);
            vector<Eigen::Vector3d> getNeighbors( vector<Eigen::Vector3d> set , vector<Eigen::Vector3d> C_plus, vector<Eigen::Vector3d> C);
            vector<Eigen::Vector3d> convexClusterInflation(Eigen::Vector2i seed_index);

            ////// convex cluster generate for occupied grid
            bool checkNConvexity(vector<Eigen::Vector3d> C , Eigen::Vector3d pos);
            vector<Eigen::Vector3d> getNNeighbors( vector<Eigen::Vector3d> set , vector<Eigen::Vector3d> C_plus, vector<Eigen::Vector3d> C);
            vector<Eigen::Vector3d> convexNClusterInflation(Eigen::Vector2i seed_index);


            ////// esdf
            void fillESDF(int t_distance); 
            void fillStepESDF(int t_distance); 
            void flateMap(int t_distance);
            void publishESDFMap();
            double calcESDF_Cost(const Eigen::Vector3d& pos_w);
            Eigen::Vector3d calcESDF_Grid(const Eigen::Vector3d& pos_w);

            double calcStepESDF_Cost(const Eigen::Vector3d& pos_w);
            Eigen::Vector3d calcStepESDF_Grid(const Eigen::Vector3d& pos_w);

            double getBlinearGap(const Eigen::Vector3d& pos_w);
            Eigen::Vector3d getBlinearGapGrad(const Eigen::Vector3d& pos_w);
            void calcZ_CostGrad(const Eigen::Vector3d& pos_w , double &z_cost , Eigen::Vector3d& z_grad);
            
            void getGradCostP(const Eigen::Vector3d p ,double& cost, Eigen::Vector3d& grad);

            ////// step dir con
            bool getStepDirCostAndGrad(const Eigen::Vector3d p, const Eigen::Vector3d v,
                                        Eigen::Vector3d& gradp,
                                        Eigen::Vector3d& gradv,
                                        double& costp, 
                                        double& costv);
            

            void setParam(ros::NodeHandle& nh);


        private:

            GirdInformation* grid_information = NULL;
            ros::NodeHandle nh;
            ros::Subscriber global_map_sub;
            ros::Subscriber odometry_sub;
            ros::Publisher  grid_map_pub;
            ros::Publisher  esdf_vis_pub;


            Eigen::Matrix4d Tm_w;
            nav_msgs::Odometry      ugv_odom;
            nav_msgs::OccupancyGrid grid_map;
        
        public:
            double *esdf_var = NULL;
            double *step_esdf_var = NULL;
            double map_length ;
            double map_width ;
            double map_index_xmax;
            double map_index_ymax;
            double p_grid_resolution;
            double truncation_distance;

            double max_height;            
            double min_height;         
            double max_jump_height;

            double left_boundary   ;
            double right_boundary  ;
            double up_boundary     ;
            double down_boundary   ;

            vector<JumpPoint> jps;

            typedef shared_ptr<LanGridMapManager> Ptr;
    };
}

#endif