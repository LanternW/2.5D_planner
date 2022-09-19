
#include "grid_map/global_map_manager.h"
#include <visualization_msgs/Marker.h>

const double pi = 3.1415926535;



namespace ugv_planner
{

    LanGridMapManager::LanGridMapManager()
    {
        p_grid_resolution    = 0.1;
        map_length           = 20.0;
        map_width            = 10.0;
        truncation_distance  = 0.5;

        //max_height           = 0.8;
        min_height           = 0.3;
        max_jump_height      = 0.3;

        Tm_w                 = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd v(pi / 2, Eigen::Vector3d(0, 0, 1));
        Tm_w.block(0,0,3,3) = v.matrix();


    }
    LanGridMapManager::~LanGridMapManager(){}


    void LanGridMapManager::setParam(ros::NodeHandle& nh)
    {
        nh.param("max_height", max_height, 1.0);
    }

    void LanGridMapManager::rcvGlobalMapHandler(const sensor_msgs::PointCloud2& global_map)
    {
        left_boundary    = -100000.0;
        right_boundary   = 100000.0;
        up_boundary      = -100000.0;
        down_boundary    = 100000.0;

    	pcl::PointCloud<pcl::PointXYZ> global_cloud;
    	pcl::fromROSMsg(global_map, global_cloud);
        Eigen::Vector4d pt_w,  pt_m;

        for(int i = 0 ; i < global_cloud.points.size(); i++)
        {
            if(global_cloud.points[i].x > up_boundary){
                up_boundary = global_cloud.points[i].x;
            }
            if(global_cloud.points[i].x < down_boundary){
                down_boundary = global_cloud.points[i].x;
            }
            if(global_cloud.points[i].y > left_boundary){
                left_boundary = global_cloud.points[i].y;
            }
            if(global_cloud.points[i].y < right_boundary){
                right_boundary = global_cloud.points[i].y;
            }
        }

        map_length = left_boundary - right_boundary;
        map_width  =  up_boundary  - down_boundary;

        Tm_w(0,3)  = -right_boundary;
        Tm_w(1,3)  = -down_boundary;


        grid_map.header.frame_id     = "world";
        grid_map.info.resolution     = p_grid_resolution;
        grid_map.info.height         = int(map_length/p_grid_resolution) + 2 ;
        grid_map.info.width          = int(map_width/p_grid_resolution) + 2 ;

        cout<<"[LOCALMAP DEBUG] index = "<<grid_map.info.height << " | " <<grid_map.info.width<<endl;

        pt_w(0) = down_boundary;
        pt_w(1) = right_boundary;
        pt_w(2) = 0;
        pt_w(3) = 1;
        grid_map.info.origin.position.x = pt_w(0);
        grid_map.info.origin.position.y = pt_w(1);
        grid_map.info.origin.position.z = pt_w(2);

        //map_index_xmax = int(map_length / p_grid_resolution);
        //map_index_ymax = int(map_width  / p_grid_resolution);

        map_index_xmax = grid_map.info.height;
        map_index_ymax = grid_map.info.width;


        int floor[grid_map.info.width * grid_map.info.height] = {0};   // [0,100]

        vector<double> sorted_grid_map[grid_map.info.width * grid_map.info.height];
        if (grid_information != NULL){ delete[] grid_information;}

        grid_information = new GirdInformation[grid_map.info.width * grid_map.info.height];


        int index_x = 0;
        int index_y = 0;
        double new_h = 0;
        for(int i = 0 ; i < global_cloud.points.size(); i++)
        {

            pt_w(0) = global_cloud.points[i].x;
            pt_w(1) = global_cloud.points[i].y;
            pt_w(2) = global_cloud.points[i].z;
            pt_w(3) = 1;
            new_h    = pt_w(2);

            pt_m    = posW2posM(pt_w);
            
            index_x = pt_m(0) / p_grid_resolution;
            index_y = pt_m(1) / p_grid_resolution;

            //sorted_grid_map[index_x * grid_map.info.width + index_y].push_back(new_h);
            sorted_grid_map[toAddress(index_x, index_y)].push_back(new_h);
        }

/////////// get height info and gap info
        for(int i = 0 ; i < grid_map.info.width * grid_map.info.height ; i++)
        {
            sort(sorted_grid_map[i].begin(), sorted_grid_map[i].end());
            double h = 0;
            for(int j = 0 ; j < sorted_grid_map[i].size(); j++)
            {
                if(sorted_grid_map[i][j] > h && sorted_grid_map[i][j] < h +0.3)
                {
                    h = sorted_grid_map[i][j];
                    grid_information[i].height = h;
                    floor[i] = (h>1)?100 :h* 100;
                    if(h > max_jump_height){grid_information[i].is_occupied = true;}
                }
                else if(sorted_grid_map[i][j] >= h + 0.3)
                {
                    grid_information[i].gap = sorted_grid_map[i][j] - h;
                    grid_information[i].height = h;
                    floor[i] = (h>1)?100 :h* 100;
                    if(h > max_jump_height){grid_information[i].is_occupied = true;}
                    break;
                }
            }
        }


        // get is_occupied info
        //double center_h, sorround_h;
        //for(int i = 2 ; i < map_index_xmax - 2 ; i++)
        //{
        //    for(int j = 2 ; j < map_index_ymax - 2; j++)
        //    {
        //        center_h = grid_information[toAddress(i,j)].height;
        //        for(int dx = -1 ; dx <= 1 ; dx++)
        //        {
        //            for(int dy = -1 ; dy <= 1; dy++)
        //            {
        //                sorround_h = grid_information[toAddress(i+dx, j+dy)].height;
        //                if(sorround_h > 1e8) {break;}
        //                if( fabs(sorround_h - center_h) > 0.12 )
        //                {
        //                    grid_information[toAddress(i,j)].is_occupied = true;
        //                    grid_information[toAddress(i+dx, j+dy)].is_occupied = true;
        //                }
        //                
        //            }
        //            if(sorround_h > 1e8) {break;}
        //        }
//
//
        //    }
        //}

        ////
        //floor equals to GridInformation.is_occupied
        vector<signed char> a(floor, floor + grid_map.info.width * grid_map.info.height); //ori_occupied_map


        flateMap(0);
        fillESDF(10); 
        //fillStepESDF(5);
////////////////esdf
        
        esdf_var = new double[grid_map.info.width * grid_map.info.height];
        for(int i = 0 ; i < grid_map.info.width * grid_map.info.height ; i++)
        {
            esdf_var[i] = grid_information[i].esdf_var;
        }
        vector<signed char> b(esdf_var, esdf_var + grid_map.info.width * grid_map.info.height); // esdf_map

/////////////////step_esdf
        step_esdf_var = new double[grid_map.info.width * grid_map.info.height];
        for(int i = 0 ; i < grid_map.info.width * grid_map.info.height ; i++)
        {
            step_esdf_var[i] = grid_information[i].step_esdf_var;
        }
        vector<signed char> f(step_esdf_var, step_esdf_var + grid_map.info.width * grid_map.info.height); // esdf_map
        

//////////////// occupied
        double *occupied_map = new double[grid_map.info.width * grid_map.info.height];
        for(int i = 0 ; i < grid_map.info.width * grid_map.info.height ; i++)
        {
            occupied_map[i] = grid_information[i].is_occupied * 90;
        }
        vector<signed char> c(occupied_map, occupied_map + grid_map.info.width * grid_map.info.height); // occupied_map

//////////////// flate

        double *flate_map = new double[grid_map.info.width * grid_map.info.height];
        for(int i = 0 ; i < grid_map.info.width * grid_map.info.height ; i++)
        {
            flate_map[i] = grid_information[i].is_occupied_flate * 90;
        }
        vector<signed char> d(flate_map, flate_map + grid_map.info.width * grid_map.info.height); // esdf_map

////////////////////////
//////////////// height

        double *height_map = new double[grid_map.info.width * grid_map.info.height];
        for(int i = 0 ; i < grid_map.info.width * grid_map.info.height ; i++)
        {
            height_map[i] = grid_information[i].height * 60;
        }
        vector<signed char> e(height_map, height_map + grid_map.info.width * grid_map.info.height); // esdf_map

////////////////////////
        grid_map.data = c;
        grid_map_pub.publish(grid_map);
        publishESDFMap();

    }

    void LanGridMapManager::rcvOdomHandler(const nav_msgs::Odometry odom)
    {
        ugv_odom    = odom;
    }


    void LanGridMapManager::publishESDFMap()  //not only used for ESDF visualization
    {
        double esdf;
        Eigen::Vector3d posw ,g;
        Eigen::Vector3d gp,gv;
        double cp,cv;
        pcl::PointXYZI pt;
        pcl::PointCloud<pcl::PointXYZI> esdf_cloud;
        //jps.push_back(JumpPoint(Eigen::Vector2d(3,-17) , Eigen::Vector2d(0,-1)));
        for(double z = 0.0 ; z <= 0.0 ; z += 0.1)
        {
            for(double x = down_boundary  + 0.2; x <= up_boundary - 0.2; x += 0.08 * p_grid_resolution)
            {
                for(double y = right_boundary + 0.2; y <= left_boundary - 0.2; y += 0.08 * p_grid_resolution)
                {
                    posw(0) = x;
                    posw(1) = y;
                    posw(2) = z;
                    //esdf = getHeightByPosW3(posw);
                    esdf = calcESDF_Cost(posw);
                    //esdf = calcStepESDF_Cost(posw);
                    //esdf = is_step(point324(posw));
                    //esdf = is_occupied(point324(posw),0);
                    //getStepDirCostAndGrad(posw, g, gp,gv,esdf,cv);
                    pt.x = x;
                    pt.y = y;
                    pt.z = 0;
                    if( esdf < truncation_distance && esdf != 0)
                        pt.z = pow(truncation_distance - esdf, 3);
                    esdf_cloud.push_back(pt);
                }
            }
        }
        esdf_cloud.width = esdf_cloud.points.size();
        esdf_cloud.is_dense = true;
        esdf_cloud.header.frame_id = "world";
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(esdf_cloud, cloud_msg);
        esdf_vis_pub.publish(cloud_msg);

    }


    ///////////////////////////////////////////////////
    ///////////  查询
    ///////////////////////////////////////////////////
    bool LanGridMapManager::is_occupied(Eigen::Vector4d posw, int flate)
    {
        Eigen::Vector4d pos_m = posW2posM(posw);
        Eigen::Vector4d posm ;
        Eigen::Vector2i index;
        int index_x, index_y;
        for(int i = - flate ; i <= flate; i++)
        {
            for(int j = -flate; j <= flate; j++)
            {
                posm(0) = pos_m(0) + i * p_grid_resolution;
                posm(1) = pos_m(1) + j * p_grid_resolution;
                index = posM2Index(posm);
                index_x = index(0);
                index_y = index(1);
                if(posInMap(posm)) 
                {  
                    return grid_information[toAddress(index_x, index_y)].is_occupied_flate;
                }  
            }
        }

         
        return false;
    }

    bool LanGridMapManager::is_occupiedI(Eigen::Vector2i index, int flate)
    {
        for(int j = -flate; j <= flate; j++)
        {
            for(int i = -flate; i <= flate; i++)
            {
                if(grid_information[toAddress(index(0)+i , index(1)+j)].is_occupied ){return true;}
            }
        }
        return false;
    }

    bool LanGridMapManager::is_occupied_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2, int flate)
    {
        Eigen::Vector4d posw_inner;
        double distance = (posw1 - posw2).norm();
        int    segs     = 2 * (distance / p_grid_resolution);
        
        for(double t = 1.0/segs ; t < 1 ; t += 1.0/segs)
        {
            posw_inner = t * posw1 + (1-t) * posw2;
            if(is_occupied(posw_inner, flate)){
                return true;
            }
        }
        return false;
    }

    bool LanGridMapManager::is_not_occupied(Eigen::Vector4d posw, int flate)
    {
        Eigen::Vector4d pos_m = posW2posM(posw);
        Eigen::Vector4d posm ;
        Eigen::Vector2i index;
        int index_x, index_y;
        for(int i = - flate ; i <= flate; i++)
        {
            for(int j = -flate; j <= flate; j++)
            {
                posm(0) = pos_m(0) + i * p_grid_resolution;
                posm(1) = pos_m(1) + j * p_grid_resolution;
                index = posM2Index(posm);
                index_x = index(0);
                index_y = index(1);
                if(posInMap(posm)) 
                {  
                    return !(grid_information[toAddress(index_x, index_y)].is_occupied_flate);
                }  
            }
        }

         
        return false;
    }

    bool LanGridMapManager::is_not_occupiedI(Eigen::Vector2i index, int flate)
    {
        for(int j = -flate; j <= flate; j++)
        {
            for(int i = -flate; i <= flate; i++)
            {
                if(grid_information[toAddress(index(0)+i , index(1)+j)].is_occupied ){return false;}
            }
        }
        return true;
    }

    bool LanGridMapManager::is_not_occupied_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2, int flate)
    {
        Eigen::Vector4d posw_inner;
        double distance = (posw1 - posw2).norm();
        int    segs     = 2 * (distance / p_grid_resolution);
        
        for(double t = 1.0/segs ; t < 1 ; t += 1.0/segs)
        {
            posw_inner = t * posw1 + (1-t) * posw2;
            if(is_not_occupied(posw_inner, flate)){
                return true;
            }
        }
        return false;
    }

    bool LanGridMapManager::is_step(Eigen::Vector4d posw)
    {
        Eigen::Vector3d pos_w = point423(posw);
        double h = getHeightByPosW3(pos_w);
        if(h > 0.09)
        {
            return true;
        }
        return false;
    }

    bool LanGridMapManager::is_stepI(Eigen::Vector2i index)
    {
        double h = getHeightByI(index);
        if(h > 0.09)
        {
            return true;
        }
        return false;
    }
    bool LanGridMapManager::is_not_step_line(Eigen::Vector4d posw1, Eigen::Vector4d posw2)
    {
        Eigen::Vector4d posw_inner;
        double distance = (posw1 - posw2).norm();
        int    segs     = 2 * (distance / p_grid_resolution);
        for(double t = 1.0/segs ; t < 1 ; t += 1.0/segs)
        {
            posw_inner = t * posw1 + (1-t) * posw2;
            if(!is_step(posw_inner)){
                return true;
            }
        }
        return false;
    }


    double LanGridMapManager::getGapByI(Eigen::Vector2i index)
    {
     
        double gap = grid_information[toAddress(index(0), index(1))].gap;
        gap = min(gap, max_height); 
        gap = max(min_height, gap);
        return gap;
    }

    double LanGridMapManager::getHeightByI(Eigen::Vector2i index)
    {
        double height = grid_information[toAddress(index(0), index(1))].height;
        return height;
    }

    
    ///////////////////////////////////////////////////
    /////////// 坐标变换
    ///////////////////////////////////////////////////
    

    bool LanGridMapManager::indexInMap(int index_x, int index_y)
    {
        if(index_x >= 0 && index_y >= 0 && index_x < map_index_xmax && index_y < map_index_ymax) 
        {
            return true; 
        }
        return false;
    }
    bool LanGridMapManager::posInMap(Eigen::Vector4d pos)
    {
        int index_x, index_y;
        index_x = pos(0) / p_grid_resolution;
        index_y = pos(1) / p_grid_resolution;
        return indexInMap(index_x, index_y);
    }

    bool LanGridMapManager::posWInMap(Eigen::Vector4d pos)
    {
        return posInMap(posW2posM(pos));
    }


    Eigen::Vector2i LanGridMapManager::posM2Index(Eigen::Vector4d pos)
    {
        Eigen::Vector2i index;
        index(0) = pos(0) / p_grid_resolution;
        index(1) = pos(1) / p_grid_resolution;
        return index;
    }

    Eigen::Vector2i LanGridMapManager::posW2Index(Eigen::Vector4d pos)
    {
        Eigen::Vector4d posm = posW2posM(pos);
        return posM2Index(posm);
    }

    Eigen::Vector4d LanGridMapManager::index2PosW(Eigen::Vector2i index)
    {
        Eigen::Vector4d posw , posm;
        posm(0) = index(0) * p_grid_resolution + p_grid_resolution/2;
        posm(1) = index(1) * p_grid_resolution + p_grid_resolution/2;
        posm(2) = 0;
        posm(3) = 1;

        posw    = posM2posW(posm);
        return posw;
    }

    ///////////////////////////////////////////////////
    ///////////  Init
    ///////////////////////////////////////////////////
    void LanGridMapManager::init(ros::NodeHandle& node_handler)
    {
        nh                   = node_handler;
        //p_grid_resolution    = 0.1;
        global_map_sub        = nh.subscribe("global_map",1, &LanGridMapManager::rcvGlobalMapHandler, this);
        odometry_sub         = nh.subscribe("odom", 1 , &LanGridMapManager::rcvOdomHandler, this);
        grid_map_pub         = nh.advertise<nav_msgs::OccupancyGrid>("grid_map",1);
        esdf_vis_pub         = nh.advertise<sensor_msgs::PointCloud2>("/esdf_vis", 10);

    }


    ///////////////////////////////////////////////////
    ///////////  ESDF
    ///////////////////////////////////////////////////


    bool LanGridMapManager::isBoundaryStep(int index_x , int index_y)
    {
        bool boundary = false;
        int n_indx, n_indy;
        for(int i = -1; i <= 1; i++)
        {
            for(int j = -1; j <= 1; j++)
            {
                n_indx = index_x + i;
                n_indy = index_y + j;
                if(!indexInMap(n_indx,n_indy))
                {
                    return false;
                }
                if( is_stepI(Eigen::Vector2i(n_indx, n_indy)) == false)
                {
                    boundary = true;
                    break;
                }
            }
            if(boundary){break;}
        }
        return boundary;
    }

    bool LanGridMapManager::isBoundaryOcc(int index_x , int index_y)
    {
        bool boundary = false;
        int n_indx, n_indy;
        for(int i = -1; i <= 1; i++)
        {
            for(int j = -1; j <= 1; j++)
            {
                n_indx = index_x + i;
                n_indy = index_y + j;
                if(!indexInMap(n_indx,n_indy))
                {
                    return false;
                }
                if(grid_information[toAddress(n_indx ,n_indy)].is_occupied_flate == false)
                {
                    boundary = true;
                    break;
                }
            }
            if(boundary){break;}
        }
        return boundary;
    }


    void LanGridMapManager::fillStepESDF(int t_distance)
    {
        int addr;
        double var;
        Eigen::Vector2i index;
        for(int index_x = 0 ; index_x < map_index_xmax; index_x ++)
        {
            for(int index_y = 0 ; index_y < map_index_ymax; index_y ++)
            {
                if( is_stepI(Eigen::Vector2i(index_x, index_y))  == false ) {continue;}
                else if(!isBoundaryStep(index_x,index_y)){continue;}

                for(int dx = -t_distance ; dx <= t_distance; dx++)
                {
                    for(int dy = -t_distance ; dy <= t_distance; dy++)
                    {  
                        index(0) = index_x + dx;
                        index(1) = index_y + dy;
                        addr = toAddress(index(0) ,index(1));
                        if( !indexInMap(index)) {continue;}

                        var  = Eigen::Vector2d(dx, dy).norm() * p_grid_resolution ;   
                        grid_information[addr].step_esdf_var = min( grid_information[addr].step_esdf_var , var );

                    }
                }
            }
        }

        for(int index_x = 0 ; index_x < map_index_xmax; index_x ++)
        {
            for(int index_y = 0 ; index_y < map_index_ymax; index_y ++)
            {
                addr = toAddress(index_x ,index_y);
                if( is_stepI(Eigen::Vector2i(index_x, index_y))  == false ) {continue;}
                grid_information[addr].step_esdf_var *= -1;
            }
        }
    }

    void LanGridMapManager::fillESDF(int t_distance)
    {
        int addr;
        double var;
        Eigen::Vector2i index;
        for(int index_x = 0 ; index_x < map_index_xmax; index_x ++)
        {
            for(int index_y = 0 ; index_y < map_index_ymax; index_y ++)
            {
                addr = toAddress(index_x ,index_y);
                if( grid_information[addr].is_occupied_flate  == false ) {continue;}
                else if(!isBoundaryOcc(index_x,index_y)){continue;}

                for(int dx = -t_distance ; dx <= t_distance; dx++)
                {
                    for(int dy = -t_distance ; dy <= t_distance; dy++)
                    {  
                        index(0) = index_x + dx;
                        index(1) = index_y + dy;
                        addr = toAddress(index(0) ,index(1));
                        if( !indexInMap(index)) {continue;}

                        var  = Eigen::Vector2d(dx, dy).norm() * p_grid_resolution ;   
                        grid_information[addr].esdf_var = min( grid_information[addr].esdf_var , var );

                    }
                }
            }
        }

        for(int index_x = 0 ; index_x < map_index_xmax; index_x ++)
        {
            for(int index_y = 0 ; index_y < map_index_ymax; index_y ++)
            {
                addr = toAddress(index_x ,index_y);
                if( grid_information[addr].is_occupied_flate  == false ) {continue;}
                grid_information[addr].esdf_var *= -1;
            }
        }
    }


    double LanGridMapManager::calcESDF_Cost(const Eigen::Vector3d& pos_w)
    {
        double posx, posy;        //posm
        double index_x , index_y;
        int lines_index_x1 , lines_index_x2;
        int lines_index_y1 , lines_index_y2;
        double var_lu, var_ru;
        double var_ld, var_rd;
        double var_u, var_d;
        double esdf_cost = 0.0;
        int addr_lu, addr_ld, addr_ru, addr_rd;

        Eigen::Vector4d posw, posm;
        posw(0) = pos_w(0);
        posw(1) = pos_w(1);
        posw(2) = 0;
        posw(3) = 1;
        posm = posW2posM(posw);

        if( !posWInMap(posw) )
        {
            ROS_WARN("[calcESDF] point out of map!");
            return 0;
        }

        //Eigen::Vector2i index = posW2Index(posw);
        //return esdf_var[toAddress(index(0), index(1))];

        posx = posm(0);
        posy = posm(1);

        index_x = posx /  p_grid_resolution;
        if(index_x - floor(index_x) <= 0.5)
        {
            lines_index_x1 = floor(index_x);
            lines_index_x2 = floor(index_x) - 1;
        }
        else
        {
            lines_index_x1 = ceil(index_x);
            lines_index_x2 = floor(index_x);
        }

        index_y = posy / p_grid_resolution;
        if(index_y - floor(index_y) <= 0.5)
        {
            lines_index_y1 = floor(index_y);
            lines_index_y2 = floor(index_y) - 1;
        }
        else
        {
            lines_index_y1 = ceil(index_y);
            lines_index_y2 = floor(index_y);
        }



        ////////////////////////
        addr_lu = toAddress(lines_index_x1 , lines_index_y1);
        addr_ru = toAddress(lines_index_x2 , lines_index_y1);
        addr_ld = toAddress(lines_index_x1 , lines_index_y2);
        addr_rd = toAddress(lines_index_x2 , lines_index_y2);
        if( !isAddressOutOfMap(addr_lu) &&
            !isAddressOutOfMap(addr_ru) &&
            !isAddressOutOfMap(addr_ld) &&
            !isAddressOutOfMap(addr_rd)  )
        {
            var_lu = esdf_var[addr_lu];
            var_ru = esdf_var[addr_ru];
            var_ld = esdf_var[addr_ld];
            var_rd = esdf_var[addr_rd];

            var_d  = var_rd + (var_ld - var_rd) * (index_x - (lines_index_x2 + 0.5));
            var_u  = var_ru + (var_lu - var_ru) * (index_x - (lines_index_x2 + 0.5));

            if(fabs(var_lu) > 1e8 || fabs(var_ru) > 1e8 ||fabs(var_ld) > 1e8 ||fabs(var_rd) > 1e8 ){return 0;}
            esdf_cost       = var_d + (var_u - var_d) * (index_y - (lines_index_y2 + 0.5));
        }

        return esdf_cost;

    }
    Eigen::Vector3d LanGridMapManager::calcESDF_Grid(const Eigen::Vector3d& pos_w)
    {
        double posx, posy;        //posm
        double index_x , index_y;

        int lines_index_x1 , lines_index_x2;
        int lines_index_y1 , lines_index_y2;

        double var_lu, var_ru;
        double var_ld, var_rd;
        double g_u, g_d;
        double g_l, g_r;
        double gx,gy,gz;
        int addr_lu, addr_ld, addr_ru, addr_rd;
        Eigen::Vector4d posw, posm;
        posw(0) = pos_w(0);
        posw(1) = pos_w(1);
        posw(2) = 0;
        posw(3) = 1;
        posm = posW2posM(posw);
        posx = posm(0);
        posy = posm(1);

        index_x = posx /  p_grid_resolution;
        if(index_x - floor(index_x) <= 0.5)
        {
            lines_index_x1 = floor(index_x);
            lines_index_x2 = floor(index_x) - 1;
        }
        else
        {
            lines_index_x1 = ceil(index_x);
            lines_index_x2 = floor(index_x);
        }

        index_y = posy / p_grid_resolution;
        if(index_y - floor(index_y) <= 0.5)
        {
            lines_index_y1 = floor(index_y);
            lines_index_y2 = floor(index_y) - 1;
        }
        else
        {
            lines_index_y1 = ceil(index_y);
            lines_index_y2 = floor(index_y);
        }
   
        ////////////////////////
        addr_lu = toAddress(lines_index_x1 , lines_index_y1);
        addr_ru = toAddress(lines_index_x2 , lines_index_y1);
        addr_ld = toAddress(lines_index_x1 , lines_index_y2);
        addr_rd = toAddress(lines_index_x2 , lines_index_y2);
        if( !isAddressOutOfMap(addr_lu) &&
            !isAddressOutOfMap(addr_ru) &&
            !isAddressOutOfMap(addr_ld) &&
            !isAddressOutOfMap(addr_rd)  )
        {
            var_lu = esdf_var[addr_lu];
            var_ru = esdf_var[addr_ru];
            var_ld = esdf_var[addr_ld];
            var_rd = esdf_var[addr_rd];
            if(fabs(var_lu) > 1e8 || fabs(var_ru) > 1e8 ||fabs(var_ld) > 1e8 ||fabs(var_rd) > 1e8 ){return Eigen::Vector3d::Zero();}

            g_u = (var_lu - var_ru) / p_grid_resolution;
            g_d = (var_ld - var_rd) / p_grid_resolution;
            g_l = (var_lu - var_ld) / p_grid_resolution;
            g_r = (var_ru - var_rd) / p_grid_resolution;
            //gx
            gx =  -(  g_r + (g_l - g_r) * (index_x - (lines_index_x2 + 0.5))  );

            //gy
            gy =  -(  g_d + (g_u - g_d) * (index_y - (lines_index_y2 + 0.5))  );
        }
        else
        {
            gx = gy = 0;
        }
        gz = 0;
        Eigen::Vector3d g;
        g(0) = gx;
        g(1) = gy;
        g(2) = gz;
        return g;
    }

    double LanGridMapManager::calcStepESDF_Cost(const Eigen::Vector3d& pos_w)
    {
        double posx, posy;        //posm
        double index_x , index_y;
        int lines_index_x1 , lines_index_x2;
        int lines_index_y1 , lines_index_y2;
        double var_lu, var_ru;
        double var_ld, var_rd;
        double var_u, var_d;
        double esdf_cost = 0.0;
        int addr_lu, addr_ld, addr_ru, addr_rd;

        Eigen::Vector4d posw, posm;
        posw(0) = pos_w(0);
        posw(1) = pos_w(1);
        posw(2) = 0;
        posw(3) = 1;
        posm = posW2posM(posw);

        if( !posWInMap(posw) )
        {
            ROS_WARN("[calcESDF] point out of map!");
            return 0;
        }

        //Eigen::Vector2i index = posW2Index(posw);
        //return esdf_var[toAddress(index(0), index(1))];

        posx = posm(0);
        posy = posm(1);

        index_x = posx /  p_grid_resolution;
        if(index_x - floor(index_x) <= 0.5)
        {
            lines_index_x1 = floor(index_x);
            lines_index_x2 = floor(index_x) - 1;
        }
        else
        {
            lines_index_x1 = ceil(index_x);
            lines_index_x2 = floor(index_x);
        }

        index_y = posy / p_grid_resolution;
        if(index_y - floor(index_y) <= 0.5)
        {
            lines_index_y1 = floor(index_y);
            lines_index_y2 = floor(index_y) - 1;
        }
        else
        {
            lines_index_y1 = ceil(index_y);
            lines_index_y2 = floor(index_y);
        }



        ////////////////////////
        addr_lu = toAddress(lines_index_x1 , lines_index_y1);
        addr_ru = toAddress(lines_index_x2 , lines_index_y1);
        addr_ld = toAddress(lines_index_x1 , lines_index_y2);
        addr_rd = toAddress(lines_index_x2 , lines_index_y2);
        if( !isAddressOutOfMap(addr_lu) &&
            !isAddressOutOfMap(addr_ru) &&
            !isAddressOutOfMap(addr_ld) &&
            !isAddressOutOfMap(addr_rd)  )
        {
            var_lu = step_esdf_var[addr_lu];
            var_ru = step_esdf_var[addr_ru];
            var_ld = step_esdf_var[addr_ld];
            var_rd = step_esdf_var[addr_rd];

            var_d  = var_rd + (var_ld - var_rd) * (index_x - (lines_index_x2 + 0.5));
            var_u  = var_ru + (var_lu - var_ru) * (index_x - (lines_index_x2 + 0.5));

            if(fabs(var_lu) > 1e8 || fabs(var_ru) > 1e8 ||fabs(var_ld) > 1e8 ||fabs(var_rd) > 1e8 ){return 0;}
            esdf_cost       = var_d + (var_u - var_d) * (index_y - (lines_index_y2 + 0.5));
        }

        return esdf_cost;

    }
    Eigen::Vector3d LanGridMapManager::calcStepESDF_Grid(const Eigen::Vector3d& pos_w)
    {
        double posx, posy;        //posm
        double index_x , index_y;

        int lines_index_x1 , lines_index_x2;
        int lines_index_y1 , lines_index_y2;

        double var_lu, var_ru;
        double var_ld, var_rd;
        double g_u, g_d;
        double g_l, g_r;
        double gx,gy,gz;
        int addr_lu, addr_ld, addr_ru, addr_rd;
        Eigen::Vector4d posw, posm;
        posw(0) = pos_w(0);
        posw(1) = pos_w(1);
        posw(2) = 0;
        posw(3) = 1;
        posm = posW2posM(posw);
        posx = posm(0);
        posy = posm(1);

        index_x = posx /  p_grid_resolution;
        if(index_x - floor(index_x) <= 0.5)
        {
            lines_index_x1 = floor(index_x);
            lines_index_x2 = floor(index_x) - 1;
        }
        else
        {
            lines_index_x1 = ceil(index_x);
            lines_index_x2 = floor(index_x);
        }

        index_y = posy / p_grid_resolution;
        if(index_y - floor(index_y) <= 0.5)
        {
            lines_index_y1 = floor(index_y);
            lines_index_y2 = floor(index_y) - 1;
        }
        else
        {
            lines_index_y1 = ceil(index_y);
            lines_index_y2 = floor(index_y);
        }
   
        ////////////////////////
        addr_lu = toAddress(lines_index_x1 , lines_index_y1);
        addr_ru = toAddress(lines_index_x2 , lines_index_y1);
        addr_ld = toAddress(lines_index_x1 , lines_index_y2);
        addr_rd = toAddress(lines_index_x2 , lines_index_y2);
        if( !isAddressOutOfMap(addr_lu) &&
            !isAddressOutOfMap(addr_ru) &&
            !isAddressOutOfMap(addr_ld) &&
            !isAddressOutOfMap(addr_rd)  )
        {
            var_lu = step_esdf_var[addr_lu];
            var_ru = step_esdf_var[addr_ru];
            var_ld = step_esdf_var[addr_ld];
            var_rd = step_esdf_var[addr_rd];
            if(fabs(var_lu) > 1e8 || fabs(var_ru) > 1e8 ||fabs(var_ld) > 1e8 ||fabs(var_rd) > 1e8 ){return Eigen::Vector3d::Zero();}

            g_u = (var_lu - var_ru) / p_grid_resolution;
            g_d = (var_ld - var_rd) / p_grid_resolution;
            g_l = (var_lu - var_ld) / p_grid_resolution;
            g_r = (var_ru - var_rd) / p_grid_resolution;
            //gx
            gx =  -(  g_r + (g_l - g_r) * (index_x - (lines_index_x2 + 0.5))  );

            //gy
            gy =  -(  g_d + (g_u - g_d) * (index_y - (lines_index_y2 + 0.5))  );
        }
        else
        {
            gx = gy = 0;
        }
        gz = 0;
        Eigen::Vector3d g;
        g(0) = gx;
        g(1) = gy;
        g(2) = gz;
        return g;
    }


    double LanGridMapManager::getBlinearGap(const Eigen::Vector3d& pos_w)
    {
        double posx, posy;        //posm
        double index_x , index_y;
        int lines_index_x1 , lines_index_x2;
        int lines_index_y1 , lines_index_y2;
        double var_lu, var_ru;
        double var_ld, var_rd;
        double var_u, var_d;
        double gap_value;
        int addr_lu, addr_ld, addr_ru, addr_rd;

        Eigen::Vector4d posw, posm;
        posw(0) = pos_w(0);
        posw(1) = pos_w(1);
        posw(2) = 0;
        posw(3) = 1;
        posm = posW2posM(posw);

        if( !posWInMap(posw) )
        {
            ROS_WARN("[calcESDF] point out of map!");
            return 0;
        }

        posx = posm(0);
        posy = posm(1);

        index_x = posx /  p_grid_resolution;
        if(index_x - floor(index_x) <= 0.5)
        {
            lines_index_x1 = floor(index_x);
            lines_index_x2 = floor(index_x) - 1;
        }
        else
        {
            lines_index_x1 = ceil(index_x);
            lines_index_x2 = floor(index_x);
        }

        index_y = posy / p_grid_resolution;
        if(index_y - floor(index_y) <= 0.5)
        {
            lines_index_y1 = floor(index_y);
            lines_index_y2 = floor(index_y) - 1;
        }
        else
        {
            lines_index_y1 = ceil(index_y);
            lines_index_y2 = floor(index_y);
        }

        ////////////////////////
        addr_lu = toAddress(lines_index_x1 , lines_index_y1);
        addr_ru = toAddress(lines_index_x2 , lines_index_y1);
        addr_ld = toAddress(lines_index_x1 , lines_index_y2);
        addr_rd = toAddress(lines_index_x2 , lines_index_y2);
        if( !isAddressOutOfMap(addr_lu) &&
            !isAddressOutOfMap(addr_ru) &&
            !isAddressOutOfMap(addr_ld) &&
            !isAddressOutOfMap(addr_rd)  )
        {
            var_lu = getGapByI( Eigen::Vector2i(lines_index_x1 , lines_index_y1));
            var_ru = getGapByI( Eigen::Vector2i(lines_index_x2 , lines_index_y1));
            var_ld = getGapByI( Eigen::Vector2i(lines_index_x1 , lines_index_y2));
            var_rd = getGapByI( Eigen::Vector2i(lines_index_x2 , lines_index_y2));

            var_d  = var_rd + (var_ld - var_rd) * (index_x - (lines_index_x2 + 0.5));
            var_u  = var_ru + (var_lu - var_ru) * (index_x - (lines_index_x2 + 0.5));

            gap_value       = var_d + (var_u - var_d) * (index_y - (lines_index_y2 + 0.5));
        }

        return gap_value;
    }


    Eigen::Vector3d LanGridMapManager::getBlinearGapGrad(const Eigen::Vector3d& pos_w)
    {
        double posx, posy;        //posm
        double index_x , index_y;

        int lines_index_x1 , lines_index_x2;
        int lines_index_y1 , lines_index_y2;

        double var_lu, var_ru;
        double var_ld, var_rd;
        double g_u, g_d;
        double g_l, g_r;
        Eigen::Vector3d ret;
        int addr_lu, addr_ld, addr_ru, addr_rd;
        Eigen::Vector4d posw, posm;
        posw(0) = pos_w(0);
        posw(1) = pos_w(1);
        posw(2) = 0;
        posw(3) = 1;
        posm = posW2posM(posw);
        posx = posm(0);
        posy = posm(1);

        index_x = posx /  p_grid_resolution;
        if(index_x - floor(index_x) <= 0.5)
        {
            lines_index_x1 = floor(index_x);
            lines_index_x2 = floor(index_x) - 1;
        }
        else
        {
            lines_index_x1 = ceil(index_x);
            lines_index_x2 = floor(index_x);
        }

        index_y = posy / p_grid_resolution;
        if(index_y - floor(index_y) <= 0.5)
        {
            lines_index_y1 = floor(index_y);
            lines_index_y2 = floor(index_y) - 1;
        }
        else
        {
            lines_index_y1 = ceil(index_y);
            lines_index_y2 = floor(index_y);
        }
   
        ////////////////////////
        addr_lu = toAddress(lines_index_x1 , lines_index_y1);
        addr_ru = toAddress(lines_index_x2 , lines_index_y1);
        addr_ld = toAddress(lines_index_x1 , lines_index_y2);
        addr_rd = toAddress(lines_index_x2 , lines_index_y2);
        if( !isAddressOutOfMap(addr_lu) &&
            !isAddressOutOfMap(addr_ru) &&
            !isAddressOutOfMap(addr_ld) &&
            !isAddressOutOfMap(addr_rd)  )
        {
            var_lu = getGapByI( Eigen::Vector2i(lines_index_x1 , lines_index_y1));
            var_ru = getGapByI( Eigen::Vector2i(lines_index_x2 , lines_index_y1));
            var_ld = getGapByI( Eigen::Vector2i(lines_index_x1 , lines_index_y2));
            var_rd = getGapByI( Eigen::Vector2i(lines_index_x2 , lines_index_y2));

            g_u = (var_lu - var_ru) / p_grid_resolution;
            g_d = (var_ld - var_rd) / p_grid_resolution;
            g_l = (var_lu - var_ld) / p_grid_resolution;
            g_r = (var_ru - var_rd) / p_grid_resolution;
            //gx
            ret(0) =  (  g_r + (g_l - g_r) * (index_x - (lines_index_x2 + 0.5))  );

            //gy
            ret(1) =  (  g_d + (g_u - g_d) * (index_y - (lines_index_y2 + 0.5))  );
        }
        else
        {
            return ret;
        }
        ret(2) = 0;
        return ret;
    }



    void LanGridMapManager::calcZ_CostGrad(const Eigen::Vector3d& pos_w , double &z_cost , Eigen::Vector3d& z_grad)
    {

        double gap_value = getBlinearGap(pos_w);
        Eigen::Vector3d gvgrad;
        gvgrad = getBlinearGapGrad(pos_w);
        // gvgrad means " d gap_value(x,y) "

        double a = ( sqrt(max_height - min_height) + min_height ) / max_height;
        double minco_h  = pow( a * gap_value - min_height , 2) + min_height; 
        double z = pos_w(2);

        double f_minco_h =  pow( minco_h - max_height ,2);
        double h_minco_h =  pow( z - minco_h, 2 );
        z_cost = f_minco_h * h_minco_h;
        z_grad = Eigen::Vector3d(0,0,0);
        
        double z_gradx_f, z_gradx_h, z_grady_f, z_grady_h;
        z_gradx_h = 2 * (z - minco_h) * (-2) * a * (a * gap_value - min_height) * gvgrad(0);
        z_grady_h = 2 * (z - minco_h) * (-2) * a * (a * gap_value - min_height) * gvgrad(1);
        z_gradx_f = 2 * ( minco_h - max_height ) * (2) * a * (a * gap_value - min_height) * gvgrad(0);
        z_grady_f = 2 * ( minco_h - max_height ) * (2) * a * (a * gap_value - min_height) * gvgrad(1);

        z_grad(0) = z_gradx_f * h_minco_h + z_gradx_h * f_minco_h;
        z_grad(1) = z_grady_f * h_minco_h + z_grady_h * f_minco_h;
        z_grad(2) = 2 * (z - minco_h) * f_minco_h ;
    }


    bool LanGridMapManager::getStepDirCostAndGrad(const Eigen::Vector3d p, const Eigen::Vector3d v,
                                        Eigen::Vector3d& gradp,
                                        Eigen::Vector3d& gradv,
                                        double& costp, 
                                        double& costv)
    {
        Eigen::Vector2d focus1, focus2;
        Eigen::Vector2d p_err1, p_err2;
        Eigen::Vector2d v_dir       ;
        Eigen::Vector2d focus_dir1  ;
        Eigen::Vector2d focus_dir2  ;
        Eigen::Vector2d p_xy   = Eigen::Vector2d( p(0) , p(1) );
        Eigen::Vector2d v_xy   = Eigen::Vector2d( v(0) , v(1) );
        double ellipse_a = 1.0;
        double ellipse_b = 0.6;
        double ellipse_c = sqrt( ellipse_a * ellipse_a - ellipse_b * ellipse_b );
        double slit_width = 1.25;
        double pdis1 = 0, pdis2 = 0;
        double dpen  = 0;
        double cost_p = 0 , cost_v = 0;
        double grad_px = 0 , grad_vx = 0;
        double grad_py = 0 , grad_vy = 0;

        double vdis = 0;
        bool ret = false;
        if( jps.size() == 0)
        {
            return false;
        }
        for(JumpPoint jp : jps)
        {
            v_dir       = jp.velocity_dir;
            v_dir.normalize();
            v_dir *= 2 * ellipse_c;
            focus_dir1  = Eigen::Vector2d( -v_dir(1),  v_dir(0) );
            focus_dir2  = Eigen::Vector2d( v_dir(1) , -v_dir(0) );
            focus_dir1.normalize();
            focus_dir2.normalize();

            focus1  = jp.takeoff_pos + focus_dir1 * slit_width/2 - v_dir/2;
            focus2  = focus1 + v_dir;
            p_err1  = (p_xy - focus1);
            p_err2  = (p_xy - focus2);
            pdis1   = p_err1.norm();
            pdis2   = p_err2.norm();
            dpen    = 2 * ellipse_a - pdis1 - pdis2;
            if(dpen > 0)
            {
                ret = true;
                cost_p += dpen;
                grad_px += - ( (p(0) - focus1(0))/pdis1 + (p(0) - focus2(0))/pdis2 );
                grad_py += - ( (p(1) - focus1(1))/pdis1 + (p(1) - focus2(1))/pdis2 );
            }

            focus1  = jp.takeoff_pos + focus_dir2 * slit_width/2 - v_dir/2;
            focus2  = focus1 + v_dir;
            p_err1  = (p_xy - focus1);
            p_err2  = (p_xy - focus2);
            pdis1   = p_err1.norm();
            pdis2   = p_err2.norm();
            dpen    = 2 * ellipse_a - pdis1 - pdis2;
            if(dpen > 0)
            {
                ret = true;
                cost_p += dpen;
                grad_px += - ( (p(0) - focus1(0))/pdis1 + (p(0) - focus2(0))/pdis2 );
                grad_py += - ( (p(1) - focus1(1))/pdis1 + (p(1) - focus2(1))/pdis2 );
            }

            ///////////////////////////////

            vdis   = (v_dir - v_xy).squaredNorm();
            cost_v += vdis;
            grad_vx += -2 * (v_dir(0) - v(0));
            grad_vy += -2 * (v_dir(1) - v(1));
 
        }
        if(ret == false)
        {
            return false;
        }
        
        //cout<<"cp, cv = "<< -w_dir <<"  |  "<<cost_v<<endl;

        //double grad_vx =  ((v(0) * des_dir(0))/v_xy.norm() - 1) * ( -(des_v(0) - v(0)) / err ) / pow(err_plus_one , 2);
        //double grad_vy =  ((v(1) * des_dir(1))/v_xy.norm() - 1) * ( -(des_v(1) - v(1)) / err ) / pow(err_plus_one , 2);
        //double grad_vx = -((des_v(0) - v(0))*( (des_dir(0)*v(0))/v_xy.norm()  - 1) / err);
        //double grad_vy = -((des_v(1) - v(1))*( (des_dir(1)*v(1))/v_xy.norm()  - 1) / err);
        //double grad_vx = -2 * (des_v(0) - v(0));
        //double grad_vy = -2 * (des_v(1) - v(1));


        costp = cost_p;
        //costp = 1;
        costv = cost_v;
        //costv = 1;
        gradp = Eigen::Vector3d(grad_px, grad_py , 0);
        //gradp = Eigen::Vector3d(0, 0 , 0);
        gradv = Eigen::Vector3d(grad_vx, grad_vy , 0);
        //gradv = Eigen::Vector3d(0, 0 , 0);

        return true;

    }


    void LanGridMapManager::getGradCostP(const Eigen::Vector3d p ,double& cost, Eigen::Vector3d& grad)
    {
        double c = 0;
        double dist = 0;
        int addr;
        Eigen::Vector3d g = Eigen::Vector3d::Zero() , dg;
        Eigen::Vector3d pc;

        Eigen::Vector2i ori_index = posW2Index(point324(p));
        for(int index_x = 0 ; index_x < map_index_xmax; index_x ++)
        {
            for(int index_y = 0 ; index_y < map_index_ymax; index_y ++)
            {
                addr = toAddress(index_x ,index_y);
                if( grid_information[addr].is_occupied  == false ) {continue;}

                pc = point423(index2PosW(Eigen::Vector2i(index_x, index_y)));
                dist = ( pc - p ).norm();

                if(dist > 0.4)
                    continue;
                c   += 1 / pow(dist,2);
                dg  = (pc - p);
                dg.normalize();
                g   -= dg * ( 2*(1/ pow(dist,3)) );

            }
        }
        cost = c;
        grad = g;
    }

    ///////////////////////////////////////////////////
    ///////////    FLATE
    ///////////////////////////////////////////////////
    void LanGridMapManager::flateMap(int t_distance)
    {
        int addr;
        double var;
        Eigen::Vector2i index;
        for(int index_x = 0 ; index_x < map_index_xmax; index_x ++)
        {
            for(int index_y = 0 ; index_y < map_index_ymax; index_y ++)
            {
                addr = toAddress(index_x ,index_y);
                if( grid_information[addr].is_occupied  == false ) {continue;}

                for(int dx = -t_distance ; dx <= t_distance; dx++)
                {
                    for(int dy = -t_distance ; dy <= t_distance; dy++)
                    {  
                        index(0) = index_x + dx;
                        index(1) = index_y + dy;
                        addr = toAddress(index(0) ,index(1));
                        if( !indexInMap(index)) {continue;}
                        if( grid_information[addr].is_occupied_flate == true ) {continue;} 

                        grid_information[addr].is_occupied_flate = true;
                    }
                }
            }
        }
    }


    ///////////////////////////////////////////////////
    ///////////  A* Path search
    ///////////////////////////////////////////////////

    bool isInSet(vector<GridNodePtr> set, int set_id, Eigen::Vector2i pos_index)
    {
        for(int i = 0 ; i < set.size(); i++)
        {
            if(set[i] -> index == pos_index)
            {
                if(set[i] -> id == set_id)
                    return true;
                else
                    return false;
            }
        }
        return false;
    }

    vector<Eigen::Vector3d> LanGridMapManager::AstarPathSearch(Eigen::Vector3d start, Eigen::Vector3d end)
    {
        
        vector<Eigen::Vector3d> path_0;

        if(!posWInMap(point324(start)) || !posWInMap(point324(end)))
        {
            ROS_ERROR("[Astar] boundary points out of map.");
            return path_0;
        }

        int open_set_size = 0 , close_set_size = 0;
        vector<GridNodePtr> set;

        set.push_back( new GridNode(start, 0.0, posW2Index(point324(start)) , NULL , 1) );
        open_set_size ++;

        Eigen::Vector2i end_index = posW2Index(point324(end));

        Eigen::Vector2i ite_index;
        Eigen::Vector3d ite_coord;

        Eigen::Vector2i neighbor_index;
        int neighbor_indx, neighbor_indy;
        double ite_cost, neighbor_cost, heu, stepeu;

        int iter = 0;

        while(open_set_size > 0)
        {

            iter ++;
            
            ////find minco gridnode
            double minco = 100000;
            int minco_index = 0;
            for(int i = 0 ; i < set.size(); i++)
            {
                if(set[i] -> id != 1) 
                    continue;
                if(set[i] -> cost < minco){
                    minco = set[i] -> cost;
                    minco_index = i;
                }
            }

            GridNodePtr ite_par = set[minco_index];
            ite_par -> id = -1;
            open_set_size  --;
            close_set_size ++;
            //////////////////////////////


            ite_index = ite_par->index;
            ite_coord = ite_par->coord;
            ite_cost  = ite_par->cost;

            if(ite_index == end_index)
            {
                cout<<"[AStar INFO] path found! " << endl;
                break;
            }

            for(int i = -1; i <= 1; i++)
            {
                for(int j = -1; j <= 1; j++)
                {
                    if(i == 0 && j == 0){continue;}
                    neighbor_index(0) = neighbor_indx = ite_index(0) + i;
                    neighbor_index(1) = neighbor_indy = ite_index(1) + j;

                    if( indexInMap(neighbor_indx, neighbor_indy) && 
                       !isInSet(set, 1 , neighbor_index) && 
                       !isInSet(set, -1, neighbor_index) && 
                       !is_occupiedI(neighbor_index,3) )
                    {
                        heu = (end_index - neighbor_index).norm();
                        stepeu = abs(getHeightByI(ite_index) - getHeightByI(neighbor_index));

                        neighbor_cost = ( (i * j == 0) ? p_grid_resolution : (p_grid_resolution * 1.41)) + ite_cost
                                        + heu * p_grid_resolution; 
                                        //+ stepeu * 100;
                        set.push_back(new GridNode( point423(index2PosW(neighbor_index)),  neighbor_cost,  neighbor_index, (ite_par) ,1) );
                        open_set_size ++;

                    }

                }
            }         
        }

        if(open_set_size == 0)
        {
            ROS_ERROR("[AStar ERR] path not found!");
            set.clear();
            return path_0;
        }
        

        GridNodePtr ite;
        for(int i = set.size() - 1; i >= 0 ; i--)
        {
            if(set[i] -> id == -1)
            {
                ite = set[i];
                break;
            }
        }

        for( ; ite->father != NULL ; ite = ite->father )
        {
            
            path_0.push_back(ite->coord);
        }
        
        set.clear();
        return path_0;
    }


 

    /////////////////////////////////////////////////////
    //////////// convexClusterInflation
    //////////// input : seed pixel , map
    /////////////////////////////////////////////////////
    bool isInSet(vector<Eigen::Vector3d> set, Eigen::Vector3d pos)
    {
        for(int i = 0 ; i < set.size(); i++)
        {
            if(set[i] == pos)
            {
                return true;
            }
        }
        return false;
    }

    vector<Eigen::Vector3d> LanGridMapManager::getNeighbors( vector<Eigen::Vector3d> set , vector<Eigen::Vector3d> C_plus, vector<Eigen::Vector3d> C)
    {
        vector<Eigen::Vector3d> neighbors;
        Eigen::Vector2i iter_index;
        Eigen::Vector2i neighbor_index;
        for(int i = 0 ; i < set.size(); i++)
        {
            iter_index = posW2Index(point324(set[i]));
            for(int px = -1 ; px <= 1 ; px++)
            {
                for(int py = -1; py <=1 ; py++)
                {
                    if(px == 0 && py == 0){continue;}
                    neighbor_index(0) = iter_index(0) + px;
                    neighbor_index(1) = iter_index(1) + py;
                    if(!indexInMap(neighbor_index) ||
                        isInSet(C_plus, point423(index2PosW(neighbor_index)))    || 
                        isInSet(C, point423(index2PosW(neighbor_index)))         ||
                        isInSet(neighbors, point423(index2PosW(neighbor_index))) ||
                        is_occupiedI(neighbor_index,0)){continue;}
                    neighbors.push_back(point423(index2PosW(neighbor_index)));
                }
            }
        }
        return neighbors;
    }

    bool LanGridMapManager::checkConvexity(vector<Eigen::Vector3d> C , Eigen::Vector3d pos)
    {
        for(int i = 0 ; i < C.size() ; i++)
        {
            //                                 here flate must be 0
            if( is_occupied_line( point324(C[i]) , point324(pos) ,0)) { return false;}
        }
        return true;
    }

    void CPushBack(vector<Eigen::Vector3d>& C, Eigen::Vector3d par ,
                    Eigen::Vector3d &C_north_edge, Eigen::Vector3d &C_south_edge,
                    Eigen::Vector3d &C_east_edge,  Eigen::Vector3d &C_west_edge, vector<Eigen::Vector3d> &C_edge)
    {
        if(par(0) > C_north_edge(0)) C_north_edge = par;
        if(par(0) < C_south_edge(0)) C_south_edge = par;
        if(par(1) > C_west_edge(1))  C_west_edge = par;
        if(par(1) < C_east_edge(1)) C_east_edge = par;

        Eigen::Vector3d v_sw, v_se, v_nw, v_ne;
        v_sw = Eigen::Vector3d(C_south_edge(0), C_west_edge(1) , 0);
        v_se = Eigen::Vector3d(C_south_edge(0), C_east_edge(1) , 0);
        v_nw = Eigen::Vector3d(C_north_edge(0), C_west_edge(1) , 0);
        v_ne = Eigen::Vector3d(C_north_edge(0), C_east_edge(1) , 0);
        C_edge.clear();
        C_edge.push_back(v_sw);
        C_edge.push_back(v_se);
        C_edge.push_back(v_nw);
        C_edge.push_back(v_ne);

        C.push_back(par);
    }

    vector<Eigen::Vector3d> LanGridMapManager::convexClusterInflation(Eigen::Vector2i seed_index)
    {
        vector<Eigen::Vector3d> C , C_plus , C_star, C_star_neighbors;
        Eigen::Vector3d C_north_edge = Eigen::Vector3d(-1e10,0,0);
        Eigen::Vector3d C_south_edge = Eigen::Vector3d(1e10,0,0);
        Eigen::Vector3d C_east_edge = Eigen::Vector3d(0,1e10,0);
        Eigen::Vector3d C_west_edge = Eigen::Vector3d(0,-1e10,0);
        vector<Eigen::Vector3d> C_edge;

        //C.push_back( point423(index2PosW(seed_index)) );
        CPushBack(C, point423(index2PosW(seed_index)) , C_north_edge,C_south_edge,C_east_edge,C_west_edge ,C_edge);
        //get neighbors
        C_plus = getNeighbors(C, C_plus,C);
        while(C_plus.size() != 0)
        {
            for(int i = 0; i < C_plus.size() ; i++)
            {
                if(checkConvexity(C, C_plus[i]))
                //if(checkConvexity(C_edge, C_plus[i]))
                {
                    //C.push_back(C_plus[i]);
                    CPushBack(C, C_plus[i] , C_north_edge,C_south_edge,C_east_edge,C_west_edge ,C_edge);
                    C_star.push_back(C_plus[i]);
                }
            }
            C_plus.clear();
            C_star_neighbors.clear();
            C_star_neighbors = getNeighbors(C_star, C_plus, C);
            for(int i = 0; i < C_star_neighbors.size() ; i++)
            {
                C_plus.push_back(C_star_neighbors[i]);
            }
            C_star.clear();

        }
        return C;
    }







    vector<Eigen::Vector3d> LanGridMapManager::getNNeighbors( vector<Eigen::Vector3d> set , vector<Eigen::Vector3d> C_plus, vector<Eigen::Vector3d> C)
    {
        vector<Eigen::Vector3d> neighbors;
        Eigen::Vector2i iter_index;
        Eigen::Vector2i neighbor_index;
        for(int i = 0 ; i < set.size(); i++)
        {
            iter_index = posW2Index(point324(set[i]));
            for(int px = -1 ; px <= 1 ; px++)
            {
                for(int py = -1; py <=1 ; py++)
                {
                    if(px == 0 && py == 0){continue;}
                    neighbor_index(0) = iter_index(0) + px;
                    neighbor_index(1) = iter_index(1) + py;
                    if(!indexInMap(neighbor_index) ||
                        isInSet(C_plus, point423(index2PosW(neighbor_index)))    || 
                        isInSet(C, point423(index2PosW(neighbor_index)))         ||
                        isInSet(neighbors, point423(index2PosW(neighbor_index))) ||
                        !is_stepI(neighbor_index) ){continue;}
                    neighbors.push_back(point423(index2PosW(neighbor_index)));
                }
            }
        }
        return neighbors;
    }

    bool LanGridMapManager::checkNConvexity(vector<Eigen::Vector3d> C , Eigen::Vector3d pos)
    {
        for(int i = 0 ; i < C.size() ; i++)
        {
            if( is_not_step_line( point324(C[i]) , point324(pos)) ) { return false;}
        }
        return true;
    }
    vector<Eigen::Vector3d> LanGridMapManager::convexNClusterInflation(Eigen::Vector2i seed_index)
    {
        vector<Eigen::Vector3d> C , C_plus , C_star, C_star_neighbors;
        Eigen::Vector3d C_north_edge = Eigen::Vector3d(-1e10,0,0);
        Eigen::Vector3d C_south_edge = Eigen::Vector3d(1e10,0,0);
        Eigen::Vector3d C_east_edge = Eigen::Vector3d(0,1e10,0);
        Eigen::Vector3d C_west_edge = Eigen::Vector3d(0,-1e10,0);
        vector<Eigen::Vector3d> C_edge;

        //C.push_back( point423(index2PosW(seed_index)) );
        CPushBack(C, point423(index2PosW(seed_index)) , C_north_edge,C_south_edge,C_east_edge,C_west_edge ,C_edge);
        //get neighbors
        C_plus = getNNeighbors(C, C_plus,C);
        while(C_plus.size() != 0)
        {
            for(int i = 0; i < C_plus.size() ; i++)
            {
                if(checkNConvexity(C, C_plus[i]))
                //if(checkConvexity(C_edge, C_plus[i]))
                {
                    //C.push_back(C_plus[i]);
                    CPushBack(C, C_plus[i] , C_north_edge,C_south_edge,C_east_edge,C_west_edge ,C_edge);
                    C_star.push_back(C_plus[i]);
                }
            }
            C_plus.clear();
            C_star_neighbors.clear();
            C_star_neighbors = getNNeighbors(C_star, C_plus, C);
            for(int i = 0; i < C_star_neighbors.size() ; i++)
            {
                C_plus.push_back(C_star_neighbors[i]);
            }
            C_star.clear();

        }
        return C;
    }

}