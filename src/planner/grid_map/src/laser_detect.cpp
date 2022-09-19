
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <vector>

using namespace std;

const double pi = 3.1415926535;

ros::Subscriber colored_pointcloud_sub;
ros::Publisher  target_pub, laser_cloud_pub;

double pitch = 27 * 2 * pi / 360.0;
Eigen::AngleAxisd Vw_c = Eigen::AngleAxisd(pitch,  Eigen::Vector3d(0, 0, 1));
Eigen::Matrix3d   Rw_c = Vw_c.matrix();


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud)
{
    
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    
    pcl::fromROSMsg(*point_cloud, colored_cloud);
    
    vector<int> feasible_ids;
    feasible_ids.clear();

    
    for (size_t i = 0; i < colored_cloud.points.size (); ++i)  
    {  
        if(colored_cloud.points[i].g > 170 && colored_cloud.points[i].r + colored_cloud.points[i].b < 100)
        {
            feasible_ids.push_back(i);
        }
    } 

    pcl::PointCloud<pcl::PointXYZRGB> laser_cloud;
    laser_cloud.resize(feasible_ids.size());

    for (size_t i = 0; i < feasible_ids.size() ; ++i)  
    {  
        laser_cloud.points[i].x = colored_cloud.points[feasible_ids[i]].x;
        laser_cloud.points[i].y = colored_cloud.points[feasible_ids[i]].y;
        laser_cloud.points[i].z = colored_cloud.points[feasible_ids[i]].z;
        laser_cloud.points[i].r = colored_cloud.points[feasible_ids[i]].r;
        laser_cloud.points[i].g = colored_cloud.points[feasible_ids[i]].g;
        laser_cloud.points[i].b = colored_cloud.points[feasible_ids[i]].b;
    } 

    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(laser_cloud, cloud_msg);

    cloud_msg.header.frame_id = "world";
    laser_cloud_pub.publish(cloud_msg);

    //target
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(laser_cloud, centroid);
    Eigen::Vector3d centroid_c = Eigen::Vector3d(centroid(0) , centroid(1) , centroid(2)); 
    Eigen::Vector3d centroid_w = Rw_c * centroid_c;

    //cout << centroid_c << " | " <<centroid_w <<endl;

    if(laser_cloud.size() > 1)
    {
        geometry_msgs::PoseStamped laser_point_world;
        laser_point_world.header.frame_id    = "world";
        laser_point_world.header.stamp       = ros::Time::now();

        laser_point_world.pose.position.x    = centroid_w(2);
        laser_point_world.pose.position.y    = -centroid_w(0);
        laser_point_world.pose.position.z    = 0.0;

        target_pub.publish(laser_point_world);
    }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_detect");
  ros::NodeHandle nh("~");

  colored_pointcloud_sub = nh.subscribe("/camera/depth/color/points", 1, pointCloudCallback );
  laser_cloud_pub        = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud",1);
  target_pub             = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
