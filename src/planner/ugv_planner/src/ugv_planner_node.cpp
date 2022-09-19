#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/planner_manager.h>

using namespace ugv_planner;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ugv_planner_node");
  ros::NodeHandle nh("~");

  UGVPlannerManager main_planner;
  main_planner.init(nh);
  
  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
