
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://temoto-telerobotics.github.io
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <fstream>
#include <class_loader/class_loader.hpp>
#include "ta_state_initialize/temoto_action.h"
#include "temoto_robot_manager/robot_manager_interface.h"
#include "gazebo_msgs/SpawnModel.h"
#include "ros/package.h"

/* 
 * ACTION IMPLEMENTATION of TaStateInitialize 
 */
class TaStateInitialize : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  TEMOTO_INFO_STREAM("initializing robot " << robot_name_ << "...");
  rmi_.initialize();
  rmi_.loadRobot(robot_name_);

  /*
   * Get the SDF description of the location markers
   */
  spawn_model_srvclient_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("robot_manager/robots/husky_sim/gazebo/spawn_sdf_model");
  model_base_path_ = ros::package::getPath("robot_temoto_config") + "/launch/include";

  spawnModel("charger_location_marker", 0, 0);
  spawnModel("pickup_location_marker", -7, 2);
  spawnModel("dropoff_location_marker", -5, -5);

  TEMOTO_INFO_STREAM("The robot is initialized");
}

void spawnModel(std::string model_name, double x, double y)
{
  std::ifstream ifs(model_base_path_ + "/" + model_name + ".sdf");
  std::string content( (std::istreambuf_iterator<char>(ifs) ), (std::istreambuf_iterator<char>()));

  gazebo_msgs::SpawnModel spawn_model_srvmsg;
  spawn_model_srvmsg.request.model_name = model_name;
  spawn_model_srvmsg.request.model_xml = content;
  spawn_model_srvmsg.request.initial_pose.position.x = x;
  spawn_model_srvmsg.request.initial_pose.position.y = y;
  spawn_model_srvmsg.request.initial_pose.position.z = -0.2;
  spawn_model_srvmsg.request.initial_pose.orientation.w = 1;
  spawn_model_srvmsg.request.reference_frame = "world";

  spawn_model_srvclient_.call(spawn_model_srvmsg);
}

// Destructor
~TaStateInitialize()
{
  TEMOTO_DEBUG("Action instance destructed");
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

ros::NodeHandle nh_;
ros::ServiceClient spawn_model_srvclient_;
temoto_robot_manager::RobotManagerInterface rmi_;
std::string robot_name_ = "husky_sim";
std::string model_base_path_;

}; // TaStateInitialize class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaStateInitialize, ActionBase);
