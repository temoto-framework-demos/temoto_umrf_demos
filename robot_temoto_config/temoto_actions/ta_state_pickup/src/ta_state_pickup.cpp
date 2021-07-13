
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
#include "ta_state_pickup/temoto_action.h"
#include "gazebo_msgs/SpawnModel.h"
#include "ros/package.h"
#include "gazebo_ros_link_attacher/Attach.h"

/* 
 * ACTION IMPLEMENTATION of TaStatePickup 
 */
class TaStatePickup : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();
  TEMOTO_INFO_STREAM("Picking up the cargo ...");
  
  /*
   * Get the SDF description of the dummy cargo
   */
  std::string base_path = ros::package::getPath("robot_temoto_config") + "/launch/include";
  std::string sdf_model_name = "cargo.sdf";
  std::ifstream ifs(base_path + "/" + sdf_model_name);
  std::string content( (std::istreambuf_iterator<char>(ifs) ), (std::istreambuf_iterator<char>()));

  spawn_model_srvclient_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("robot_manager/robots/husky_sim/gazebo/spawn_sdf_model");
  attach_model_srvclient_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("robot_manager/robots/husky_sim/link_attacher_node/attach");

  /*
   * Spawn the dummy cargo ontop of the robot
   */
  gazebo_msgs::SpawnModel spawn_model_srvmsg;
  spawn_model_srvmsg.request.model_name = "dummy_cargo";
  spawn_model_srvmsg.request.model_xml = content;
  spawn_model_srvmsg.request.initial_pose.position.x = 0;
  spawn_model_srvmsg.request.initial_pose.position.y = 0;
  spawn_model_srvmsg.request.initial_pose.position.z = 0.05;
  spawn_model_srvmsg.request.initial_pose.orientation.w = 1;
  spawn_model_srvmsg.request.reference_frame = "base_link";
  spawn_model_srvclient_.call(spawn_model_srvmsg);

  /*
   * Attach the dummy cargo ontop of the robot
   */
  gazebo_ros_link_attacher::Attach attach_model_srvmsg;
  attach_model_srvmsg.request.model_name_1 = "husky";
  attach_model_srvmsg.request.link_name_1 = "base_link";
  attach_model_srvmsg.request.model_name_2 = "dummy_cargo";
  attach_model_srvmsg.request.link_name_2 = "link";
  attach_model_srvclient_.call(attach_model_srvmsg);

  TEMOTO_INFO_STREAM("Done picking up the cargo");
}

// Destructor
~TaStatePickup()
{
  TEMOTO_DEBUG("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_state = GET_PARAMETER("state", std::string);
}

// Sets the output parameters which can be passed to other actions
void setOutputParameters()
{
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Declaration of input parameters
std::string in_param_state;

// Other members
ros::NodeHandle nh_;
ros::ServiceClient spawn_model_srvclient_;
ros::ServiceClient attach_model_srvclient_;


}; // TaStatePickup class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaStatePickup, ActionBase);
