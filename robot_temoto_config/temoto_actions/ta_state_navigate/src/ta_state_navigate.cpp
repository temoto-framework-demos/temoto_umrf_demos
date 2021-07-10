
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

#include <class_loader/class_loader.hpp>
#include "ta_state_navigate/temoto_action.h"
#include "temoto_robot_manager/robot_manager_interface.h"
#include "tf/tf.h"

/* 
 * ACTION IMPLEMENTATION of TaStateNavigate 
 */
class TaStateNavigate : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();
  rmi_.initialize();

  /*
   * Move the robot
   */
  geometry_msgs::PoseStamped target_pose;
  target_pose.pose.position.x = in_param_nav_goal_x;
  target_pose.pose.position.y = in_param_nav_goal_y;
  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, in_param_nav_goal_yaw);

  bool goal_reached = false;
  while (!goal_reached && actionOk())
  try
  {
    TEMOTO_INFO("Moving to location [x = %.2f m; y = %.2f m; yaw = %.2f rad] ..."
    , in_param_nav_goal_x
    , in_param_nav_goal_y
    , in_param_nav_goal_yaw);
    rmi_.navigationGoal(robot_name_, "map", target_pose);

    TEMOTO_INFO_STREAM_("Done navigating");
    goal_reached = true;
  }
  catch(const resource_registrar::TemotoErrorStack &e)
  {
    TEMOTO_WARN_STREAM_("Caught an error: " << e.what() << "\nRequesting the same navigation goal again ... ");
  }
}

// Destructor
~TaStateNavigate()
{
  TEMOTO_DEBUG("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_nav_goal_x = GET_PARAMETER("nav_goal::x", double);
  in_param_nav_goal_y = GET_PARAMETER("nav_goal::y", double);
  in_param_nav_goal_yaw = GET_PARAMETER("nav_goal::yaw", double);
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
double in_param_nav_goal_x;
double in_param_nav_goal_y;
double in_param_nav_goal_yaw;
std::string in_param_state;

temoto_robot_manager::RobotManagerInterface rmi_;
std::string robot_name_ = "husky_sim";


}; // TaStateNavigate class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaStateNavigate, ActionBase);
