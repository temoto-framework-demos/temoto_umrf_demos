
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
#include "ta_robot_keyboard_twist/temoto_action.h"
#include "temoto_robot_manager/robot_manager_interface.h"
#include "temoto_component_manager/component_manager_interface.h"

/* 
 * ACTION IMPLEMENTATION of TaRobotKeyboardTwist 
 */
class TaRobotKeyboardTwist : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();
  rmi_.initialize();
  cmi_.initialize();

  // Get the robot's configuration and find the name of the twist topic
  TEMOTO_INFO_STREAM_("trying to get config of '" << in_param_robot_name << "' ...");
  YAML::Node robot_config = rmi_.getRobotConfig(in_param_robot_name);
  TEMOTO_INFO_STREAM_("Config of robot '" << in_param_robot_name << "': " << robot_config);

  std::string robot_cmd_vel_topic = robot_config["robot_absolute_namespace"].as<std::string>() + "/"
    + robot_config["navigation"]["driver"]["cmd_vel_topic"].as<std::string>();
  TEMOTO_INFO_STREAM_("cmd_vel topic of '" << in_param_robot_name << "' is '" << robot_cmd_vel_topic << "'");

  // Start the keyboard_twist component and provide it the robot's twist topic
  ComponentTopicsReq req_topics;
  req_topics.addInputTopic("twist_topic", robot_cmd_vel_topic);
  cmi_.startComponent("joystick_twist", req_topics);
}

// Default Constructor (REQUIRED)
TaRobotKeyboardTwist()
{
  std::cout << __func__ << " constructed\n";
}

// Destructor
~TaRobotKeyboardTwist()
{
  TEMOTO_INFO("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_robot_name = GET_PARAMETER("robot_name", std::string);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

temoto_robot_manager::RobotManagerInterface rmi_;
temoto_component_manager::ComponentManagerInterface cmi_;

// Declaration of input parameters
std::string in_param_robot_name;


}; // TaRobotKeyboardTwist class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaRobotKeyboardTwist, ActionBase);
