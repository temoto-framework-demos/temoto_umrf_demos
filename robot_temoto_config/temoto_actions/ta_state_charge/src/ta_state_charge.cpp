
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
#include "ta_state_charge/temoto_action.h"
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "robot_temoto_config/SetChargeMode.h"


/* 
 * ACTION IMPLEMENTATION of TaStateCharge 
 */
class TaStateCharge : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();

  battery_state_sub_ = nh_.subscribe("robot_manager/robots/husky_sim/battery_state", 1, &TaStateCharge::batteryStateCallback, this);
  battery_charge_mode_srvclient_ = nh_.serviceClient<robot_temoto_config::SetChargeMode>("robot_manager/robots/husky_sim/set_charge_mode");
  TEMOTO_INFO_STREAM("Charging the robot ...");
  
  robot_temoto_config::SetChargeMode charge_mode_srvmsg;
  charge_mode_srvmsg.request.charge_mode = robot_temoto_config::SetChargeMode::Request::CHARGING;
  battery_charge_mode_srvclient_.call(charge_mode_srvmsg);

  while(!battery_charged_ && actionOk())
  {
    ros::Duration(1).sleep();
  }

  charge_mode_srvmsg.request.charge_mode = robot_temoto_config::SetChargeMode::Request::DISCHARGING;
  battery_charge_mode_srvclient_.call(charge_mode_srvmsg);
  TEMOTO_INFO_STREAM("Done charging the robot");
}

void batteryStateCallback(const sensor_msgs::BatteryState& msg)
{
  if (msg.percentage > 0.99)
  {
    battery_charged_ = true;
  }
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_state = GET_PARAMETER("state", std::string);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Declaration of input parameters
std::string in_param_state;

// Other action specific members
ros::NodeHandle nh_;
ros::Subscriber battery_state_sub_;
ros::ServiceClient battery_charge_mode_srvclient_;
bool battery_charged_ = false;

}; // TaStateCharge class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaStateCharge, ActionBase);
