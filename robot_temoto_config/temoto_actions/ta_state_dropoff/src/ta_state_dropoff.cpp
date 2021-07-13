
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
#include "ta_state_dropoff/temoto_action.h"
#include "gazebo_msgs/DeleteModel.h"

/* 
 * ACTION IMPLEMENTATION of TaStateDropoff 
 */
class TaStateDropoff : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();
  TEMOTO_INFO_STREAM("Dropping off the cargo ...");
  
  delete_model_srvclient_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("robot_manager/robots/husky_sim/gazebo/delete_model");
  gazebo_msgs::DeleteModel delete_model_srvmsg;
  delete_model_srvmsg.request.model_name = "dummy_cargo";
  delete_model_srvclient_.call(delete_model_srvmsg);

  TEMOTO_INFO_STREAM("Done dropping off the cargo");
}

// Destructor
~TaStateDropoff()
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
ros::ServiceClient delete_model_srvclient_;


}; // TaStateDropoff class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaStateDropoff, ActionBase);
