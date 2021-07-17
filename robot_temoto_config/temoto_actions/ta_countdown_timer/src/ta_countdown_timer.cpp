
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

#include <thread>
#include <class_loader/class_loader.hpp>
#include "ta_countdown_timer/temoto_action.h"
#include "std_msgs/Int32.h"

/* 
 * ACTION IMPLEMENTATION of TaCountdownTimer 
 */
class TaCountdownTimer : public TemotoAction
{
public:

/*
 * Function that gets invoked only once (when the action is initialized) throughout the action's lifecycle
 */
void initializeTemotoAction()
{
  /* * * * * * * * * * * * * * * * * * * * * * *
   *                          
   * ===> YOUR INITIALIZATION ROUTINES HERE <===
   *                          
   * * * * * * * * * * * * * * * * * * * * * * */

  TEMOTO_INFO_STREAM("Action initialized");
}

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();

  ros::NodeHandle nh;
  ros::Publisher countdown_pub = nh.advertise<std_msgs::Int32>("countdown_timer", 1);
  std_msgs::Int32 countdown_msg;

  TEMOTO_INFO_STREAM("Starting to count down from " << in_param_count_from);
  for (int i=in_param_count_from; i>=0; i--)
  {
    TEMOTO_INFO_STREAM(i << " seconds remaining ...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    countdown_msg.data = i;
    countdown_pub.publish(countdown_msg);
  }
  TEMOTO_INFO("Countdown finished");
}

// Destructor
~TaCountdownTimer()
{
  TEMOTO_INFO("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_count_from = GET_PARAMETER("count_from", double);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Declaration of input parameters
double in_param_count_from;


}; // TaCountdownTimer class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaCountdownTimer, ActionBase);
