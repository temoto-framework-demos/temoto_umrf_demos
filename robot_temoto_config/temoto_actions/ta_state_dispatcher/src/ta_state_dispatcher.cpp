
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

#include <deque>
#include <class_loader/class_loader.hpp>
#include "ta_state_dispatcher/temoto_action.h"
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"

/* 
 * ACTION IMPLEMENTATION of TaStateDispatcher 
 */
class TaStateDispatcher : public TemotoAction
{
public:

/*
 * Function that gets invoked only once (when the action is initialized) throughout the action's lifecycle
 */
void initializeTemotoAction()
{
  fillStateTransitionSequence();
  battery_state_sub_ = nh_.subscribe("robot_manager/robots/husky_sim/battery_state", 1, &TaStateDispatcher::batteryStateCallback, this);
  TEMOTO_INFO_STREAM("Initializing the dispatcher ... done");
}

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  TEMOTO_INFO_STREAM("Returned from state '" << current_state_ << "'. Deciding on the next state ...");

  // If the queue is empty then add new states to the queue
  if (state_transition_sequence_.empty())
  {
    fillStateTransitionSequence();
  }

  // If the robot has to be charged then add a charging and navigation states in front of the state queue
  if (robot_must_charge_)
  {
    // Pick up or drop off the package before moving to the charger
    if (state_transition_sequence_.front() != "state_navigate")
    {
      std::string tmp = state_transition_sequence_.front();
      state_transition_sequence_.pop_front();
      state_transition_sequence_.push_front("state_charge");
      state_transition_sequence_.push_front("state_navigate");
      state_transition_sequence_.push_front(tmp);
    }
    else
    {
      state_transition_sequence_.push_front("state_charge");
      state_transition_sequence_.push_front("state_navigate");
    }
  }

  // If the next state is navigation state, then assign the appropriate coordinates based on proceeding state
  if (state_transition_sequence_.front() == "state_navigate")
  {
    std::string next_state = state_transition_sequence_.at(1);

    if (next_state == "state_pickup")
    {
      out_param_nav_goal_x = -7;
      out_param_nav_goal_y = 2;
      out_param_nav_goal_yaw = 1.57;
    }
    else if (next_state == "state_dropoff")
    {
      out_param_nav_goal_x = -5;
      out_param_nav_goal_y = -5;
      out_param_nav_goal_yaw = 0;
    }
    else if (next_state == "state_charge")
    {
      out_param_nav_goal_x = 0;
      out_param_nav_goal_y = 0;
      out_param_nav_goal_yaw = 0;
    }
  }

  current_state_ = state_transition_sequence_.front();
  state_transition_sequence_.pop_front();

  TEMOTO_INFO_STREAM("Dispatching state '" << current_state_ << "'");
  out_param_state = current_state_;
  setOutputParameters();
}

void batteryStateCallback(const sensor_msgs::BatteryState& msg)
{
  if (msg.percentage < battery_min_charge_percentage_)
  {
    robot_must_charge_ = true;
  }
  else
  {
    robot_must_charge_ = false;
  }
}

void fillStateTransitionSequence()
{
  state_transition_sequence_.push_back("state_navigate");
  state_transition_sequence_.push_back("state_pickup");
  state_transition_sequence_.push_back("state_navigate");
  state_transition_sequence_.push_back("state_dropoff");
}

// Sets the output parameters which can be passed to other actions
void setOutputParameters()
{
  SET_PARAMETER("nav_goal::x", "number", out_param_nav_goal_x);
  SET_PARAMETER("nav_goal::y", "number", out_param_nav_goal_y);
  SET_PARAMETER("nav_goal::yaw", "number", out_param_nav_goal_yaw);
  SET_PARAMETER("state", "string", out_param_state);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Declaration of output parameters
double out_param_nav_goal_x;
double out_param_nav_goal_y;
double out_param_nav_goal_yaw;
std::string out_param_state;

// Other action specific members
ros::NodeHandle nh_;
ros::Subscriber battery_state_sub_;
std::string current_state_;
float battery_min_charge_percentage_ = 0.9;
bool robot_must_charge_ = false;

// const std::map<std::string, std::string> next_state_transition = 
// { {"state_initialize", "state_pickup"}
// , {"state_pickup", "state_dropoff"}
// , {"state_charge", "state_pickup"}
// , {"state_dropoff", "state_pickup"}};

std::deque<std::string> state_transition_sequence_;

}; // TaStateDispatcher class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaStateDispatcher, ActionBase);
