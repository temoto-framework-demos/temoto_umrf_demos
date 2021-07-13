
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
#include <mutex>
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
  std::lock_guard<std::mutex> l(shared_states_mutex_);
  
  // If the queue is empty then add new states to the queue
  if (state_transition_sequence_.empty())
  {
    fillStateTransitionSequence();
  }

  // If the robot has to be charged then add a charging and navigation states in front of the state queue
  if (robot_must_charge_ && !charging_state_queued_)
  {
    TEMOTO_WARN("The robot must be charged");

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
    charging_state_queued_ = true;
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
      out_param_nav_goal_yaw = -1.57;
    }
    else if (next_state == "state_charge")
    {
      out_param_nav_goal_x = 0;
      out_param_nav_goal_y = 0;
      out_param_nav_goal_yaw = 0;
    }
  }

  TEMOTO_INFO_STREAM("Returned from state '" << out_param_state << "'. Switching to state '" << state_transition_sequence_.front() << "'");
  out_param_state = state_transition_sequence_.front();
  state_transition_sequence_.pop_front();
  setOutputParameters();
}

void batteryStateCallback(const sensor_msgs::BatteryState& msg)
{
  std::lock_guard<std::mutex> l(shared_states_mutex_);
  if (msg.percentage < battery_min_charge_percentage_)
  {
    robot_must_charge_ = true;
  }
  else
  {
    robot_must_charge_ = false;
    charging_state_queued_ = false;
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
float battery_min_charge_percentage_ = 0.85;
bool robot_must_charge_ = false;
bool charging_state_queued_ = false;
std::mutex shared_states_mutex_;
std::deque<std::string> state_transition_sequence_;

}; // TaStateDispatcher class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaStateDispatcher, ActionBase);
