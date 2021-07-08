#include <mutex>
#include <chrono>
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "robot_temoto_config/SetChargeMode.h"

enum class State
{
  CHARGING,
  DISCHARGING
};

std::chrono::steady_clock::time_point previous_charge_calc_time;
State state = State::DISCHARGING;
ros::Publisher battery_state_pub;
ros::ServiceServer charge_mode_server;
std::mutex state_mutex;

double charge_percentage = 1;
double discharge_rate = 0.1; // discharging 10% in every minute
double charge_rate = 0.5; // charging 50% in every minute

void timerCallback(const ros::TimerEvent&)
{
  double time_diff_in_minutes = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - previous_charge_calc_time).count() / 60000.0;

  ROS_INFO_STREAM("Timediff is " << time_diff_in_minutes);

  /*
   * Increase or decrease the charge based on the state
   */
  std::lock_guard<std::mutex> l(state_mutex);
  switch (state)
  {
    case State::CHARGING:
      charge_percentage += time_diff_in_minutes*charge_rate;
      break;
    case State::DISCHARGING:
      charge_percentage -= time_diff_in_minutes*discharge_rate;
      break;
  }

  /*
   * Clip the values
   */
  if (charge_percentage > 1)
    charge_percentage = 1;
  else if (charge_percentage < 0)
    charge_percentage = 0;

  /*
   * Publish the battery state
   */
  sensor_msgs::BatteryState battery_msg;
  battery_msg.percentage = charge_percentage;
  battery_state_pub.publish(battery_msg);

  previous_charge_calc_time = std::chrono::steady_clock::now();
}

bool setChargeModeCb(robot_temoto_config::SetChargeMode::Request& req, robot_temoto_config::SetChargeMode::Response& res)
{
  std::lock_guard<std::mutex> l(state_mutex);
  if (req.charge_mode == req.CHARGING)
  {
    state = State::CHARGING;
  }
  else if (req.charge_mode == req.DISCHARGING)
  {
    state = State::DISCHARGING;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_charge_controller");
  ros::NodeHandle nh;

  battery_state_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 1);
  charge_mode_server = nh.advertiseService("set_charge_mode", &setChargeModeCb);

  previous_charge_calc_time = std::chrono::steady_clock::now();
  ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);
  ros::spin();
}