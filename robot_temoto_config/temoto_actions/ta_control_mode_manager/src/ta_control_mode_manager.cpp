
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
#include "ta_control_mode_manager/temoto_action.h"
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/umrf_graph_fs.h"
#include "temoto_action_engine/StartUmrfGraph.h"
#include "temoto_action_engine/StopUmrfGraph.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <boost/filesystem/operations.hpp>
#include <map>

/* 
 * ACTION IMPLEMENTATION of TaControlModeManager 
 */
class TaControlModeManager : public TemotoAction
{
public:

/*
 * Function that gets invoked when the action is executed (REQUIRED)
 */
void executeTemotoAction()
{
  getInputParameters();

  // Get the base path to UMRF graph json files
  std::string base_path = ros::package::getPath(in_param_umrf_graph_jsons_path) + "/../umrf_graphs";
  TEMOTO_INFO_STREAM("The base path to UMRF graphs is '" << base_path << "'");

  // Read the UMRF graph json files and convert them into UmrfGraph objects
  parseUmrfGraphFiles(base_path);

  // Initialize a publisher that allows to invoke UMRF graphs via the Action Engine
  start_umrf_graph_srvclient_ = nh_.serviceClient<temoto_action_engine::StartUmrfGraph>(start_umrf_graph_srv_name_);

  // Initialize a publisher that allows to stop UMRF graphs via the Action Engine
  stop_umrf_graph_srvclient_ = nh_.serviceClient<temoto_action_engine::StopUmrfGraph>(stop_umrf_graph_srv_name_);

  // Initialize a subscriber that converts "high-level" commands into UMRF graph ROS messages
  command_sub_ = nh_.subscribe(command_topic_name_, 1, &TaControlModeManager::commandCallback, this);

  // Now start Vaultbot by invoking the "vaultbot_initialize" graph
  TEMOTO_INFO_STREAM("Starting Vaultbot");
  temoto_action_engine::StartUmrfGraph start_umrfg_query;
  start_umrfg_query.request.umrf_graph_name = "vaultbot_initialize";
  start_umrfg_query.request.umrf_graph_json = umrf_json_converter::toUmrfGraphJsonStr(umrf_graphs_.at("vaultbot_initialize"));
  start_umrfg_query.request.name_match_required = true;
  start_umrf_graph_srvclient_.call(start_umrfg_query);
}

// Default Constructor (REQUIRED)
TaControlModeManager()
{
  std::cout << __func__ << " constructed\n";
}

// Destructor
~TaControlModeManager()
{
  TEMOTO_INFO("Action instance destructed");
}

// Loads in the input parameters
void getInputParameters()
{
  in_param_umrf_graph_jsons_path = GET_PARAMETER("umrf_graph_jsons_path", std::string);
}

// Sets the output parameters which can be passed to other actions
void setOutputParameters()
{
}

void commandCallback(const std_msgs::String& msg)
{
  TEMOTO_INFO_STREAM("Received a request to start UMRF graph '" << msg.data << "' ...");

  /*
   * Check if this graph is known
   */
  const auto& umrf_graph_it = umrf_graphs_.find(msg.data);
  if (umrf_graph_it == umrf_graphs_.end())
  {
    TEMOTO_WARN_STREAM("Looks like this UMRF graph is unknown, sorry");
    return;
  }

  /*
   * Stop the previous control mode
   */
  if (!current_control_mode_.empty())
  {
    TEMOTO_INFO_STREAM("Stopping the current control mode '" << current_control_mode_ << "' ...");
    temoto_action_engine::StopUmrfGraph stop_umrfg_query;
    stop_umrfg_query.request.umrf_graph_name = current_control_mode_;
    if (stop_umrf_graph_srvclient_.call(stop_umrfg_query))
    {
      current_control_mode_.clear();
    }
  }

  /*
   * Invoke the UMRF graph
   */
  TEMOTO_INFO_STREAM("Passing the request to the Action Engine ...");
  temoto_action_engine::StartUmrfGraph start_umrfg_query;
  start_umrfg_query.request.umrf_graph_name = msg.data;
  start_umrfg_query.request.umrf_graph_json = umrf_json_converter::toUmrfGraphJsonStr(umrf_graph_it->second);
  start_umrfg_query.request.name_match_required = true;
  if (start_umrf_graph_srvclient_.call(start_umrfg_query))
  {
    current_control_mode_ = msg.data;
  }
}

void parseUmrfGraphFiles(const std::string& umrf_parameters_path)
{
  for (auto& itr: boost::filesystem::directory_iterator(umrf_parameters_path))
  try
  {
    // if its a file and contains "umrfg.json" in its name, process the file. Otherwise skip
    if ( !boost::filesystem::is_regular_file(itr) 
    || (itr.path().string().find("umrfg.json") == std::string::npos) )
    {
      continue;
    }
    
    std::string umrf_graph_json_str = temoto_action_engine::readFromFile(itr.path().string());
    UmrfGraph ug = umrf_json_converter::fromUmrfGraphJsonStr(umrf_graph_json_str);
    umrf_graphs_.insert({ug.getName(), ug});
    TEMOTO_INFO_STREAM("Found UMRF graph '" << ug.getName() << "'");
  }
  catch (std::exception& e)
  {
    TEMOTO_WARN_STREAM("Unable to read/process UMRF graph file '" << itr.path().string() << "' because: " << e.what());
  }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Class members
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

ros::NodeHandle nh_;
ros::ServiceClient start_umrf_graph_srvclient_;
ros::ServiceClient stop_umrf_graph_srvclient_;
ros::Subscriber command_sub_;

std::string start_umrf_graph_srv_name_ = "start_umrf_graph";
std::string stop_umrf_graph_srv_name_ = "stop_umrf_graph";
std::string command_topic_name_ = "vaultbot_control_mode";

std::string current_control_mode_;
std::map<std::string, UmrfGraph> umrf_graphs_;

// Declaration of input parameters
std::string in_param_umrf_graph_jsons_path;


}; // TaControlModeManager class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaControlModeManager, ActionBase);
