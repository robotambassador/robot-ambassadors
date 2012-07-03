/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, UC Regents
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of California nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <map>
#include <string>
#include <set>
#include <algorithm>
#include "topic_tools/MuxAdd.h"
#include "topic_tools/MuxSelect.h"
#include "topic_tools/MuxList.h"
//#include "flyer_controller/control_mode.h"
#include "flyer_controller/control_modes.h"
#include "flyer_controller/control_mode_cmd.h"
#include "flyer_controller/control_mode_status.h"
#include "flyer_controller/controller_status.h"
#include "flyer_controller/controller_cmd.h"
#include <boost/algorithm/string.hpp>

using namespace std;

namespace flyer_controller
{

namespace ControllerTypes
{
enum ControllerStates
{
  ERROR = 0, OFF = 1, INITIALIZE = 2, STANDBY = 3, OPERATIONAL = 4
};
typedef ControllerStates ControllerState;

}

using namespace ControllerTypes;

class Controller : public nodelet::Nodelet
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  // Parameters
  string initial_active_mode;
  string initial_standby_mode;
  double mainloop_rate;
  // Publishers
  ros::Publisher status_pub;
  map<string, ros::Publisher> mode_cmd_pubs;
  // Subscribers
  ros::Subscriber cmd_sub;
  map<string, ros::Subscriber> mode_status_subs;
  // Timers
  ros::Timer mainloop_timer;
  // Diagnostic Updater
  diagnostic_updater::Updater diag_updater;

  // Service Servers
  ros::ServiceServer control_modes_service;
  // Members
  string current_mode;
  string standby_mode;
  set<string> registered_modes;
  map<string, string> node_mode_map; // map from node name to mode
  map<string, string> mode_node_map; //map from mode name to node
  map<string, control_mode_status> latest_mode_status;
  ControllerState state;
  string info;
  bool command_pending;
  string pending_command;
  boost::mutex mainloop_mutex;

  // Methods
public:
  Controller() :
    initial_active_mode("idle"), initial_standby_mode("attitude"), mainloop_rate(5.0), diag_updater(),
        state(OFF), info(""), command_pending(false), pending_command("")
  {
  }

  void onInit()
  {

    nh = getMTNodeHandle();
    nh_priv = getMTPrivateNodeHandle();

    NODELET_INFO("controller.cpp onInit() begin");

    // Diagnostics
    diag_updater.add("Controller Status", this, &Controller::diagnostics);
    diag_updater.setHardwareID("none");
    diag_updater.force_update();
    // Parameters
    nh_priv.param("initial_active_mode", initial_active_mode, initial_active_mode);
    nh_priv.param("initial_standby_mode", initial_standby_mode, initial_standby_mode);
    nh_priv.param("mainloop_rate", mainloop_rate, mainloop_rate);
    // Publishers
    status_pub = nh_priv.advertise<controller_status> ("status", 1, true);
    // Subscribers
    cmd_sub = nh_priv.subscribe("cmd", 10, &Controller::commandCallback, this);
    // Service Server
    control_modes_service = nh_priv.advertiseService("control_modes", &Controller::controlModesServiceCallback, this);

    // Timers
    mainloop_timer = nh.createTimer(ros::Duration(1 / mainloop_rate), &Controller::mainLoopCallback, this);

    state = INITIALIZE;

    NODELET_INFO("controller.cpp onInit() done");

  }

  std::string stateToString(ControllerTypes::ControllerState t)
  {
    switch (t)
    {
      case ControllerTypes::OFF:
        return "OFF";
      case ControllerTypes::ERROR:
        return "ERROR";
      case ControllerTypes::INITIALIZE:
        return "INITIALIZE";
      case ControllerTypes::STANDBY:
        return "STANDBY";
      case ControllerTypes::OPERATIONAL:
        return "OPERATIONAL";
    }
    return "Unknown";
  }

private:

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    int status;
    status = ((state != ERROR) ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR);
    stat.summary(status, stateToString(state));
    stat.add("info", info);
    stat.add("registered modes", registered_modes.size());
    stat.add("current mode", current_mode);
    stat.add("standby mode", standby_mode);
  }

  void commandCallback(const controller_cmdConstPtr& msg)
  {
    // TODO: we could implement "callerid" to ensure that we are only getting commands
    // from 'authorized' sources..
    ROS_INFO_STREAM("Command received: " << msg->cmd);
    if (command_pending)
    {
      ROS_ERROR("Command already pending - new command ignored");
    }
    else
    {
      command_pending = true;
      pending_command = msg->cmd;
    }
  }

  bool controlModesServiceCallback(control_modes::Request& request, control_modes::Response& response)
  {
    ROS_INFO_STREAM("Service call received. Request was: " << request.request);
    vector<string> words;
    boost::split(words, request.request, boost::is_any_of(" "));

    if (words.size() > 0)
    {
      if (words[0] == "register")
      {
        if (words.size() != 3)
        {
          ROS_ERROR("Invalid request");
          response.response = "failure";
          response.reason = "invalid number of parameters";
          return true;
        }
        else
        {
          string mode_name = words[1];
          string mode_node = words[2];
          if (registered_modes.count(mode_name) > 0)
          {
            ROS_ERROR_STREAM("A control mode named " << mode_name << " has already been registered.");
            response.response = "failure";
            response.reason = "control mode already registered";
            return true;
          }
          else
          {
            ROS_INFO_STREAM("Registering control mode, name: " << mode_name << ", node: " << mode_node);
            mode_cmd_pubs[mode_name] = nh.advertise<control_mode_cmd> (mode_node + "/cmd", 1, true);
            //string topic = mode_node + "/control_mode_status";
            //            mode_status_subs[mode_name] = nh.subscribe(mode_node + "/status", 10,
            //                                                       &Controller::controlModesStatusCallback, this);
            mode_status_subs[mode_name]
                = nh.subscribe<control_mode_status> (mode_node + "/status", 10,
                                                     boost::bind(&Controller::controlModesStatusCallback, this, _1,
                                                                 mode_node));
            registered_modes.insert(mode_name);
            node_mode_map[nh.resolveName(mode_node)] = mode_name;
            mode_node_map[mode_name] = nh.resolveName(mode_node);
            response.response = "success";
            return true;
          }
        }
      }
    }
    response.response = "failure";
    response.reason = "invalid request";

    return true; // or are we supposed to return false upon failure?
  }

  void controlModesStatusCallback(const flyer_controller::control_mode_statusConstPtr& msg, string source) //const ros::MessageEvent<flyer_controller::control_mode_status const>& event)
  {
    //    const std::string& publisher_name = event.getPublisherName();
    //    ROS_INFO_STREAM("Got control mode status from: " << publisher_name);
    //    latest_mode_status[node_mode_map[publisher_name]] = *(event.getMessage()); //msg;
    latest_mode_status[node_mode_map[source]] = *msg; //msg;
  }

  void mainLoopCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock lock (mainloop_mutex);

    diag_updater.update();

    switch (state)
    {
      case ERROR:
        break;
      case OFF:
        break;
      case INITIALIZE:
        execute_INITIALIZE();
        break;
      case STANDBY:
        execute_STANDBY();
        break;
      case OPERATIONAL:
        execute_OPERATIONAL();
        break;
      default:
        break;
    }
    controller_status status_msg;
    status_msg.state = state;
    status_msg.info = info;
    status_msg.active_mode = current_mode;
    map<string, control_mode_status>::iterator it;
    getStandbyModes(status_msg.standby_modes);
    status_pub.publish(status_msg);
  }

  void getStandbyModes(vector<string>& modes)
  {
    map<string, control_mode_status>::iterator it;
    modes.clear();
    for (it = latest_mode_status.begin(); it != latest_mode_status.end(); it++)
    {
      if ((*it).second.state == control_mode_status::STATE_STANDBY)
      {
        modes.push_back((*it).first);
      }
    }

  }

  void execute_INITIALIZE()
  {
    // In this state we are waiting for our initial active and standby modes to register themselves..
    // We also need the mux to be ready to add topics
    static bool initialize_waiting_modes_mux = true, initialize_transitioned_active_mode = false,
                initialize_transitioned_standby_mode = false, initialize_active_mode_in_standby = false,
                initialize_standby_mode_in_standby = false, initialize_active_muxed = false, initialize_standby_muxed =
                    false;
    enum InitializePhases
    {
      INITIALIZE_WAIT_MODES_MUX, INITIALIZE_TRANSITION_ACTIVE, INITIALIZE_TRANSITION_STANDBY,
      INITIALIZE_WAIT_STANDBY_TALKBACK
    };

    static InitializePhases init_phase = INITIALIZE_WAIT_MODES_MUX;

    // TODO: complete this refactoring..
    switch (init_phase)
    {
      case INITIALIZE_WAIT_MODES_MUX:
      {

      }
        break;
    }

    if (initialize_waiting_modes_mux)
    {
      if (registered_modes.count(initial_active_mode) > 0 && registered_modes.count(initial_standby_mode) > 0
          && ros::service::exists("controller_mux/add", false))
      {
        ROS_INFO("Initial modes have registered");
        info = "got modes and mux";
        diag_updater.force_update();
        initialize_waiting_modes_mux = false;
      }
      else
      {
        info = "waiting for modes and mux";
      }
    }
    else // got modes and mux
    {
      topic_tools::MuxAdd mux_add_srv;
      if (!initialize_transitioned_active_mode) // need to command active mode to STANDBY
      {
        control_mode_cmd active_to_standby_msg;
        active_to_standby_msg.cmd = "mode standby";
        mode_cmd_pubs[initial_active_mode].publish(active_to_standby_msg);
        initialize_transitioned_active_mode = true;
      }
      else
      {
        if (!initialize_active_muxed)
        {
          mux_add_srv.request.topic = "control_mode_" + initial_active_mode + "/output";
          if (ros::service::call("controller_mux/add", mux_add_srv))
          {
            initialize_active_muxed = true;
          }
        }
      }
      if (!initialize_transitioned_standby_mode) // need to command active mode to STANDBY
      {
        control_mode_cmd standby_to_standby_msg;
        standby_to_standby_msg.cmd = "mode standby";
        mode_cmd_pubs[initial_standby_mode].publish(standby_to_standby_msg);
        initialize_transitioned_standby_mode = true;
      }
      else
      {
        if (!initialize_standby_muxed)
        {
          mux_add_srv.request.topic = "control_mode_" + initial_standby_mode + "/output";
          if (ros::service::call("controller_mux/add", mux_add_srv))
          {
            initialize_standby_muxed = true;
          }
        }
      }
      if (initialize_transitioned_active_mode && initialize_active_muxed && initialize_transitioned_standby_mode
          && initialize_standby_muxed)
      {
        info = "waiting for STANDBY talkback";
        //ROS_INFO_STREAM("Latest active status:" << int(latest_mode_status[initial_active_mode].state));
        if (latest_mode_status[initial_active_mode].state == control_mode_status::STATE_STANDBY
            && latest_mode_status[initial_standby_mode].state == control_mode_status::STATE_STANDBY)
        {
          ROS_INFO_STREAM("Initial active and standby modes reporting STANDBY state, controller going to STANDBY");
          state = STANDBY;
          info = "";
          current_mode = initial_active_mode;
          standby_mode = initial_standby_mode;
          diag_updater.force_update();
        }
      }
    }
  }

  void execute_STANDBY()
  {
    // This is the state that the Controller should end up in after a successful startup, with no action on the operator's part
    // required. In this state, both the 'active' and 'standby' control modes are in state STANDBY, meaning that they are
    // receiving joystick and state messages and they are routed to the mux. The mux however is still routing the nonexistent
    // 'dummy' topic.
    // In order to proceed to OPERATIONAL, the operator should have to perform some action, like clicking a specific joystick button
    // or something like that.

    // STANDBY:
    static bool standby_entering_operational = false, standby_commanded_active_mode = false,
                standby_mux_selected_active = false;

    // TODO: add checking that current and standby modes are still in standby as expected; otherwise... go to ERROR?
    if (command_pending)
    {
      if (pending_command == "mode operational" and !standby_entering_operational)
      {
        ROS_INFO("Entering OPERATIONAL");
        command_pending = false;
        pending_command = "";
        info = "entering OPERATIONAL";
        diag_updater.force_update();
        standby_entering_operational = true;
      }
      else
      {
        ROS_ERROR("Invalid command in STANDBY mode");
        command_pending = false;
        pending_command = "";
      }
    }
    if (standby_entering_operational)
    {
      if (!standby_commanded_active_mode)
      {
        // To enter operational, the current mode has to be successfully transitioned to ACTIVE
        control_mode_cmd current_to_active_msg;
        current_to_active_msg.cmd = "mode active";
        mode_cmd_pubs[current_mode].publish(current_to_active_msg);
        standby_commanded_active_mode = true;
        info = "commanded current mode to ACTIVE";
      }
      else
      {
        if (!standby_mux_selected_active)
        {
          topic_tools::MuxSelect mux_sel_srv;
          mux_sel_srv.request.topic = mode_node_map[current_mode] + "/output";
          ros::service::call("controller_mux/select", mux_sel_srv);
          standby_mux_selected_active = true;
          info = "requested mux select: " + mode_node_map[current_mode] + "/output";
        }
        else
        {
          // TODO: listen for mux talkback
        }
        // Check that current mode is reporting back as active
        if (latest_mode_status[current_mode].state == control_mode_status::STATE_ACTIVE)
        {
          state = OPERATIONAL;
          info = "current mode reporting ACTIVE";
          diag_updater.force_update();
        }
      }
    }
  }

  enum OPERATIONAL_Substate
  {
    OP_READY, OP_ACTIVATING_MODE, OP_STANDBYING_MODE
  };

  void execute_OPERATIONAL()
  {
    // Now in operational is where flying can be done. Controller's main job here is to arbitrate the switching of
    // modes. Note that since Controller has really no knowledge of what any specific mode does, it is up to the mode
    // being transitioned-to to determine if it's safe/appropriate to go ACTIVE.
    // The process of switching modes proceeds as follows:
    // - new_mode must equal standby_mode
    // - new_mode must be reporting 'ready' to become active (how?)
    // - (growth) active_mode is asked if it is okay to become inactive (think of a real case where this is needed)
    // - new_mode is commanded to become active and mux selects it
    static OPERATIONAL_Substate substate = OP_READY;
    static OPERATIONAL_Substate prev_substate = OP_READY;
    bool reset = true;
    string new_mode = "";

    if (substate == OP_READY)
    {
      execute_OPERATIONAL_READY(substate, new_mode);
    }
    if (substate != prev_substate)
    {
      ROS_INFO_STREAM("Operational substate changed: " << prev_substate << " -> " << substate);
      reset = true;
    }
    else
    {
      reset = false;
    }

    prev_substate = substate;

    bool substate_completed = false;
    switch (substate)
    {
      case OP_READY:
        break;
      case OP_ACTIVATING_MODE:
        substate_completed = execute_OPERATIONAL_ACTIVATING_MODE(reset, new_mode);
        break;
      case OP_STANDBYING_MODE:
        substate_completed = execute_OPERATIONAL_STANDBYING_MODE(reset, new_mode);
        break;
    }
    if (substate_completed)
    {
      substate = OP_READY;
    }

  }

  void execute_OPERATIONAL_READY(OPERATIONAL_Substate& substate, string& new_mode)
  {
    // In this substate we are ready to accept a new command. If one is pending,
    // it is parsed to determine its validity. If valid, then the appropriate substate
    // is returned

    if (command_pending)
    {
      bool valid = false;
      string reason = "";
      vector<string> words;
      boost::split(words, pending_command, boost::is_any_of(" "));
      if (words.size() > 0)
      {
        if (words[0] == "control_mode")
        // Do something with control modes. E.g.:
        // control_mode to_active attitude
        // control_mode to_standby hover
        {
          if (words.size() == 3)
          {
            if (words[1] == "to_active")
            {
              new_mode = words[2];
              if (new_mode == current_mode)
              {
                reason = ": current_mode already " + new_mode;
              }
              else
              {
                vector<string>::iterator it;
                vector<string> standby_modes;
                getStandbyModes(standby_modes);
                if (find(standby_modes.begin(), standby_modes.end(), new_mode) != standby_modes.end())//(new_mode == standby_mode)
                {
                  valid = true;
                  info = "mode change accepted";
                  substate = OP_ACTIVATING_MODE;
                }
                else
                {
                  ROS_ERROR_STREAM("Mode " << new_mode << " is not in STANDBY, cannot activate");
                }
              }
            }
            else if (words[1] == "to_standby")
            {
              new_mode = words[2];
              valid = true;
              substate = OP_STANDBYING_MODE;
            }
            else if (words[1] == "to_idle")
            {
              ROS_ERROR("control_mode to_idle commands not yet implemented");
            }
          }
        }
      }

      if (!valid)
      {
        ROS_ERROR_STREAM("Invalid command" << reason);
        info = "mode change rejected";
      }

      // In any event the command has been processed..
      command_pending = false;
      pending_command = "";
    }

  }

  bool execute_OPERATIONAL_ACTIVATING_MODE(bool reset, const string mode_to_activate)
  {
    enum SequenceStatus
    {
      MODE_CHANGE_ACCEPTED, WAITING_NEW_ACTIVE_TALKBACK, WAITING_OLD_STANDBY_TALKBACK, FINISHED
    };
    static SequenceStatus sequence_status = FINISHED;
    static string new_mode;
    ROS_INFO_STREAM("execute_OPERATIONAL_ACTIVATING_MODE, reset=" << reset << ", new_mode=" << new_mode
        << ", sequence_status=" << sequence_status);
    if (reset)
    {
      sequence_status = MODE_CHANGE_ACCEPTED;
      new_mode = mode_to_activate;
    }

    switch (sequence_status)
    {
      case MODE_CHANGE_ACCEPTED:
      {
        if (latest_mode_status[new_mode].ready)
        {
          // Mux select new_mode/output
          topic_tools::MuxSelect mux_sel_srv;
          mux_sel_srv.request.topic = mode_node_map[new_mode] + "/output";
          ros::service::call("controller_mux/select", mux_sel_srv);
          // Transition new_mode to ACTIVE:
          control_mode_cmd new_to_active_msg;
          new_to_active_msg.cmd = "mode active";
          mode_cmd_pubs[new_mode].publish(new_to_active_msg);
          info = "mux selected and commanded current mode to ACTIVE - waiting to see ACTIVE";
          sequence_status = WAITING_NEW_ACTIVE_TALKBACK;
        }
        else
        {
          ROS_ERROR_STREAM("mode " << new_mode << " reporting not ready to go ACTIVE, mode transition cancelled");
          info = "mode change failed -- see log";
          sequence_status = FINISHED;
        }
      }
        break;
      case WAITING_NEW_ACTIVE_TALKBACK:
      {
        if (latest_mode_status[new_mode].state == control_mode_status::STATE_ACTIVE)
        {
          // Transition old mode to STANDBY:
          control_mode_cmd old_to_standby_msg;
          old_to_standby_msg.cmd = "mode standby";
          mode_cmd_pubs[current_mode].publish(old_to_standby_msg);
          info = "commanded current mode to STANDBY - waiting to see STANDBY";
          sequence_status = WAITING_OLD_STANDBY_TALKBACK;
        }
      }
        break;
      case WAITING_OLD_STANDBY_TALKBACK:
      {
        if (latest_mode_status[current_mode].state == control_mode_status::STATE_STANDBY)
        {
          info = "old mode in STANDBY -- mode transition complete!";
          standby_mode = current_mode;
          current_mode = new_mode;
          sequence_status = FINISHED;
        }
      }
        break;
      case FINISHED:
      {
        ROS_ERROR("The code shouldn't get here--please debug!");
      }
        break;
    }

    return (sequence_status == FINISHED);
  }

  bool execute_OPERATIONAL_STANDBYING_MODE(bool reset, const string mode_to_standby)
  {
    enum SequenceStatus
    {
      MODE_CHANGE_ACCEPTED, WAITING_STANDBY_TALKBACK, ADDING_MUXER_INPUT, WAITING_MUXER_TALKBACK, FINISHED
    };
    static SequenceStatus sequence_status = FINISHED;
    static string new_mode;
    static int muxer_talkback_wait_cycles = 10;

    ROS_INFO_STREAM("execute_OPERATIONAL_STANDBYING_MODE, reset=" << reset << ", new_mode=" << new_mode
        << ", sequence_status=" << sequence_status);
    if (reset)
    {
      sequence_status = MODE_CHANGE_ACCEPTED;
      new_mode = mode_to_standby;
      muxer_talkback_wait_cycles = 10;
    }

    switch (sequence_status)
    {
      case MODE_CHANGE_ACCEPTED:
      {
        // Transition new_mode to STANDBY:
        control_mode_cmd new_to_standby_msg;
        new_to_standby_msg.cmd = "mode standby";
        mode_cmd_pubs[new_mode].publish(new_to_standby_msg);
        info = "mode " + new_mode + " commanded to STANDBY - waiting to see STANDBY";
        sequence_status = WAITING_STANDBY_TALKBACK;
      }
        break;
      case WAITING_STANDBY_TALKBACK:
      {
        if (latest_mode_status[new_mode].state == control_mode_status::STATE_STANDBY)
        {
          info = "mode in STANDBY - adding to muxer";
          sequence_status = ADDING_MUXER_INPUT;
        }
      }
        break;
      case ADDING_MUXER_INPUT:
      {
        topic_tools::MuxAdd mux_add_srv;
        mux_add_srv.request.topic = "control_mode_" + new_mode + "/output";
        ros::service::call("controller_mux/add", mux_add_srv);
        info = "waiting for muxer talkback...";
        sequence_status = WAITING_MUXER_TALKBACK;
      }
        break;
      case WAITING_MUXER_TALKBACK:
      {
        topic_tools::MuxList mux_list_srv;
        ros::service::call("controller_mux/list", mux_list_srv);
        vector<string>::iterator it;
        string full_topic_name = ros::names::resolve("control_mode_" + new_mode + "/output");
        if (find(mux_list_srv.response.topics.begin(), mux_list_srv.response.topics.end(), full_topic_name)
            != mux_list_srv.response.topics.end())
        {
          sequence_status = FINISHED;
          info = "muxer has " + full_topic_name + " in its input list - mode is in STANDBY";
        }
        else
        {
          muxer_talkback_wait_cycles--;
        }
        if (muxer_talkback_wait_cycles <= 0)
        {
          ROS_ERROR_STREAM("Muxer still doesn't have topic " << full_topic_name << " after " << (10-muxer_talkback_wait_cycles) << "tries...");
        }
      }
        break;
      case FINISHED:
      {
        ROS_ERROR("The code shouldn't get here--please debug!");
      }
        break;
    }

    return (sequence_status == FINISHED);
  }

}; // class
PLUGINLIB_DECLARE_CLASS(flyer_controller, Controller, flyer_controller::Controller, nodelet::Nodelet)
;

} // namespace

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "controller");
//  flyer_controller::Controller controller;
//  ros::spin();
//
//  return 0;
//}

