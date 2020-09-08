#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sbus_serial_port.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "sbus_msg.h"

namespace robocars_host_sbus {

enum class BridgeState { OFF, ARMING, AUTONOMOUS_FLIGHT, RC_FLIGHT };

class SBusBridge : public SBusSerialPort {
 public:
  SBusBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  SBusBridge() : SBusBridge(ros::NodeHandle(), ros::NodeHandle("~")) {}

  virtual ~SBusBridge();

 private:

  void handleReceivedSbusMessage(const SBusMsg& received_sbus_msg) override;

  bool loadParameters();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Mutex for:
  // - bridge_state_
  // - control_mode_
  // - bridge_armed_
  // - time_last_active_control_command_received_
  // - time_last_rc_msg_received_
  // - arming_counter_
  // - time_last_sbus_msg_sent_
  // Also "setBridgeState" and "sendSBusMessageToSerialPort" should only be
  // called when "main_mutex_" is locked
  mutable std::mutex main_mutex_;

  // Publishers
  ros::Publisher received_sbus_msg_pub_;

  // Subscribers

  std::atomic_bool destructor_invoked_;

  // Parameters
  std::string port_name_;
  bool enable_receiving_sbus_messages_;

};

}  // namespace 

