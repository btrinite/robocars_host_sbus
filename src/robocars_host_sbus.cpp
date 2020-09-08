#include "robocars_host_sbus.h"
#include "robocars_host_sbus/SbusRosMessage.h"
#include "channel_mapping.h"

namespace robocars_host_sbus {

SBusBridge::SBusBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      destructor_invoked_(false) {
  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Publishers
  if (enable_receiving_sbus_messages_) {
    received_sbus_msg_pub_ =
        nh_.advertise<robocars_msgs::robocars_radio_channels>("/radio_channels", 1);
  }

  // Subscribers

  // Start serial port with receiver thread if receiving sbus messages is
  // enabled
  if (!setUpSBusSerialPort(port_name_, enable_receiving_sbus_messages_)) {
    ros::shutdown();
    return;
  }
}

SBusBridge::~SBusBridge() {
  destructor_invoked_ = true;

  // Stop SBus receiver thread
  if (enable_receiving_sbus_messages_) {
    stopReceiverThread();
  }

  // Close serial port
  disconnectSerialPort();
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg& received_sbus_msg) {
  received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}




bool SBusBridge::loadParameters() {

  if (!pnh_.hasParam("port_name")) {
       ROS_INFO("[%s] Missing configuration port_name", pnh_.getNamespace().c_str());
  }
  pnh_.getParam("port_name",port_name_);
  pnh_.getParam("enable_receiving_sbus_messages",enable_receiving_sbus_messages_);
    
  ROS_INFO("[%s] Configured port %s", pnh_.getNamespace().c_str(), port_name_.c_str());

  return true;

}

}  // namespace 

int main(int argc, char **argv) {
  ros::init(argc, argv, "sbus_bridge");

  robocars_host_sbus::SBusBridge sbus_bridge;

  ros::spin();

  return 0;
}
