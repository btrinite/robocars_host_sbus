#include "robocars_host_sbus.h"
#include "robocars_host_sbus/SbusRosMessage.h"
#include "channel_mapping.h"

namespace robocars_host_sbus {

SBusBridge::SBusBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      stop_watchdog_thread_(false),
      time_last_rc_msg_received_(),
      time_last_sbus_msg_sent_(ros::Time::now()),
      bridge_state_(BridgeState::OFF),
      arming_counter_(0),
      bridge_armed_(false),
      rc_was_disarmed_once_(false),
      destructor_invoked_(false) {
  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Publishers
  if (enable_receiving_sbus_messages_) {
    received_sbus_msg_pub_ =
        nh_.advertise<robocars_host_sbus::SbusRosMessage>("received_sbus_message", 1);
  }

  // Subscribers

  // Start serial port with receiver thread if receiving sbus messages is
  // enabled
  if (!setUpSBusSerialPort(port_name_, enable_receiving_sbus_messages_)) {
    ros::shutdown();
    return;
  }

  // Start watchdog thread
  try {
    watchdog_thread_ = std::thread(&SBusBridge::watchdogThread, this);
  } catch (...) {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.",
              pnh_.getNamespace().c_str());
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

  // Stop watchdog thread
  stop_watchdog_thread_ = true;
  // Wait for watchdog thread to finish
  watchdog_thread_.join();

  // Now only one thread (the ROS thread) is remaining

  setBridgeState(BridgeState::OFF);

  // Send disarming SBus message for safety
  // We repeat it to prevent any possible smoothing of commands on the flight
  // controller to interfere with this
  SBusMsg shut_down_message;
  shut_down_message.setArmStateDisarmed();
  ros::Rate loop_rate(110.0);

  for (int i = 0; i < kSmoothingFailRepetitions_; i++) {
    transmitSerialSBusMessage(shut_down_message);
    loop_rate.sleep();
  }

  // Close serial port
  disconnectSerialPort();
}

void SBusBridge::watchdogThread() {
  ros::Rate watchdog_rate(110.0);
  while (ros::ok() && !stop_watchdog_thread_) {
    watchdog_rate.sleep();

    std::lock_guard<std::mutex> main_lock(main_mutex_);

    const ros::Time time_now = ros::Time::now();

    if (bridge_state_ == BridgeState::RC_FLIGHT &&
        time_now - time_last_rc_msg_received_ > ros::Duration(rc_timeout_)) {
      // If the last received RC command was armed but was received longer than
      // rc_timeout ago we switch the bridge state to AUTONOMOUS_FLIGHT.
      // In case there are no valid control commands the bridge state is set to
      // OFF in the next check below
      ROS_WARN(
          "[%s] Remote control was active but no message from it was received "
          "within timeout (%f s).",
          pnh_.getNamespace().c_str(), rc_timeout_);
      setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
    }

    if (bridge_state_ == BridgeState::OFF) {
      // Send off message that disarms the vehicle
      // We repeat it to prevent any weird behavior that occurs if the flight
      // controller is not receiving commands for a while
      SBusMsg off_msg;
      off_msg.setArmStateDisarmed();
      sendSBusMessageToSerialPort(off_msg);
    }

    // Mutexes are unlocked because they go out of scope here
  }
}

void SBusBridge::handleReceivedSbusMessage(const SBusMsg& received_sbus_msg) {
  {
    std::lock_guard<std::mutex> main_lock(main_mutex_);

    time_last_rc_msg_received_ = ros::Time::now();

    if (received_sbus_msg.isArmed()) {
      if (!rc_was_disarmed_once_) {
        // This flag prevents that the vehicle can be armed if the RC is armed
        // on startup of the bridge
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] RC needs to be disarmed once before it can take over control",
            pnh_.getNamespace().c_str());
        return;
      }

      // Immediately go into RC_FLIGHT state since RC always has priority
      if (bridge_state_ != BridgeState::RC_FLIGHT) {
        setBridgeState(BridgeState::RC_FLIGHT);
        ROS_INFO("[%s] Control authority taken over by remote control.",
                 pnh_.getNamespace().c_str());
      }
      sendSBusMessageToSerialPort(received_sbus_msg);
    } else if (bridge_state_ == BridgeState::RC_FLIGHT) {
      // If the bridge was in state RC_FLIGHT and the RC is disarmed we set the
      // state to AUTONOMOUS_FLIGHT
      // In case there are valid control commands, the bridge will stay in
      // AUTONOMOUS_FLIGHT, otherwise the watchdog will set the state to OFF
      ROS_INFO("[%s] Control authority returned by remote control.",
               pnh_.getNamespace().c_str());
      if (bridge_armed_) {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      } else {
        // When switching the bridge state to off, our watchdog ensures that a
        // disarming off message is sent
        setBridgeState(BridgeState::OFF);
      }
    } else if (!rc_was_disarmed_once_) {
      ROS_INFO(
          "[%s] RC was disarmed once, now it is allowed to take over control",
          pnh_.getNamespace().c_str());
      rc_was_disarmed_once_ = true;
    }

    // Main mutex is unlocked here because it goes out of scope
  }

  received_sbus_msg_pub_.publish(received_sbus_msg.toRosMessage());
}


void SBusBridge::sendSBusMessageToSerialPort(const SBusMsg& sbus_msg) {
  SBusMsg sbus_message_to_send = sbus_msg;

  switch (bridge_state_) {
    case BridgeState::OFF:
      // Disarm vehicle
      sbus_message_to_send.setArmStateDisarmed();
      break;

    case BridgeState::ARMING:
      // Since there might be some RC commands smoothing and checks on multiple
      // messages before arming, we repeat the messages with minimum throttle
      // and arming command multiple times. 5 times seems to work robustly.
      if (arming_counter_ >= kSmoothingFailRepetitions_) {
        setBridgeState(BridgeState::AUTONOMOUS_FLIGHT);
      } else {
        // Set thrust to minimum command to make sure FC arms
        sbus_message_to_send.setThrottleCommand(SBusMsg::kMinCmd);
        sbus_message_to_send.setArmStateArmed();
        arming_counter_++;
      }
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      sbus_message_to_send.setArmStateArmed();
      break;

    case BridgeState::RC_FLIGHT:
      // Passing RC command straight through
      break;

    default:
      // Disarm the vehicle because this code must be terribly wrong
      sbus_message_to_send.setArmStateDisarmed();
      ROS_WARN("[%s] Bridge is in unknown state, vehicle will be disarmed",
               pnh_.getNamespace().c_str());
      break;
  }

  if ((ros::Time::now() - time_last_sbus_msg_sent_).toSec() <= 0.006) {
    // An SBUS message takes 3ms to be transmitted by the serial port so let's
    // not stress it too much. This should only happen in case of switching
    // between control commands and rc commands
    if (bridge_state_ == BridgeState::ARMING) {
      // In case of arming we want to send kSmoothingFailRepetitions_ messages
      // with minimum throttle to the flight controller. Since this check
      // prevents the message from being sent out we reduce the counter that
      // was incremented above assuming the message would actually be sent.
      arming_counter_--;
    }
    return;
  }

  sbus_message_to_send.timestamp = ros::Time::now();
  transmitSerialSBusMessage(sbus_message_to_send);
  time_last_sbus_msg_sent_ = ros::Time::now();
}

void SBusBridge::setBridgeState(const BridgeState& desired_bridge_state) {
  switch (desired_bridge_state) {
    case BridgeState::OFF:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::ARMING:
      bridge_state_ = desired_bridge_state;
      arming_counter_ = 0;
      break;

    case BridgeState::AUTONOMOUS_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    case BridgeState::RC_FLIGHT:
      bridge_state_ = desired_bridge_state;
      break;

    default:
      ROS_WARN("[%s] Wanted to switch to unknown bridge state",
               pnh_.getNamespace().c_str());
  }
}

void SBusBridge::armBridgeCallback(const std_msgs::Bool::ConstPtr& msg) {
  std::lock_guard<std::mutex> main_lock(main_mutex_);

  if (destructor_invoked_) {
    // On shut down we do not allow to arm (or disarm) the bridge anymore
    return;
  }

  if (msg->data) {
    bridge_armed_ = true;
    ROS_INFO("[%s] Bridge armed", pnh_.getNamespace().c_str());
  } else {
    bridge_armed_ = false;
    if (bridge_state_ == BridgeState::ARMING ||
        bridge_state_ == BridgeState::AUTONOMOUS_FLIGHT) {
      setBridgeState(BridgeState::OFF);
    }
    ROS_INFO("[%s] Bridge disarmed", pnh_.getNamespace().c_str());
  }
}


bool SBusBridge::loadParameters() {

  if (!nh_.hasParam("port_name")) {
       ROS_INFO("[%s] Missing configuration port_name", pnh_.getNamespace().c_str());
  }
  nh_.getParam("port_name",port_name_);
  nh_.getParam("enable_receiving_sbus_messages",enable_receiving_sbus_messages_);
  nh_.getParam("rc_timeout",rc_timeout_);
    
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
