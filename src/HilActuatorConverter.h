#pragma once

#include <gz/msgs/double_v.pb.h>
#include <mavlink/common/mavlink.h>

namespace pendarlab::sdk::mavlink_hub
{
  // Number of actuator control channels in HIL_ACTUATOR_CONTROLS.
  constexpr int kHilActuatorControlCount = 16;

  // Decode a HIL_ACTUATOR_CONTROLS mavlink message into a gz::msgs::Double_V.
  // Pure conversion, independent of any live endpoint or transport, so it can
  // be unit-tested in isolation. All kHilActuatorControlCount channels are
  // copied (float -> double). Returns true if the message was a
  // HIL_ACTUATOR_CONTROLS and `out` was populated; returns false (leaving `out`
  // untouched) for any other message id, so callers know whether to publish.
  inline bool tryConvertHilActuatorControlsToDoubleV(const mavlink_message_t& msg, gz::msgs::Double_V& out)
  {
    if (msg.msgid != MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
      return false;
    }
    mavlink_hil_actuator_controls_t decoded;
    mavlink_msg_hil_actuator_controls_decode(&msg, &decoded);
    for (int i = 0; i < kHilActuatorControlCount; i++) {
      out.add_data(static_cast<double>(decoded.controls[i]));
    }
    return true;
  }
} // namespace pendarlab::sdk::mavlink_hub
