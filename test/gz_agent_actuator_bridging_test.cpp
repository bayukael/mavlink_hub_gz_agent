#include "HilActuatorConverter.h"

#include <gtest/gtest.h>
#include <mavlink/common/mavlink.h>

#include <cmath>

namespace pendarlab::app::mavlink_hub::plugin
{
  namespace
  {
    mavlink_message_t encodeActuatorControls(const float (&controls)[16])
    {
      mavlink_message_t msg;
      mavlink_msg_hil_actuator_controls_pack(1, 200, &msg, 0, controls, 0, 0);
      return msg;
    }
  } // namespace

  TEST(HilActuatorConverter, ConvertsAllSixteenControls)
  {
    const float controls[16] = {
        1.0f, -1.0f, 0.5f, -0.5f, 0.25f, 0.0f, 0.75f, -0.75f,
        0.125f, -0.125f, 0.9f, -0.9f, 0.1f, -0.1f, 1.0f, -1.0f,
    };
    auto msg = encodeActuatorControls(controls);

    gz::msgs::Double_V out;
    ASSERT_EQ(tryConvertHilActuatorControlsToDoubleV(msg, out), true);
    ASSERT_EQ(out.data_size(), 16);
    for (int i = 0; i < 16; i++) {
      EXPECT_FLOAT_EQ(static_cast<float>(out.data(i)), controls[i]);
    }
  }

  TEST(HilActuatorConverter, HandlesAllZeroControls)
  {
    const float controls[16] = {0.0f};
    auto msg = encodeActuatorControls(controls);

    gz::msgs::Double_V out;
    ASSERT_EQ(tryConvertHilActuatorControlsToDoubleV(msg, out), true);
    ASSERT_EQ(out.data_size(), 16);
    for (int i = 0; i < 16; i++) {
      EXPECT_EQ(out.data(i), 0.0);
    }
  }

  TEST(HilActuatorConverter, IgnoresNonActuatorControlMessage)
  {
    mavlink_message_t msg;
    const uint8_t some_other_id = 0;
    msg.msgid = some_other_id;

    gz::msgs::Double_V out;
    out.add_data(123.0);

    EXPECT_EQ(tryConvertHilActuatorControlsToDoubleV(msg, out), false);
    // out left untouched (no data added, existing data preserved).
    ASSERT_EQ(out.data_size(), 1);
    EXPECT_EQ(out.data(0), 123.0);
  }
} // namespace pendarlab::app::mavlink_hub::plugin
