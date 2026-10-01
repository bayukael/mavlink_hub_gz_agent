#include "GzSensorConverter.h"

#include <gtest/gtest.h>
#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <mavlink/common/mavlink.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace pendarlab::sdk::mavlink_hub
{
  namespace
  {
    constexpr std::uint16_t kAccel = 0b111;
    constexpr std::uint16_t kGyro = 0b111000;
    constexpr std::uint16_t kMag = 0b111000000;
    constexpr std::uint16_t kBaro = 0b1101000000000;
    constexpr std::uint16_t kDiffPress = 0b1010000000000;
  } // namespace

  class GzSensorConverterTest : public testing::Test
  {
  protected:
    GzAgentConfig makeConfig()
    {
      GzAgentConfig cfg;
      cfg.vehicle_name = "v";
      cfg.world_name = "w";
      return cfg;
    }

    gz::msgs::IMU makeImu(double ax, double ay, double az, double gx, double gy, double gz)
    {
      gz::msgs::IMU imu;
      imu.mutable_linear_acceleration()->set_x(ax);
      imu.mutable_linear_acceleration()->set_y(ay);
      imu.mutable_linear_acceleration()->set_z(az);
      imu.mutable_angular_velocity()->set_x(gx);
      imu.mutable_angular_velocity()->set_y(gy);
      imu.mutable_angular_velocity()->set_z(gz);
      imu.mutable_header()->mutable_stamp()->set_sec(1);
      imu.mutable_header()->mutable_stamp()->set_nsec(500000000);
      return imu;
    }

    gz::msgs::FluidPressure makeBaro(double pressure_pa)
    {
      gz::msgs::FluidPressure baro;
      baro.set_pressure(pressure_pa);
      baro.mutable_header()->mutable_stamp()->set_sec(2);
      baro.mutable_header()->mutable_stamp()->set_nsec(0);
      return baro;
    }

    gz::msgs::Magnetometer makeMag(double x, double y, double z)
    {
      gz::msgs::Magnetometer mag;
      mag.mutable_field_tesla()->set_x(x);
      mag.mutable_field_tesla()->set_y(y);
      mag.mutable_field_tesla()->set_z(z);
      mag.mutable_header()->mutable_stamp()->set_sec(3);
      mag.mutable_header()->mutable_stamp()->set_nsec(0);
      return mag;
    }

    gz::msgs::Pose makePose(double qw, double qx, double qy, double qz)
    {
      gz::msgs::Pose pose;
      pose.mutable_orientation()->set_w(qw);
      pose.mutable_orientation()->set_x(qx);
      pose.mutable_orientation()->set_y(qy);
      pose.mutable_orientation()->set_z(qz);
      return pose;
    }

    gz::msgs::NavSat makeNavSat(double lat, double lon, double alt,
                                double vn, double ve, double vu)
    {
      gz::msgs::NavSat navsat;
      navsat.set_latitude_deg(lat);
      navsat.set_longitude_deg(lon);
      navsat.set_altitude(alt);
      navsat.set_velocity_north(vn);
      navsat.set_velocity_east(ve);
      navsat.set_velocity_up(vu);
      navsat.mutable_header()->mutable_stamp()->set_sec(4);
      navsat.mutable_header()->mutable_stamp()->set_nsec(0);
      return navsat;
    }

    mavlink_hil_sensor_t findSensor(const std::vector<mavlink_message_t>& msgs)
    {
      mavlink_hil_sensor_t sensor{};
      for (const auto& m : msgs) {
        if (m.msgid == MAVLINK_MSG_ID_HIL_SENSOR) {
          mavlink_msg_hil_sensor_decode(&m, &sensor);
        }
      }
      return sensor;
    }

    mavlink_hil_gps_t findGps(const std::vector<mavlink_message_t>& msgs)
    {
      mavlink_hil_gps_t gps{};
      for (const auto& m : msgs) {
        if (m.msgid == MAVLINK_MSG_ID_HIL_GPS) {
          mavlink_msg_hil_gps_decode(&m, &gps);
        }
      }
      return gps;
    }

    bool hasMsgid(const std::vector<mavlink_message_t>& msgs, std::uint32_t id)
    {
      for (const auto& m : msgs) {
        if (m.msgid == id) {
          return true;
        }
      }
      return false;
    }

    GzSensorConverter converter{makeConfig()};
  };

  TEST_F(GzSensorConverterTest, ImuProducesFrdAccelGyroAndBits)
  {
    gz::msgs::IMU imu = makeImu(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    SensorSnapshot snap;
    snap.imu = &imu;
    snap.imu_updated = true;

    auto msgs = converter.convert(snap);
    ASSERT_EQ(msgs.size(), 1u);
    EXPECT_EQ(msgs[0].sysid, 1);
    EXPECT_EQ(msgs[0].compid, 200);

    auto sensor = findSensor(msgs);
    EXPECT_FLOAT_EQ(sensor.xacc, 1.0f);
    EXPECT_FLOAT_EQ(sensor.yacc, -2.0f);
    EXPECT_FLOAT_EQ(sensor.zacc, -3.0f);
    EXPECT_FLOAT_EQ(sensor.xgyro, 0.1f);
    EXPECT_FLOAT_EQ(sensor.ygyro, -0.2f);
    EXPECT_FLOAT_EQ(sensor.zgyro, -0.3f);
    EXPECT_EQ(sensor.time_usec, 1500000u);
    EXPECT_EQ(sensor.fields_updated & (kAccel | kGyro), kAccel | kGyro);
  }

  TEST_F(GzSensorConverterTest, BaroAddsPressureHpaAndTemperature)
  {
    gz::msgs::FluidPressure baro = makeBaro(101325.0);
    SensorSnapshot snap;
    snap.baro = &baro;
    snap.baro_updated = true;

    auto msgs = converter.convert(snap);
    auto sensor = findSensor(msgs);
    EXPECT_FLOAT_EQ(sensor.abs_pressure, 1013.25f);
    EXPECT_FLOAT_EQ(sensor.temperature, 288.15f);
    EXPECT_NE(sensor.fields_updated & kBaro, 0u);
  }

  TEST_F(GzSensorConverterTest, MagDefaultTeslaToGaussNedFrame)
  {
    gz::msgs::Magnetometer mag = makeMag(0.0003, 0.0004, 0.0005);
    SensorSnapshot snap;
    snap.mag = &mag;
    snap.mag_updated = true;

    auto msgs = converter.convert(snap);
    auto sensor = findSensor(msgs);
    // tesla -> gauss factor 1e4, frame ned: xmag=-y, ymag=-x, zmag=z.
    EXPECT_FLOAT_EQ(sensor.xmag, -0.0004f * 1e4f);
    EXPECT_FLOAT_EQ(sensor.ymag, -0.0003f * 1e4f);
    EXPECT_FLOAT_EQ(sensor.zmag, 0.0005f * 1e4f);
    EXPECT_NE(sensor.fields_updated & kMag, 0u);
  }

  TEST_F(GzSensorConverterTest, MagGaussUnitSkipsScaling)
  {
    auto cfg = makeConfig();
    cfg.gz_magnetometer_unit = "gauss";
    GzSensorConverter local_converter(cfg);
    gz::msgs::Magnetometer mag = makeMag(0.3, 0.4, 0.5);
    SensorSnapshot snap;
    snap.mag = &mag;
    snap.mag_updated = true;

    auto msgs = local_converter.convert(snap);
    auto sensor = findSensor(msgs);
    EXPECT_FLOAT_EQ(sensor.xmag, -0.4f);
    EXPECT_FLOAT_EQ(sensor.ymag, -0.3f);
    EXPECT_FLOAT_EQ(sensor.zmag, 0.5f);
  }

  TEST_F(GzSensorConverterTest, MagEnuFrame)
  {
    auto cfg = makeConfig();
    cfg.gz_magnetometer_frame = "enu";
    cfg.gz_magnetometer_unit = "gauss";
    GzSensorConverter local_converter(cfg);
    gz::msgs::Magnetometer mag = makeMag(0.3, 0.4, 0.5);
    SensorSnapshot snap;
    snap.mag = &mag;
    snap.mag_updated = true;

    auto msgs = local_converter.convert(snap);
    auto sensor = findSensor(msgs);
    EXPECT_FLOAT_EQ(sensor.xmag, 0.3f);
    EXPECT_FLOAT_EQ(sensor.ymag, -0.4f);
    EXPECT_FLOAT_EQ(sensor.zmag, -0.5f);
  }

  TEST_F(GzSensorConverterTest, AirspeedAndGpsFromNavsat)
  {
    gz::msgs::IMU imu = makeImu(1, 2, 3, 0.1, 0.2, 0.3);
    gz::msgs::Pose pose = makePose(1.0, 0.0, 0.0, 0.0);
    gz::msgs::NavSat navsat = makeNavSat(-33.5, 151.2, 100.0, 10.0, 5.0, 0.0);
    SensorSnapshot snap;
    snap.imu = &imu;
    snap.imu_updated = true;
    snap.pose = &pose;
    snap.pose_updated = true;
    snap.navsat = &navsat;
    snap.navsat_updated = true;

    auto msgs = converter.convert(snap);
    ASSERT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_GPS), true);
    ASSERT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_SENSOR), true);

    auto gps = findGps(msgs);
    EXPECT_EQ(gps.lat, static_cast<std::int32_t>(-33.5 * 1e7));
    EXPECT_EQ(gps.lon, static_cast<std::int32_t>(151.2 * 1e7));
    EXPECT_EQ(gps.alt, static_cast<std::int32_t>(100.0 * 1e3));
    EXPECT_EQ(gps.fix_type, 3);
    EXPECT_EQ(gps.satellites_visible, 20);
    EXPECT_EQ(gps.eph, 100);
    EXPECT_EQ(gps.epv, 100);
    EXPECT_EQ(gps.vn, static_cast<std::int16_t>(10.0 * 100.0));
    EXPECT_EQ(gps.ve, static_cast<std::int16_t>(5.0 * 100.0));
    EXPECT_EQ(gps.vd, static_cast<std::int16_t>(0.0));
    EXPECT_EQ(gps.vel, static_cast<std::uint16_t>(std::hypot(5.0, 10.0) * 100.0));
    EXPECT_EQ(gps.cog, static_cast<std::uint16_t>((std::atan2(5.0, 10.0) / M_PI) * 18000.0));

    auto sensor = findSensor(msgs);
    // With identity pose the body frame equals NED, so vx = vn = 10.
    EXPECT_NE(sensor.fields_updated & kDiffPress, 0u);
    EXPECT_GT(sensor.diff_pressure, 0.0f);
  }

  TEST_F(GzSensorConverterTest, AirspeedDiffPressureDeterministicNoNoise)
  {
    gz::msgs::IMU imu = makeImu(1, 2, 3, 0.1, 0.2, 0.3);
    gz::msgs::Pose pose = makePose(1.0, 0.0, 0.0, 0.0);
    gz::msgs::NavSat navsat = makeNavSat(0.0, 0.0, 0.0, 20.0, 0.0, 0.0);
    SensorSnapshot snap;
    snap.imu = &imu;
    snap.imu_updated = true;
    snap.pose = &pose;
    snap.pose_updated = true;
    snap.navsat = &navsat;
    snap.navsat_updated = true;

    auto msgs1 = converter.convert(snap);
    auto msgs2 = converter.convert(snap);
    EXPECT_EQ(findSensor(msgs1).diff_pressure, findSensor(msgs2).diff_pressure);

    // Reproduce the reference attitude rotation to derive vx (body frame), then the
    // exact no-noise diff_pressure formula.
    const gz::math::Vector3d vel_ned(20.0, 0.0, 0.0);
    const gz::math::Quaterniond q_gr(1.0, 0.0, 0.0, 0.0);
    const gz::math::Quaterniond q_flu_to_frd(0, 1, 0, 0);
    const gz::math::Quaterniond q_enu_to_ned(0, 0.70711, 0.70711, 0);
    const gz::math::Quaterniond q_nb = q_enu_to_ned * q_gr * q_flu_to_frd.Inverse();
    const gz::math::Matrix3d vehicle_attitude(q_nb);
    const double vx = (vehicle_attitude.Transposed() * vel_ned).X();

    const double alt = 0.0;
    const double temperature_local = 288.15 - 0.0065 * alt;
    const double density_ratio = std::pow(288.15 / temperature_local, 4.256);
    const double air_density = 1.225 / density_ratio;
    const double expected = ((vx > 0) - (vx < 0)) * 0.005 * air_density * vx * vx;
    EXPECT_NEAR(static_cast<double>(findSensor(msgs1).diff_pressure), expected, 1e-4);
  }

  TEST_F(GzSensorConverterTest, GatingNoFreshDataReturnsEmpty)
  {
    gz::msgs::IMU imu = makeImu(1, 2, 3, 0.1, 0.2, 0.3);
    SensorSnapshot snap;
    snap.imu = &imu; // pointer set but not updated

    auto msgs = converter.convert(snap);
    EXPECT_EQ(msgs.empty(), true);
  }

  TEST_F(GzSensorConverterTest, ImuOnlyEmitsSensorNotGps)
  {
    gz::msgs::IMU imu = makeImu(1, 2, 3, 0.1, 0.2, 0.3);
    SensorSnapshot snap;
    snap.imu = &imu;
    snap.imu_updated = true;

    auto msgs = converter.convert(snap);
    EXPECT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_SENSOR), true);
    EXPECT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_GPS), false);
  }

  TEST_F(GzSensorConverterTest, NavsatOnlyEmitsGpsNotSensor)
  {
    gz::msgs::Pose pose = makePose(1.0, 0.0, 0.0, 0.0);
    gz::msgs::NavSat navsat = makeNavSat(0.0, 0.0, 50.0, 5.0, 0.0, 0.0);
    SensorSnapshot snap;
    snap.pose = &pose;
    snap.pose_updated = true;
    snap.navsat = &navsat;
    snap.navsat_updated = true;

    auto msgs = converter.convert(snap);
    EXPECT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_GPS), true);
    EXPECT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_SENSOR), false);
  }

  TEST_F(GzSensorConverterTest, CombinedSnapshotProducesBothMessages)
  {
    gz::msgs::IMU imu = makeImu(1, 2, 3, 0.1, 0.2, 0.3);
    gz::msgs::FluidPressure baro = makeBaro(101325.0);
    gz::msgs::Magnetometer mag = makeMag(0.0003, 0.0004, 0.0005);
    gz::msgs::Pose pose = makePose(1.0, 0.0, 0.0, 0.0);
    gz::msgs::NavSat navsat = makeNavSat(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    SensorSnapshot snap;
    snap.imu = &imu;
    snap.imu_updated = true;
    snap.baro = &baro;
    snap.baro_updated = true;
    snap.mag = &mag;
    snap.mag_updated = true;
    snap.pose = &pose;
    snap.pose_updated = true;
    snap.navsat = &navsat;
    snap.navsat_updated = true;

    auto msgs = converter.convert(snap);
    EXPECT_EQ(msgs.size(), 2u);
    EXPECT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_SENSOR), true);
    EXPECT_EQ(hasMsgid(msgs, MAVLINK_MSG_ID_HIL_GPS), true);

    auto sensor = findSensor(msgs);
    EXPECT_NE(sensor.fields_updated & kAccel, 0u);
    EXPECT_NE(sensor.fields_updated & kGyro, 0u);
    EXPECT_NE(sensor.fields_updated & kMag, 0u);
    EXPECT_NE(sensor.fields_updated & kBaro, 0u);
    EXPECT_NE(sensor.fields_updated & kDiffPress, 0u);
  }

  TEST_F(GzSensorConverterTest, AirspeedWithoutPoseUsesZeroBodyVelocity)
  {
    gz::msgs::IMU imu = makeImu(1, 2, 3, 0.1, 0.2, 0.3);
    gz::msgs::NavSat navsat = makeNavSat(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    SensorSnapshot snap;
    snap.imu = &imu;
    snap.imu_updated = true;
    snap.navsat = &navsat;
    snap.navsat_updated = true;

    auto msgs = converter.convert(snap);
    auto sensor = findSensor(msgs);
    // No pose -> body velocity stays zero -> diff_pressure == 0.
    EXPECT_FLOAT_EQ(sensor.diff_pressure, 0.0f);
    EXPECT_NE(sensor.fields_updated & kDiffPress, 0u);
  }
} // namespace pendarlab::sdk::mavlink_hub
