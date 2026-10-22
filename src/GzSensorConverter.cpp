#include "GzSensorConverter.h"

#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <cmath>
#include <cstdint>

namespace pendarlab::app::mavlink_hub::plugin
{
  namespace
  {
    // The installed mavlink headers do not define the HIL_SENSOR_SOURCE_* bitmask
    // constants, so define them locally. Values match the reference
    // SensorSource enum (references/mavlink_sim_vehicle/sim_vehicle.hh).
    constexpr std::uint16_t kSourceAccel = 0b111;             // bits 0-2
    constexpr std::uint16_t kSourceGyro = 0b111000;           // bits 3-5
    constexpr std::uint16_t kSourceMag = 0b111000000;         // bits 6-8
    constexpr std::uint16_t kSourceBaro = 0b1101000000000;    // bits 9, 11, 12
    constexpr std::uint16_t kSourceDiffPress = 0b1010000000000; // bits 10, 12

    constexpr std::uint8_t kSysId = 1;
    constexpr std::uint8_t kCompId = 200;

    // Airspeed constants (from references/mavlink_sim_vehicle/gz_airspeed.hh/cc).
    constexpr float kTemperatureMsl = 288.15f;
    constexpr float kLapseRate = 0.0065f;
    constexpr float kAirDensityMsl = 1.225f;

    constexpr double kPi = 3.14159265358979323846;

    /// Converts a gz::msgs Header timestamp to microseconds since epoch. @param header the
    /// message header. @return the stamp in microseconds.
    std::uint64_t stampToUsec(const gz::msgs::Header& header)
    {
      const auto& stamp = header.stamp();
      return static_cast<std::uint64_t>(stamp.sec()) * 1000000ULL +
             static_cast<std::uint64_t>(stamp.nsec()) / 1000ULL;
    }
  } // namespace

  struct GzSensorConverter::Impl
  {
    std::string mag_unit = "tesla";
    std::string mag_frame = "ned";
  };

  /// Constructs the converter, storing the magnetometer unit/frame from the config.
  /// @param cfg the agent config.
  GzSensorConverter::GzSensorConverter(const GzAgentConfig& cfg)
      : d(std::make_unique<Impl>())
  {
    d->mag_unit = cfg.gz_magnetometer_unit;
    d->mag_frame = cfg.gz_magnetometer_frame;
  }

  /// Destroys the converter.
  GzSensorConverter::~GzSensorConverter() = default;

  /// Converts a snapshot into sendable HIL_SENSOR / HIL_GPS messages. Emits HIL_SENSOR
  /// only if imu/baro/mag updated, HIL_GPS only if navsat updated. @param snap the latest
  /// sensor data and update flags. @return the messages to send (empty if nothing new).
  std::vector<mavlink_message_t> GzSensorConverter::convert(const SensorSnapshot& snap)
  {
    std::vector<mavlink_message_t> result;

    const bool sensor_updated = snap.imu_updated || snap.baro_updated || snap.mag_updated;
    const bool navsat_updated = snap.navsat_updated;

    if (sensor_updated || navsat_updated) {
      mavlink_hil_sensor_t sensor{};
      sensor.time_usec = 0;
      sensor.fields_updated = 0;

      if (snap.imu_updated && snap.imu != nullptr) {
        const auto& accel = snap.imu->linear_acceleration();
        const auto& gyro = snap.imu->angular_velocity();
        sensor.time_usec = stampToUsec(snap.imu->header());
        // IMU is FLU, HIL_SENSOR is FRD.
        sensor.xacc = static_cast<float>(accel.x());
        sensor.yacc = static_cast<float>(-accel.y());
        sensor.zacc = static_cast<float>(-accel.z());
        sensor.xgyro = static_cast<float>(gyro.x());
        sensor.ygyro = static_cast<float>(-gyro.y());
        sensor.zgyro = static_cast<float>(-gyro.z());
        sensor.fields_updated |= kSourceAccel | kSourceGyro;
      }

      if (snap.baro_updated && snap.baro != nullptr) {
        sensor.abs_pressure = static_cast<float>(snap.baro->pressure() / 100.0);
        sensor.temperature = 288.15f;
        sensor.fields_updated |= kSourceBaro;
      }

      if (snap.mag_updated && snap.mag != nullptr) {
        const double factor = (d->mag_unit == "gauss") ? 1.0 : 1e4;
        const auto& ft = snap.mag->field_tesla();
        const double gz_x = ft.x() * factor;
        const double gz_y = ft.y() * factor;
        const double gz_z = ft.z() * factor;
        if (d->mag_frame == "enu") {
          sensor.xmag = static_cast<float>(gz_x);
          sensor.ymag = static_cast<float>(-gz_y);
          sensor.zmag = static_cast<float>(-gz_z);
        } else { // ned (default)
          sensor.xmag = static_cast<float>(-gz_y);
          sensor.ymag = static_cast<float>(-gz_x);
          sensor.zmag = static_cast<float>(gz_z);
        }
        sensor.fields_updated |= kSourceMag;
      }

      if (navsat_updated && snap.navsat != nullptr) {
        const auto& ns = *snap.navsat;

        // HIL_GPS from navsat.
        mavlink_hil_gps_t gps{};
        gps.time_usec = stampToUsec(ns.header());
        gps.lat = static_cast<std::int32_t>(ns.latitude_deg() * 1e7);
        gps.lon = static_cast<std::int32_t>(ns.longitude_deg() * 1e7);
        gps.alt = static_cast<std::int32_t>(ns.altitude() * 1e3);
        gps.fix_type = 3;
        gps.satellites_visible = 20;
        gps.eph = 100;
        gps.epv = 100;
        const double velocity = std::hypot(ns.velocity_east(), ns.velocity_north());
        gps.vel = static_cast<std::uint16_t>(velocity * 100.0);
        gps.vn = static_cast<std::int16_t>(ns.velocity_north() * 100.0);
        gps.ve = static_cast<std::int16_t>(ns.velocity_east() * 100.0);
        gps.vd = static_cast<std::int16_t>(-ns.velocity_up() * 100.0);
        double cog = std::atan2(ns.velocity_east(), ns.velocity_north());
        if (cog < 0.0) {
          cog += 2.0 * kPi;
        }
        gps.cog = static_cast<std::uint16_t>((cog / kPi) * 18000.0);

        mavlink_message_t gps_msg;
        mavlink_msg_hil_gps_encode(kSysId, kCompId, &gps_msg, &gps);
        result.push_back(gps_msg);

        // Airspeed from navsat (velocity + altitude), attitude from latest pose.
        const gz::math::Vector3d vel_ned(ns.velocity_north(), ns.velocity_east(), -ns.velocity_up());
        const double alt = ns.altitude();
        const float temperature_local = kTemperatureMsl - kLapseRate * static_cast<float>(alt);
        const float density_ratio = std::pow(kTemperatureMsl / temperature_local, 4.256f);
        const float air_density = kAirDensityMsl / density_ratio;

        gz::math::Vector3d vel_in_body(0.0, 0.0, 0.0);
        if (snap.pose != nullptr) {
          const auto& o = snap.pose->orientation();
          const gz::math::Quaterniond q_gr(o.w(), o.x(), o.y(), o.z());
          static const gz::math::Quaterniond q_flu_to_frd(0, 1, 0, 0);
          static const gz::math::Quaterniond q_enu_to_ned(0, 0.70711, 0.70711, 0);
          const gz::math::Quaterniond q_nb = q_enu_to_ned * q_gr * q_flu_to_frd.Inverse();
          const gz::math::Matrix3d vehicle_attitude(q_nb);
          vel_in_body = vehicle_attitude.Transposed() * vel_ned;
        }
        const double vx = vel_in_body.X();
        sensor.diff_pressure =
            static_cast<float>(((vx > 0.0) - (vx < 0.0)) * 0.005 * air_density * vx * vx);
        sensor.temperature = temperature_local;
        sensor.fields_updated |= kSourceDiffPress;
      }

      if (sensor_updated) {
        mavlink_message_t sensor_msg;
        mavlink_msg_hil_sensor_encode(kSysId, kCompId, &sensor_msg, &sensor);
        result.insert(result.begin(), sensor_msg);
      }
    }

    return result;
  }
} // namespace pendarlab::app::mavlink_hub::plugin
