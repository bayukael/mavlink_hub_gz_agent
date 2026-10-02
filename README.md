# mavlink_hub_gz_agent

A plugin for [mavlink_hub](https://github.com/bayukael/mavlink_hub) that acts as a bidirectional bridge between an autopilot software that speaks Mavlink and Gazebo Simulator.

The agent is loaded by `mavlink_hub` as a runtime plugin (`.so`). It exposes a single `GzAgent` agent that:

- subscribes to Gazebo sensor topics (IMU, barometer, magnetometer, NavSat, pose),
- converts fresh sensor data into Mavlink `HIL_SENSOR` and `HIL_GPS` messages and sends them to the autopilot through a Mavlink endpoint, and
- receives Mavlink `HIL_ACTUATOR_CONTROLS` messages from the autopilot and publishes them to Gazebo as `gz::msgs::Double_V` actuator commands.

## Overview / purpose

`mavlink_hub_gz_agent` bridges the gap between a real autopilot software (which communicates over Mavlink) and a simulated vehicle running in Gazebo Simulator. It lets the autopilot believe it is controlling a real vehicle while in fact its commands are applied to a simulated model, and the simulated model's sensors are fed back to it.

The bridge is bidirectional:

- **Gazebo → Autopilot:** Gazebo's simulated sensor data is converted to Mavlink `HIL_SENSOR` / `HIL_GPS` messages and written to the autopilot over a Mavlink endpoint.
- **Autopilot → Gazebo:** The autopilot's `HIL_ACTUATOR_CONTROLS` Mavlink messages are converted to `gz::msgs::Double_V` and published on a Gazebo topic that the simulated vehicle's actuator plugin consumes.

## Data flow

### Sensor data: Gazebo Simulator -> Autopilot Software

Gazebo Simulator provides sensor messages:

- imu
- baro
- magnetometer
- navsat
- pose

`GzAgent` converts these into `HIL_GPS` and `HIL_SENSOR` and sends them to the autopilot so that it knows the state of the vehicle it tries to control.

Sensor sending runs on a background thread at the configured `send_freq` (in Hz). Each tick, the agent snapshots the latest sensor data, and only sends if there is new data:

- A `HIL_SENSOR` message is emitted only when IMU, barometer, and magnetometer have all been updated since the last tick.
- A `HIL_GPS` message is emitted only when the NavSat has been updated since the last tick.
- If nothing is new, nothing is sent.

The conversion is deterministic (no noise / no RNG): FLU→FRD for accel/gyro, barometer pressure + temperature, magnetometer unit/frame handling, and airspeed derived from NavSat velocity and altitude via the `gz::math` attitude rotation.

### Actuator data: Autopilot Software -> Gazebo Simulator

The Autopilot Software provides `HIL_ACTUATOR` mavlink messages, which `GzAgent` converts into `gz::msgs::Double_V` and publishes via `gz_actuator_pub`, so that Gazebo knows what commands the autopilot tries to give to the simulated vehicle.

When the autopilot sends a `HIL_ACTUATOR_CONTROLS` (id 93) message, the agent decodes all 16 channels and publishes them as a `gz::msgs::Double_V` on the configured `actuator_topic`. Non-`HIL_ACTUATOR_CONTROLS` messages are ignored.

## Communication

- To communicate with Gazebo, `GzAgent` uses the `gz::transport` system, which follows a publish/subscribe pattern.
- To communicate with the Autopilot Software, `GzAgent` uses `pendarlab::sdk::mavlink_hub::IMavlinkEndpointUser`, which provides `writeMessage()` and `createListener()`.

`pendarlab::sdk::mavlink_hub::Agent` is designed such that it can obtain an `IMavlinkEndpointUser` by making a request through `IManagerResourceRequester`, which is provided by the mavlink_hub app when it calls `GzAgentDefinition::create()`.

## Building and installing

### Prerequisites

- CMake (>= 3.13) and a C++17 compiler.
- The `mavlink_hub_sdk` installed and discoverable by CMake. As long as it is installed and its prefix is on `CMAKE_PREFIX_PATH`, its transitive dependencies are resolved automatically — no additional setup is needed.
- The Gazebo packages (system install, found via `find_package`):
  - `gz-cmake3`, `gz-msgs10`, `gz-transport13`, `sdformat14`.

### Configure and build

From the project root (the directory containing this `README.md`):

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="<prefix containing your mavlink_hub_sdk install>"
cmake --build build
```

Set `<prefix containing your mavlink_hub_sdk install>` to the install prefix where `mavlink_hub_sdk` was installed (for example, the directory that contains the SDK's CMake config files). As long as `mavlink_hub_sdk` is found, everything else is pulled in automatically.

Alternatively, from inside a `build/` directory you created yourself:

```bash
cd build
cmake .. -DCMAKE_PREFIX_PATH="<prefix containing your mavlink_hub_sdk install>"
make
```

The build produces a shared library in your build directory:

```
libmavlink_hub_gz_agent.so
```

### Making the plugin available to mavlink_hub

`mavlink_hub` loads agent plugins at runtime using `dlopen`, then looks up the exported factory symbol by name. This plugin exports:

```c
extern "C" const GzAgentDefinition* getGzAgentDefinition();
```

To make `mavlink_hub` able to load the `GzAgent`, provide `mavlink_hub` with:

1. The path to the built shared library — the `libmavlink_hub_gz_agent.so` produced by the build (or a copy of it placed somewhere `mavlink_hub` can find). Use the actual path on your machine.
2. The factory symbol name `getGzAgentDefinition`.
3. A name to register the agent under (used to reference this agent definition when spawning a `GzAgent`).

When `mavlink_hub` registers the agent definition, it will spawn an instance of the agent using the config you provide (see below). The agent's `extern "C"` entry point is in `src/GzAgentDefinition.cpp`.

## Configuration

When `mavlink_hub` spawns a `GzAgent`, it provides a set of string key/value options that are parsed by `GzAgentDefinition::parseConfig()`. Invalid values are reported as parse errors (the agent is not created).

### Required fields

| Key | Description |
| --- | --- |
| `vehicle_name` | Name of the model in the Gazebo world that this agent bridges. |
| `world_name` | Name of the Gazebo world. |
| `mavlink_endpoint_name` | Name of the Mavlink endpoint (registered in `mavlink_hub`) to send/receive messages on. |

### Topic overrides

These keys override the generated topic names. When `use_topic_generator` is `true` they are optional (defaulting to the generated names); when it is `false` (or unset) **all** of them are required.

| Key | Default (generated) |
| --- | --- |
| `imu_topic` | `/world/<world_name>/model/<vehicle_name>/link/base_link/sensor/imu_sensor/imu` |
| `baro_topic` | `/world/<world_name>/model/<vehicle_name>/link/base_link/sensor/air_pressure_sensor/air_pressure` |
| `mag_topic` | `/world/<world_name>/model/<vehicle_name>/link/base_link/sensor/magnetometer_sensor/magnetometer` |
| `pose_topic` | `/model/<vehicle_name>/pose` |
| `navsat_topic` | `/world/<world_name>/model/<vehicle_name>/link/base_link/sensor/navsat_sensor/navsat` |
| `actuator_topic` | `/<vehicle_name>/command/raw_actuators` |

### Optional fields

| Key | Default | Valid values | Description |
| --- | --- | --- | --- |
| `use_topic_generator` | `false` | `true` / `1` (any other value = `false`) | When enabled, topic fields are filled from `vehicle_name` + `world_name` (still overridable per-topic). When disabled, all topic fields are required. |
| `send_freq` | `250` | positive number (Hz) | Frequency of the sensor send-loop background thread. |
| `gz_magnetometer_unit` | `tesla` | `tesla` \| `gauss` | Unit of the Gazebo magnetometer reading. |
| `gz_magnetometer_frame` | `ned` | `ned` \| `enu` | Reference frame of the Gazebo magnetometer reading. |

## Configuration examples

### Minimal configuration (required fields only)

When `use_topic_generator` is not set (i.e. `false`), all topic fields are required:

```yaml
vehicle_name: iris
world_name: default
mavlink_endpoint_name: pixhawk_serial

imu_topic: /world/default/model/iris/link/base_link/sensor/imu_sensor/imu
baro_topic: /world/default/model/iris/link/base_link/sensor/air_pressure_sensor/air_pressure
mag_topic: /world/default/model/iris/link/base_link/sensor/magnetometer_sensor/magnetometer
pose_topic: /model/iris/pose
navsat_topic: /world/default/model/iris/link/base_link/sensor/navsat_sensor/navsat
actuator_topic: /iris/command/raw_actuators
```

### Using `use_topic_generator`

Enable topic generation so only the required fields are needed:

```yaml
vehicle_name: iris
world_name: default
mavlink_endpoint_name: pixhawk_serial
use_topic_generator: true
```

Topic overrides still work in combination with the generator — any topic you specify replaces the generated value, and the rest are generated:

```yaml
vehicle_name: iris
world_name: default
mavlink_endpoint_name: pixhawk_serial
use_topic_generator: true

# Only override the actuator topic; the sensor topics are generated.
actuator_topic: /custom/raw_actuators
```

### Non-default `send_freq`

```yaml
vehicle_name: iris
world_name: default
mavlink_endpoint_name: pixhawk_serial
use_topic_generator: true
send_freq: 50
```

### Magnetometer `gauss` / `enu` combination

```yaml
vehicle_name: iris
world_name: default
mavlink_endpoint_name: pixhawk_serial
use_topic_generator: true
gz_magnetometer_unit: gauss
gz_magnetometer_frame: enu
```

The other combination (`tesla`/`ned`) is the default, so it is produced by simply omitting these keys (see the earlier examples).
