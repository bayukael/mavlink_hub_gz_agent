# mavlink_hub_gz_agent

A plugin for [mavlink_hub](https://github.com/bayukael/mavlink_hub) that acts as a bidirectional bridge between an autopilot software that speaks Mavlink and Gazebo Simulator.

## Data flow

### Sensor data: Gazebo Simulator -> Autopilot Software

Gazebo Simulator provides sensor messages:

- imu
- baro
- magnetometer
- navsat
- pose

`GzAgent` converts these into `HIL_GPS` and `HIL_SENSOR` and sends them to the autopilot so that it knows the state of the vehicle it tries to control.

### Actuator data: Autopilot Software -> Gazebo Simulator

The Autopilot Software provides `HIL_ACTUATOR` mavlink messages, which `GzAgent` converts into `gz::msgs::Double_V` and publishes via `gz_actuator_pub`, so that Gazebo knows what commands the autopilot tries to give to the simulated vehicle.

## Communication

- To communicate with Gazebo, `GzAgent` uses the `gz::transport` system, which follows a publish/subscribe pattern.
- To communicate with the Autopilot Software, `GzAgent` uses `pendarlab::sdk::mavlink_hub::IMavlinkEndpointUser`, which provides `writeMessage()` and `createListener()`.

`pendarlab::sdk::mavlink_hub::Agent` is designed such that it can obtain an `IMavlinkEndpointUser` by making a request through `IManagerResourceRequester`, which is provided by the mavlink_hub app when it calls `GzAgentDefinition::create()`.
