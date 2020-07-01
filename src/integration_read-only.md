# Read-only Fleet Integration

In this section, we will cover the prototype API for integrating the **Read Only** category of mobile robot fleets. This means we assume the mobile robot fleet manager only allows RMF to see updates about where its robots are located and where they intend to go, but it does not offer any control over where the robots are going or how they can move around. This type of adapter is primarily aimed at legacy systems that were developed before RMF and did not anticipate the possibility of a third-party being able to command the robots.

## Fleet Driver API

The Fleet Driver API was an experimental API developed in the early stages of the RMF research project. It can still be used for a read-only fleet adapter implementation until an officially supported C++ API comes out to replace it.

The Fleet Driver API uses ROS 2 messages from the [`rmf_fleet_msgs`](https://github.com/osrf/rmf_core/tree/master/rmf_fleet_msgs) package. To use this API, you will want to write a ROS 2 application (using either rclcpp or rclpy) which we will refer to as the *Fleet Driver*. The job of the Fleet Driver is to transmit [`rmf_fleet_msgs/FleetState`](https://github.com/osrf/rmf_core/blob/master/rmf_fleet_msgs/msg/FleetState.msg) messages out to the `fleet_states` topic.

Inside the `FleetState` message is the `name` field. Be sure to fill in the correct name for your fleet state. There is also a collection of [`rmf_fleet_msgs/RobotState`](https://github.com/osrf/rmf_core/blob/master/rmf_fleet_msgs/msg/RobotState.msg) messages. For integrating a read-only fleet with RMF, the most crucial fields of the `RobotState` message are:

* `name` - The name of the robot whose state is being specified.
* `location` - The current location of the robot.
* `path` - The sequence of locations that the robot will be traveling through.

Inside the [`rmf_fleet_msgs/Location`](https://github.com/osrf/rmf_core/blob/master/rmf_fleet_msgs/msg/Location.msg) message, the `t` field (which represents time) is generally ignored by the read-only fleet adapter. We assume that it is too cumbersome for your Fleet Driver to make timing predictions, so we have the read-only fleet adapter make the predictions for you based on the traits of the vehicle.

## Configuring the Read Only Fleet Adapter

For the prototype read-only integration, there are two applications that need to be launched:

1. The Fleet Driver mentioned above which you write specifically for your fleet's custom API
2. The `read_only` fleet adapter which must be launched through ROS 2

To launch the fleet adapter, you will need to use `ros2 launch` and include `rmf_fleet_adapter/fleet_adapter.launch.xml` file with the required parameters filled in. An example of this using the XML front-end of ros2 launch [can be found in `rmf_demos`](https://github.com/osrf/rmf_demos/blob/master/demos/launch/include/adapters/caddy_adapter.launch.xml), copied below:

```
<?xml version='1.0' ?>

<launch>

  <arg name="fleet_name" default="caddy" description="Name of this fleet of caddy robots"/>

  <group>
    <include file="$(find-pkg-share rmf_fleet_adapter)/fleet_adapter.launch.xml">

      <!-- The name and control type of the fleet -->
      <arg name="fleet_name" value="$(var fleet_name)"/>
      <arg name="control_type" value="read_only"/>

      <!-- The nominal linear and angular velocity of the caddy -->
      <arg name="linear_velocity" value="1.0"/>
      <arg name="angular_velocity" value="0.6"/>

      <!-- The nominal linear and angular acceleration of the caddy -->
      <arg name="linear_acceleration" value="0.7"/>
      <arg name="angular_acceleration" value="1.5"/>

      <!-- The radius of the circular footprint of the caddy -->
      <arg name="footprint_radius" value="1.5"/>
      <!-- Other robots are not allowed within this radius -->
      <arg name="vicinity_radius" value="5.0"/>

      <arg name="delay_threshold" value="1.0"/>

    </include>
  </group>
```

The critical parameters are:

* `fleet_name`: This must match the `name` value that the Fleet Driver gives to its `FleetState` messages.
* `control_type`: This must be `"read_only"`.
* `linear_velocity`, `angular_velocity`, `linear_acceleration`, and `angular_acceleration`: These are estimates of the kinematic properties of the vehicles. For the sake of effective scheduling, it is preferable to overestimate these values than underestimate them, so it is best to think of these parameters as upper bounds for the values.
* `footprint_radius`: The radius of physical space that the vehicle occupies. This should cover the maximum extent of the physical footprint.
* `vicinity_radius`: The radius around the robot in which other robots are forbidden to physically enter. It is assumed that another robot entering this radius will interfere with the ability of this robot to operate.

When the launch file and Fleet Driver application are both ready, you can launch them side-by-side and the integration of the read-only fleet adapter will be complete.
