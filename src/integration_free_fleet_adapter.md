# Free Fleet

[Free fleet](https://github.com/open-rmf/free_fleet) allows users to integrate standalone mobile robots to form a heterogeneous fleet, to work with Open-RMF using the `free_fleet_adapter`.

The `free_fleet_adapter` implements the Easy Full Control fleet adapter API, and communicates with individual robots over Zenoh bridges. Out-of-the-box, it supports robots running [ROS 1 Navigation](https://wiki.ros.org/navigation) as well as [ROS 2 Navigation2](https://github.com/ros-navigation/navigation2).

Check out the [simulation examples](https://github.com/open-rmf/free_fleet?tab=readme-ov-file#simulation-examples) for more information.

Below is the ideal integration architecture of several robots that are running either ROS 2 Navigation2 or ROS 1 Navigation.

<img src="https://raw.githubusercontent.com/open-rmf/free_fleet/09562b952a0edea09f042c4b5f6d556f8cebab30/architecture.jpg">

## ROS 2 Navigation2

For ROS 2 Navigation2, the integration is enabled by [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds). With an appropriately configured bridge running on the robot, the `free_fleet_adapter` is able to communicate and control the robot, via its navigation behavior trees.

![](https://github.com/open-rmf/free_fleet/blob/media/multirobot_sim_architecture.jpg?raw=true)

![](https://github.com/open-rmf/free_fleet/blob/media/ff_unique_faster_smaller.gif?raw=true)

The Navigation2 integration currently uses the simple `navigate_to_pose` behavior to provide navigation commands from point to point following the RMF waypoints defined.

Support for utilizing more Navigation2 behaviors or custom behaviors will be coming soon.

Below is an example of the robot defined in the fleet configuration file, you can find the file [here](https://github.com/open-rmf/free_fleet/blob/main/free_fleet_examples/config/fleet/nav2_tb3_simulation_fleet_config.yaml). Take note of the defined `navigation_stack`, this allows the `free_fleet_adapter` to pick the `Nav2RobotAdapter` for this particular robot.

```yaml
...
  robots:
    nav2_tb3:
      charger: "tb3_charger"
      responsive_wait: False # Should responsive wait be on/off for this specific robot? Overrides the fleet-wide setting.
      # For Nav2RobotAdapter
      navigation_stack: 2
      initial_map: "L1"
      maps:
        L1:
          map_url: "/opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml"
,,,
```

## ROS 1 Navigation Stack

<div class='warning'>
The `zenoh-bridge-ros1` is an experimental bridge, and is actively being iterated on. Especially with ROS 1 going EOL in May 2025 (see https://wiki.ros.org/Distributions), users are encouraged to migrate to ROS 2.

The ROS 1 Navigation integration has only been tested in simulation and in ROS 1 Noetic, and is currently still using a fork of [zenoh-plugin-ros1](https://github.com/aaronchongth/zenoh-plugin-ros1/tree/namespace), to support bridge namespacing. This will be updated after contributions to upstream has been made.
</div>

For ROS 1 Navigation, the integration is enabled by [zenoh-bridge-ros1](https://github.com/eclipse-zenoh/zenoh-plugin-ros1). Similarly, with an appropriately configured bridge running on the robot, the `free_fleet_adapter` is able to communicate and control the robot, via the `move_base` topics.

![](https://github.com/open-rmf/free_fleet/blob/media/nav1_sim_architecture.jpg?raw=true)

Below is an example of the robot defined in the fleet configuration file, you can find the file [here](https://github.com/open-rmf/free_fleet/blob/main/free_fleet_examples/config/fleet/nav1_tb3_simulation_fleet_config.yaml). Take note of the defined `navigation_stack`, this allows the `free_fleet_adapter` to pick the `Nav1RobotAdapter` for this particular robot.

```yaml
...
  robots:
    nav1_tb3:
      charger: "tb3_charger"
      responsive_wait: False # Should responsive wait be on/off for this specific robot? Overrides the fleet-wide setting.
      # For Nav1RobotAdapter
      navigation_stack: 1
      initial_map: "L1"
      maps:
        L1:
          # The nav1 robot will generally no access to this nav2_bringup package, this should point to where the nav1 map is stored.
          map_url: "/tmp/navigation2/nav2_bringup/maps/tb3_sandbox.yaml"
,,,
```

## Custom Navigation Stack

Users looking to integrate their own custom navigation stack, can implement the abstract interfaces in `RobotAdapter` [here](https://github.com/open-rmf/free_fleet/blob/main/free_fleet_adapter/free_fleet_adapter/robot_adapter.py).

The implementation will need a name, for the `free_fleet_adapter` to correctly allocate adapters to each connected robot. This name is configured in the robot's configuration in the fleet configuration file. For example, Navigation2 is just `2`, while ROS 1 Navigation is `1`.

Contributions are always welcome!

## More information and links

* Free fleet [legacy implementation](https://github.com/open-rmf/free_fleet/releases/tag/1.3.0) and [documentation page](./integration_free-fleet.md).
* [Repository](https://github.com/open-rmf/free_fleet)
* [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
* [Custom zenoh-bridge-ros1 fork](https://github.com/aaronchongth/zenoh-plugin-ros1/tree/namespace)
