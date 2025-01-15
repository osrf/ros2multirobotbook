# Free Fleet

[Free fleet](https://github.com/open-rmf/free_fleet) allows users to integrate standalone mobile robots to form a heterogeneous fleet, to work with Open-RMF using the `free_fleet_adapter`.

The `free_fleet_adapter` implements the Easy Full Control fleet adapter API, and communicates with individual robots over Zenoh bridges. Out-of-the-box, it supports robots running [ROS 1 Navigation](https://wiki.ros.org/navigation) as well as [ROS 2 Navigation2](https://github.com/ros-navigation/navigation2).

Check out the [simulation examples](https://github.com/open-rmf/free_fleet?tab=readme-ov-file#simulation-examples) for more information.

Below is the ideal integration architecture of several robots that are running either ROS 2 Navigation2 or ROS 1 Navigation.

<img src="https://raw.githubusercontent.com/open-rmf/free_fleet/09562b952a0edea09f042c4b5f6d556f8cebab30/architecture.jpg">

With Zenoh bridges, users should be able to integrate their robot as-is, without additional namespacing. The Zenoh bridges are responsible for filtering topics to reduce network traffic, and to namespace the various Zenoh keys, which the `free_fleet_adapter` operates on. Handling message definitions across domains have been greatly simplified too, thanks to [`pycdr2`](https://pypi.org/project/pycdr2/) for ROS 2 messages and [`rosbags`](https://pypi.org/project/rosbags/) for ROS 1 messages.

## ROS 2 Navigation2

For ROS 2 Navigation2, the integration is enabled by [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds). With an appropriately configured bridge running on the robot, the `free_fleet_adapter` is able to communicate and control the robot, via its navigation behavior trees.

![](https://github.com/open-rmf/free_fleet/blob/media/multirobot_sim_architecture.jpg?raw=true)

![](https://github.com/open-rmf/free_fleet/blob/media/ff_unique_faster_smaller.gif?raw=true)

The Navigation2 integration currently uses the simple `navigate_to_pose` behavior to provide navigation commands from point to point following the RMF waypoints defined. Support for utilizing more Navigation2 behaviors or custom behaviors is on the roadmap, but in the meantime, contributions are always welcome!

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

Below is an example of the Zenoh configuration file used by the `zenoh-bridge-ros2dds` on the robot. The Zenoh keys are namespaced by the `namespace` parameter, while we are able to filter and rate-limit ROS 2 topics, services, actions to reduce network traffic. With the `client` mode, the bridge will seek to connect to a locally available Zenoh router.

```json5
{
  plugins: {
    ros2dds: {
      namespace: "/nav2_tb3",
      allow: {
        publishers: [".*/tf", ".*/battery_state"],
        subscribers: [],
        service_servers: [],
        service_clients: [],
        action_servers: [".*/navigate_to_pose"],
        action_clients: [],
      },
      pub_max_frequencies: [".*/navigate_to_pose/*=1", ".*/battery_state=1"],
    },
  },
  // Zenoh related configurations
  mode: "client",
  connect: {
    endpoints: []
  },
  listen: {
    endpoints: []
  },
}
```

Check out our [integration test setup](https://github.com/open-rmf/free_fleet/blob/main/.github/docker/integration-tests/nav2-docker-compose.yaml) to get a better idea on how this integration can be deployed.

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

Below is an example of the Zenoh configuration file used by the `zenoh-bridge-ros1` on the robot. The Zenoh keys are namespaced by the `bridge_namespace` parameter, while we are able to filter ROS 1 topics and services to reduce network traffic. With the `client` mode, the bridge will seek to connect to a locally available Zenoh router.

```json5
{
  plugins: {
    ros1: {
      bridge_namespace: "nav1_tb3",
      subscriber_bridging_mode: "disabled",
      publisher_bridging_mode: "disabled",
      service_bridging_mode: "disabled",
      client_bridging_mode: "disabled",
      // Zenoh -> ROS 1
      subscriber_topic_custom_bridging_mode: {
        "/move_base_simple/goal": "auto",
        "/move_base/cancel": "auto",
      },
      // ROS 1 -> Zenoh
      publisher_topic_custom_bridging_mode: {
        "/tf": "auto",
        "/battery_state": "auto",
        "/move_base/status": "auto"
      },
    },
  },
  // Zenoh related configurations
  mode: "client",
  connect: {
    endpoints: []
  },
  listen: {
    endpoints: []
  },
}
```

Check out our [integration test setup](https://github.com/open-rmf/free_fleet/blob/main/.github/docker/integration-tests/nav1-docker-compose.yaml) to get a better idea on how this integration can be deployed.

## Custom Navigation Stack

Users looking to integrate their own custom navigation stack, can implement the abstract interfaces in `RobotAdapter` [here](https://github.com/open-rmf/free_fleet/blob/main/free_fleet_adapter/free_fleet_adapter/robot_adapter.py).

The implementation will need a name, for the `free_fleet_adapter` to correctly allocate adapters to each connected robot. This name is configured in the robot's configuration in the fleet configuration file. For example, Navigation2 is just `2`, while ROS 1 Navigation is `1`.

Contributions are always welcome!

## More information and links

* [Repository](https://github.com/open-rmf/free_fleet)
* Free fleet [legacy implementation](https://github.com/open-rmf/free_fleet/releases/tag/1.3.0) and [documentation page](./integration_free-fleet.md).
* [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)
* [Custom zenoh-bridge-ros1 fork](https://github.com/aaronchongth/zenoh-plugin-ros1/tree/namespace)
