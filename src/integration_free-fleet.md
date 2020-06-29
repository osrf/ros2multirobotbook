# Free Fleet

In the event that the user wishes to integrate a standalone mobile robot which doesn't come with its own fleet management system, the open source fleet management system `free_fleet` could be used.

The `free_fleet` system is split into a client and a server. The client is to be run on each of these standalone mobile robots alongside their navigation software and is intended to have direct control over the mobile robot while at the same time monitor its status and report back to the server. The client's base implementation is designed to allow interaction with different configurations of mobile robots yet report to the same server. This way, users are able to use `free_fleet` to manage a heterogenrous fleet of robots, each using different distributions of ROS, versions of ROS, navigation software, or onboard communication protocols.

The server is run on a central computer and consolidates the incoming status updates from each client to be either visualized using a UI, or relayed upstream to RMF. The server also relays commands from the user via the UI or from RMF down to the clients to be executed. Each server can work with multiple clients at a time, hence it serves the role of a fleet management system. The server can be implemented and used as its own fleet management system or work with larger systems like RMF, bridging the gap between each mobile robot's API and RMF's API and interface.

The communication between the `free_fleet` server and `free_fleet` clients is implemented using `CycloneDDS`, therefore we are not concerned if the mobile robot or central computer is running different versions of ROS.

In this section, we will address 4 different approaches of using `free_fleet` to integrate with RMF, specifically the navigation stack used by the robot. Each approach maintains a similar systems architecture, which is illustrated in the simple block diagram below, but there are specific examples depending on the software choice for the navigation stack used by the robot developer.

![FreeFleet Diagram](images/free_fleet_block_diagram.png)
<!--<img src="images/free_fleet_block_diagram.png">-->

## ROS 1 Navigation Stack

An implementation of a `free_fleet` client that works with a ROS 1 navigation stack can be found in the [repository](https://github.com/osrf/free_fleet). The implementation expects the transforms of the mobile robot to be fully defined, the mobile robot to accept navigation commands via the `move_base` action library, as well as publishing its battery status published using the `sensor_msgs/BatteryState` message.

After following the build instructions on the README on the mobile robot, the user can launch the client as part of their launch script while at the same time define the necessary parameters using `rosparam`. Below is a small snippet example of how a client can be launched, with its paramters defined,

```xml
<node name="free_fleet_client_node"
    pkg="free_fleet_client_ros1"
    type="free_fleet_client_ros1" output="screen">

  <!-- These parameters will be used to identify the mobile robots -->
  <param name="fleet_name" type="string" value="example_fleet"/>
  <param name="robot_name" type="string" value="example_bot"/>
  <param name="robot_model" type="string" value="Turtlebot3"/>

  <!-- These are the topics required to get battery and level information -->
  <param name="battery_state_topic" type="string" value="example_bot/battery_state"/>
  <param name="level_name_topic" type="string" value="example_bot/level_name"/>

  <!-- These frames will be used to update the mobile robot's location -->
  <param name="map_frame" type="string" value="example_bot/map"/>
  <param name="robot_frame" type="string" value="example_bot/base_footprint"/>

  <!-- The name of the move_base server for actions -->
  <param name="move_base_server_name" type="string" value="example_bot/move_base"/>

  <!-- These are DDS configurations used between Free Fleet clients and servers -->
  <param name="dds_domain" type="int" value="42"/>
  <param name="dds_state_topic" type="string" value="robot_state"/>
  <param name="dds_mode_request_topic" type="string" value="mode_request"/>
  <param name="dds_path_request_topic" type="string" value="path_request"/>
  <param name="dds_destination_request_topic" type="string" value="destination_request"/>

  <!-- This decides how long the client should wait for a valid transform and action server before failing -->
  <param name="wait_timeout" type="double" value="10"/>

  <!-- These define the frequency at which the client checks for commands and
  publishes the robot state to the server -->
  <param name="update_frequency" type="double" value="10.0"/>
  <param name="publish_frequency" type="double" value="1.0"/>

  <!-- The client will only pass on navigation commands if the destination or first waypoint
  of the path is within this distance away, otherwise it will ignore the command -->
  <param name="max_dist_to_first_waypoint" type="double" value="10.0"/>

</node>
```

The running `free_fleet` client will communicate with the nodes running on the robot via ROS 1, while publishing its state and subscribing to requests over DDS with the `free_fleet` Server.

The current implementation of the `free_fleet` server is implemented with ROS 2 and communicates with RMF using the aforementioned ROS 2 message and topic interfaces of an RMF fleet adapter. The ROS 2 build instructions can also be found on the same repository. Similar to the client, a simple ROS2 wrapper has been implemented, and it can be started using a `.launch.xml` file like so,

```xml
<node pkg="free_fleet_server_ros2"
    exec="free_fleet_server_ros2"
    name="free_fleet_server_node"
    node-name="free_fleet_server_node"
    output="both">

  <!-- Fleet name will be used to identify robots -->
  <param name="fleet_name" value="example_fleet"/>

  <!-- These are the ROS2 topic names that will be used to communicate with RMF -->
  <param name="fleet_state_topic" value="fleet_states"/>
  <param name="mode_request_topic" value="robot_mode_requests"/>
  <param name="path_request_topic" value="robot_path_requests"/>
  <param name="destination_request_topic" value="robot_destination_requests"/>

  <!-- These are the DDS specific configurations used to communicate with the clients -->
  <param name="dds_domain" value="42"/>
  <param name="dds_robot_state_topic" value="robot_state"/>
  <param name="dds_mode_request_topic" value="mode_request"/>
  <param name="dds_path_request_topic" value="path_request"/>
  <param name="dds_destination_request_topic" value="destination_request"/>

  <!-- This determines the frequency it checks for incoming state and request messages,
  as well as how often it publishes its fleet state to RMF -->
  <param name="update_state_frequency" value="20.0"/>
  <param name="publish_state_frequency" value="2.0"/>

  <!-- These transformations are required when the frame of the robot fleet is
  different from that of RMF globally. In order to transform a pose from the RMF
  frame to the free fleet robot frame, it is first scaled, rotated, then
  translated using these parameters -->
  <param name="scale" value="0.928"/>
  <param name="rotation" value="-0.013"/>
  <param name="translation_x" value="-4.117"/>
  <param name="translation_y" value="27.26"/>

</node>
```

Furthermore, an example of this configuration can be found in the repository as well, under the packages `ff_examples_ros1` and `ff_exmaples_ros2`. This example launches the example simulation from `ROBOTIS`, shown [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation), which has a small simulated world with 3 Turtlebot3 mobile robots, each running its own ROS 1 navigation stack.

After successful builds for both ROS 1 and ROS 2 workspaces, the simulation can be launched following [these instructions](https://github.com/osrf/free_fleet#turtlebot3-simulation), which also includes a ROS 2 `free_fleet` server, publishing fleet state messages and accepting mode and navigation requests over ROS 2 messages and topics.

## ROS 2 Navigation Stack

An implementation for a robot using ROS 2 would be similar to a ROS 1 navigation stack described earlier. At this time, the ROS 2 `free_fleet` client is still under development. This section will be updated once the refactoring, implementation and testing has been completed.

The same ready `free_fleet` server implementation in the repository will work in this scenario, as the interfaces provided by the fleet adapters are still the same ROS 2 messages and topics.

If required in the meantime, users can implement their own `free_fleet` client, by working with the `free_fleet` library that contains the base implementation and API for the DDS communication. This will be further elaborated in the next section [Custom Navigation Stack](#custom-navigation-stack).

## Developer Navigation Stack

In this implementation, it is assumed that the software running on the mobile robot was written by the robot developers themselves (or their immediate subcontractors) and the developers fully understand and have access to their robot's internal control software, API's and interfaces. This level of understanding and access will be necessary for implementing your own `free_fleet` client wrapper. The block diagram below illustrate this configuration.

![Custom Diagram](images/free_fleet_custom_config.png)
<!--<img src="images/free_fleet_custom_config.png">-->

Once the developer's `free_fleet` client is fully functional, it will be a simple task of launching the same ROS 2 `free_fleet` server as mentioned earlier in this section to work with the fleet adapters through ROS 2 messages and topics.
