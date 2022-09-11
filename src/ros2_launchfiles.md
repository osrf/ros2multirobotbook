# Launch Files for RMF

> Note: this tutorial focuses on launch files with respect to Open-RMF. For understanding launch files better it is recommend to use the tutorials on the website of the ROS2 distro that is being used for example for galactic the documentation can be found [here](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Launch-Main.html)

## Example Launch File

[office.launch.xml](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos/launch/office.launch.xml)

```xml
<?xml version='1.0' ?>

<launch>
  <arg name="use_sim_time" default="false"/>

  <!-- Common launch -->
  <include file="$(find-pkg-share rmf_demos)/common.launch.xml">
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
    <arg name="viz_config_file" value ="$(find-pkg-share rmf_demos)/include/office/office.rviz"/>
    <arg name="config_file" value="$(find-pkg-share rmf_demos_maps)/office/office.building.yaml"/>
    <arg name="dashboard_config_file" value="$(find-pkg-share rmf_demos_dashboard_resources)/office/dashboard_config.json"/>
  </include>

  <!-- TinyRobot fleet adapter -->
  <group>
    <include file="$(find-pkg-share rmf_demos_fleet_adapter)/launch/fleet_adapter.launch.xml">
      <arg name="use_sim_time" value="$(var use_sim_time)"/>
      <arg name="nav_graph_file" value="$(find-pkg-share rmf_demos_maps)/maps/office/nav_graphs/0.yaml" />
      <arg name="config_file" value="$(find-pkg-share rmf_demos)/config/office/tinyRobot_config.yaml"/>
    </include>
  </group>

</launch>
```

The launch file contains a include tag will contains all the nodes from ` common.launch.xml` and group tag inside which the launch files of the project can be included. This is enough to get started with your project.

---

```xml
<?xml version='1.0' ?>

<launch>

  <arg name="use_sim_time" default="false" description="Use the /clock topic for time to sync with simulation"/>
  <arg name="viz_config_file" default="$(find-pkg-share rmf_visualization_schedule)/config/rmf.rviz"/>
  <arg name="config_file" description="Building description file required by rmf_building_map_tools"/>
  <arg name="dashboard_config_file" default="" description="Path to dashboard config for web rmf panel file"/>
  <arg name="initial_map" default="L1" description="Initial map name for the visualizer"/>
  <arg name="headless" default="false" description="do not launch rviz; launch gazebo in headless mode"/>
  <arg name="bidding_time_window" description="Time window in seconds for task bidding process" default="2.0"/>
  <arg name="use_rmf_panel" default="true" description="launch web and api server for rmf_demos_panel"/>

  <!-- Traffic Schedule  -->
  <node pkg="rmf_traffic_ros2" exec="rmf_traffic_schedule" output="both" name="rmf_traffic_schedule_primary">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
  <!-- Traffic Schedule monitor and replacement node -->
  <node pkg="rmf_traffic_ros2" exec="rmf_traffic_schedule_monitor" output="both" name="rmf_traffic_schedule_backup">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Blockade Moderator -->
  <node pkg="rmf_traffic_ros2" exec="rmf_traffic_blockade" output="both">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Building Map  -->
  <group>
    <node pkg="rmf_building_map_tools" exec="building_map_server" args="$(var config_file)">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
    </node>
  </group>

  <!-- Visualizer -->
  <group>
    <include file="$(find-pkg-share rmf_visualization)/visualization.launch.xml">
      <arg name="use_sim_time" value="$(var use_sim_time)"/>
      <arg name="map_name" value="$(var initial_map)"/>
      <arg name="viz_config_file" value ="$(var viz_config_file)"/>
      <arg name="headless" value="$(var headless)"/>
    </include>
  </group>

  <!-- Door Supervisor -->
  <group>
    <node pkg="rmf_fleet_adapter" exec="door_supervisor">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
    </node>
  </group>

  <!-- Lift Supervisor -->
  <group>
    <node pkg="rmf_fleet_adapter" exec="lift_supervisor">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
    </node>
  </group>

  <!-- Dispatcher Node -->
  <group>
    <node pkg="rmf_task_ros2" exec="rmf_task_dispatcher"  output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="bidding_time_window" value="$(var bidding_time_window)"/>
    </node>
  </group>

  <group if="$(var use_rmf_panel)">
    <include file="$(find-pkg-share rmf_demos)/dashboard.launch.xml">
      <arg name="use_sim_time" value="$(var use_sim_time)"/>
      <arg name="dashboard_config_file" value ="$(var dashboard_config_file)"/>
    </include>
  </group>
</launch>
```

- Traffic Schedule

  The `rmf_traffic_schedule` handles core scheduling and traffic management systems.

- Traffic Schedule monitor and replacement node

  This node is helps in monitoring the `rmf_traffic_schedule` and provides a replacement if the rmf_traffic_schedule faces any conflicts.

- Blockade Moderator

  Checks if a blockade has occurred anywhere

- Building Map

  Handles route maps that are used by `rmf_traffic` .

- Visualizer

  Used for visualization using rviz.

- Door Supervisor

  It is part of `rmf_fleet_adapter`. It is used to supervise the door. To perform action based on request.The door can be opend or closed based on request.

- Lift Supervisor

  It is part of `rmf_fleet_adapter`. It is used to supervise the lift. To perform action based on request. A lift adapter subscribes to `lift_states` while keeping track of the internal and desired state of the lift in order to prevent it from performing any actions that might interrupt

- Dispatcher Node

  Used for task dispatch. Every fleet that can perform a task request will offer a bid for how much it would "cost" them to perform a task, and the bid with the lowest "cost" will be the winner. The "cost" will be determined by two factors:

  - How quickly the task is finished
  - How much other tasks get delayed if the new task needs to preempt them
