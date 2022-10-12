# Launch Files for RMF

> Note: This tutorial focuses on launch files with respect to Open-RMF. For understanding launch files better it is recommend to use the tutorials on the website of the ROS2 distro that is being used for example for galactic the documentation can be found [here](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Launch-Main.html)

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

The launch file contains a include tag will contains all the nodes from `common.launch.xml` and group tag inside which the launch files of the project can be included. This is enough to get started with your project.

Information about each node:

- [Blockade](./sim_launchfiles_blockade.md)
- [Dispatcher](./sim_launchfiles_dispatcher.md)
- [Door](./sim_launchfiles_door.md)
- [Lift](./sim_launchfiles_lift.md)
- [Traffic Monitor](./sim_launchfiles_trafficmonitor.md)
- [Traffic Schedule](./sim_launchfiles_trafficschedule.md)
- [Traffic Monitor](./sim_launchfiles_trafficmonitor.md)
- [Visualiser](./sim_launchfiles_visualiser.md)
