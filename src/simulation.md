# Simulation

This chapter will describe how to generate building models from the
`traffic-editor` files and simulate fleets of robots in them.

## Motivation

Simulation environments for testing robotic solutions offer immense value across
various stages of R&D and deployment. More notably, simulations provide the
following benefits.

- **Time and resource saving:** While testing with hardware is indispensible,
  the process can slow the pace of development with additional setup time, robot
  downtime and and reset time periods between trials. As the number of
  participants scale, so do costs associated with purchasing hardware and
  consumables for testing. This is especially true with solutions such as RMF
  which aim to integrate several mobile/stationary robots, and building systems
  such as doors and lifts. Simulations provide a cost effective and time saving
  alternative for evaluating the behavior of robot systems at scale. More
  importantly simulations can help answer questions with deployment such as how
  many participants can be supported or how the existing behavior would change
  with the introduction of a new fleet, both of which can inform purchasing
  decisions for facility owners. 

- **Robust testing:** Robots in simulation do not run out of battery. Scenarios
  can be tested for hours at a stretch, at faster speeds, to fine tune
  algorithms and verify their robustness. As scenarios in simulation are
  repeatable, fixes for undesirable bugs encountered can be readily validated.
  Reaction of the system to edge cases which are rare but have severe
  consequences can also be studied through simulation. Data logged from hardware
  trials can be used to recreate the scenario in simulation which may be helpful
  for debugging. Lastly, long running simulations can instill confidence in
  facility owners.

Physics-based simulators such as `Gazebo`, carry the benefit of easily
interfacing with ROS2 nodes through wrappers provided by `gazebo_ros_pkgs`.
Gazebo plugins can be developed that accurately emulate the behavior of robots,
sensors and infrastructure systems which enhance the overall fidelity of
simulations. It is worth emphasizing here that the code used to run simulations
will be run on physical hardware as well without any changes.

However, despite these compelling benefits, simulations are sparingly employed
by developers and system integrators citing complexity over generating
environments and configuring them with appropriate plugins. The RMF project also
aims to address these hurdles by simplifying the process of setting up
simulation environments for multi-fleet traffic control.

## Building Map Generator


## RMF Assets and Plugins

Assets play a pivotal role in recreating environments in simulation. Over the
course of the RMF project, several 3D models of robots, mechanical
infrastructure systems and scene objects have been open sourced. They are
available for download [here](https://app.ignitionrobotics.org/OpenRobotics/fuel/collections/Hospital).
Beyond imparting visual accuracy, assets may be dynamic and interface with rmf
core systems through the aid of plugins. 

To simulate the behavior of hardware such as robot models and infrastructure
systems(doors & lifts), several Gazebo plugins have been architected. These
plugins are derivates of the [ModelPlugin](http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1ModelPlugin.html)
class and tie in standard ROS2 and rmf_core messages to provide the necessary
functionality. The following sections briefly describe some of these plugins.

#### Robots
As highlighted earlier, several robot models (MiR100, Magni, Hospi) have been
open sourced for use in simulation. For these models to emulate the behavior of
their physical counterparts which have been integrated with RMF, they need to 1)
interface with `rmf_fleet_adapters` and 2) navigate to locations in the
simulated world. These functionalities for a "_full control_" robot type are
achieved through the `slotcar` [plugin](https://github.com/osrf/traffic_editor/blob/master/building_gazebo_plugins/src/slotcar.cpp).
The plugin subscribes to `/robot_path_requests` and `/robot_mode_requests`
topics and responds to relevant `PathRequest` and `ModeRequest` messages
published by its `rmf_fleet_adapter`. The plugin also publishes the robot's
state to the `/robot_state` topic.

To navigate the robot through waypoints in an incoming `PathRequest` message, a
simple "rail-like" navigation algorithm is utilized which accelerates and
decelerates the robot along a straight line from its current position to the
next waypoint. The plugin relies on these fundamental assumptions
  * The robot model is a two-wheel differential drive robot
  * The left and right wheel joints are named  `joint_tire_left` and `joint_tire_right` respectively

Other parameters, majority of which are related to the kinematic properties of the robot are inferred from the `model.sdf` file. 
```xml
<plugin name="slotcar" filename="libslotcar.so">
  <nominal_drive_speed>0.5</nominal_drive_speed>
  <nominal_drive_acceleration>0.25</nominal_drive_acceleration>
  <max_drive_acceleration>0.75</max_drive_acceleration>
  <nominal_turn_speed>0.6</nominal_turn_speed>
  <nominal_turn_acceleration>1.5</nominal_turn_acceleration>
  <max_turn_acceleration>2.0</max_turn_acceleration>
  <tire_radius>0.1</tire_radius>
  <base_width>0.3206</base_width>
  <stop_distance>0.75</stop_distance>
  <stop_radius>0.75</stop_radius>
</plugin>
```

During simulation, it is assumed that the robot's path is free of static
obstacles but the plugin contains logic to pause the robot's motion if an
obstacle is detected in its path. While it is possible to deploy a sensor based
navigation stack, the approach is avoided given the computational load on the
system from doing so for each robot in the simulation. Given the focus on
traffic management of heterogeneous fleets and not robot navigation, the
`slotcar` plugin provides an efficiently means to simulate the interaction
between rmf core systems and robots.

The `slotcar` plugin is meant to serve as a generalized solution. Vendors are
encouraged to develop and distribute plugins that more accurately represent the
capabilities of their robot.

TODO: readonly 
  
#### Doors
Unlike robot models whose geometries are fixed and hence can be directly
included in the generated `.world` file, doors are custom defined in
`traffic_editor` and require special processing. As seen in the figure below, an
annotated door has several properties which include the location of its ends,
the type of door (hinged, double_hinged, sliding, double_sliding) and its range
of motion (for hinged doors).


![Figure X](images/door_traffic_editor.png)

The `building_map_generator gazebo` script parses a `.building.yaml` file for
any doors and automatically generates an sdf sub-element with links and joints
required for the door along with a configured plugin. The sdf sub-element
generated for the door in the figure above is presented below.

```xml
<model name="coe_door">
  <pose>8.077686357313898 -5.898342045416362 0.0 0 0 1.1560010438234292</pose>
  <plugin filename="libdoor.so" name="door">
    <v_max_door>0.5</v_max_door>
    <a_max_door>0.3</a_max_door>
    <a_nom_door>0.15</a_nom_door>
    <dx_min_door>0.01</dx_min_door>
    <f_max_door>500.0</f_max_door>
    <door left_joint_name="left_joint" name="coe_door" right_joint_name="empty_joint" type="SwingDoor" />
  </plugin>
  <link name="left">
    <pose>0 0 1.11 0 0 0</pose>
    <visual name="left">
      <material>
        <ambient>120 60 0 0.6</ambient>
        <diffuse>120 60 0 0.6</diffuse>
      </material>
      <geometry>
        <box>
          <size>0.8766026166317483 0.03 2.2</size>
        </box>
      </geometry>
    </visual>
    <collision name="left">
      <surface>
        <contact>
          <collide_bitmask>0x02</collide_bitmask>
        </contact>
      </surface>
      <geometry>
        <box>
          <size>0.8766026166317483 0.03 2.2</size>
        </box>
      </geometry>
    </collision>
    <inertial>
      <mass>50.0</mass>
      <inertia>
        <ixx>20.17041666666667</ixx>
        <iyy>23.36846728119012</iyy>
        <izz>3.20555061452345</izz>
      </inertia>
    </inertial>
  </link>
  <joint name="left_joint" type="revolute">
    <parent>world</parent>
    <child>left</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <lower>-1.57</lower>
        <upper>0</upper>
      </limit>
    </axis>
    <pose>0.44330130831587417 0 0 0 0 0</pose>
  </joint>
</model>
```

The door [plugin](https://github.com/osrf/traffic_editor/blob/master/building_gazebo_plugins/src/door.cpp) responds to `DoorRequest` messages with `door_name` matching its `model name` sdf tag. These messages are published over the `/door_requests` topic. The plugin is agnostic of the type of door defined and relies on the `left_joint_name` and `right_joint_name` parameters to determine which joints to actuate during open and close motions. During these motions, the joints are commanded to their appropriate limits which are specified in the parent element. The joint motions follow similar acceleration and deceleration profiles as in the `slotcar`. 
 

#### Lifts


#### Dispensers
A common scenario of interest is that of robots performing deliveries where a payload is loaded onto the robot at one location and the unloaded at another.


## Creating a Simulation
With the understanding of various facets of a simulation, we can the creation of a simulation along with scripts for assigning tasks to robots.