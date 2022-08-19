`fleet_adapter` acts as a bridge between the robots and the core RMF system.  
Its responsibilities include but are not limited to:  
- updating the traffic schedule with the fleet robot's positions  
- responding to tasks  
- controlling the vendor robots.  
  
`fleet_adapter` Recieves information(position, current ongoing tasks, battery levels etc.) about each robot in the fleet and sends them to the core RMF system to for task planning and scheduling.  
- When the core RMF system has a task to dispatch, it communicates with the various fleet adapters to check which fleet is suitable for taking this task.  
- It sends a request, to which fleet adapters respond by sending robot availability and statuses.  
- RMF determines the best fleet for the task and responds to the winning bid, i.e. the fleet that is selected. The response contains navigation commands relevant to the delegated task.  
- The fleet adapter will then send the navigation commands to the robot in appropriate API.


>The example given below refers to the [fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) demo in[ rmf_demos](https://github.com/open-rmf/rmf_demos) repository.


>Note, the example below is uses REST API with FAST API framework


Fetch dependencies
```bash
pip3 install fastapi uvicorn
```

```bash
mkdir fleet_adapter_demo && cd fleet_adapter_demo
ros2 pkg create --build-type ament_python fleet_adapter_demo
cd fleet_adapter_demo
mkdir launch && cd launch
touch fleet_adapter.launch.xml
```

Add the following to the `fleet_adapter.launch.xml`
```xml
<?xml version='1.0' ?>

<launch>

  <arg name="use_sim_time" default="true" description="Use the /clock topic for time to sync with simulation"/>
  <arg name="config_file" description="The config file that provides important parameters for setting up the adapter"/>
  <arg name="nav_graph_file" description="The graph that this fleet should use for navigation"/>
  <arg name="server_uri" default="" description="The URI of the api server to transmit state and task information."/>
  <arg name="output" default="screen"/>

  <!-- Fleet manager -->
  <node pkg="fleet_adapter_demo"
        exec="fleet_manager"
        args="--config_file $(var config_file) --nav_graph $(var nav_graph_file)"
        output="both">

    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Fleet adapter -->
  <group if="$(var use_sim_time)">
    <node pkg="fleet_adapter_demo"
          exec="fleet_adapter"
          args="--config_file $(var config_file) --nav_graph $(var nav_graph_file) --use_sim_time"
          output="both">

      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="server_uri" value="$(var server_uri)"/>
    </node>
  </group>

  <group unless="$(var use_sim_time)">
    <node pkg="fleet_adapter_demo"
          exec="fleet_adapter"
          args="--config_file $(var config_file) --nav_graph $(var nav_graph_file)"
          output="both">

      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="server_uri" value="$(var server_uri)"/>
    </node>
  </group>

</launch>
```
Add the following to the setup.py

```python
import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rmf_demos_fleet_adapter'

setup(
    name=package_name,
    version='1.4.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools', 'fastapi>=0.79.0', 'uvicorn>=0.18.2'],
    zip_safe=True,
    description='Fleet adapters for interfacing with RMF Demos robots with a '
                'fleet manager via REST API',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=rmf_demos_fleet_adapter.fleet_adapter:main',
            'fleet_manager=rmf_demos_fleet_adapter.fleet_manager:main',
        ],
    },
)
```

In `package.xml` add 
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>fleet_adapter_demo</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="manekdev2001@gmail.com">dev</maintainer>
  <license>TODO: License declaration</license>
  <exec_depend>rmf_fleet_adapter_python</exec_depend>

  <depend>rclpy</depend>
  <depend>rmf_fleet_msgs</depend>
  <depend>rmf_task_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```


Create a new file called `config.yaml` with following contents
```yaml

# FLEET CONFIG =================================================================
# RMF Fleet parameters

rmf_fleet:
  name: "tinyRobot"
  fleet_manager:
    ip: "127.0.0.1"
    port: 7001
    user: "some_user"
    password: "some_password"
  limits:
    linear: [0.5, 0.75] # velocity, acceleration
    angular: [0.6, 2.0] # velocity, acceleration
  profile: # Robot profile is modelled as a circle
    footprint: 0.3 # radius in m
    vicinity: 0.5 # radius in m
  reversible: True # whether robots in this fleet can reverse
  battery_system:
    voltage: 12.0 # V
    capacity: 24.0 # Ahr
    charging_current: 5.0 # A
  mechanical_system:
    mass: 20.0 # kg
    moment_of_inertia: 10.0 #kgm^2
    friction_coefficient: 0.22
  ambient_system:
    power: 20.0 # W
  tool_system:
    power: 0.0 # W
  recharge_threshold: 0.10
  recharge_soc: 1.0 
  publish_fleet_state: 10.0 
  account_for_battery_drain: True
  task_capabilities: 
    loop: True
    delivery: True
    clean: False
    finishing_request: "park" # [park, charge, nothing]

# TinyRobot CONFIG =================================================================

robots:
  tinyRobot1:
    robot_config:
      max_delay: 15.0 
      filter_waypoints: False
    rmf_config:
      robot_state_update_frequency: 10.0
      start:
        map_name: "L1"
        waypoint: "tinyRobot1_charger"
        orientation: 0.0 # radians
      charger:
        waypoint: "tinyRobot1_charger"
  # Configuration for the second robot in this fleet if there is a second robot
  tinyRobot2:
    robot_config:
      max_delay: 15.0 
      filter_waypoints: False
    rmf_config:
      robot_state_update_frequency: 10.0
      start:
        map_name: "L1"
        waypoint: "tinyRobot2_charger"
        orientation: 0.0 # radians
      charger:
        waypoint: "tinyRobot2_charger"

# TRANSFORM CONFIG =============================================================
# For computing transforms between Robot and RMF coordinate systems
# For demos, robots operate in the same coordinate system as RMF

reference_coordinates:
  rmf: [[0.0, 0.0 ],
        [1.0, 1.0],
        [2.0, 2.0],
        [3.0, 3.0]]
  robot: [[0.0, 0.0 ],
        [1.0, 1.0],
        [2.0, 2.0],
        [3.0, 3.0]]
```

- `rmf_fleet`  information about the fleet and the bots in the fleet. 
	- `fleet_manager`  prefix, username and password
		- `limits`  maximum values for linear and angular accelerations and velocities.
		- `profile`  footprint and vicinity.
		- `reversible`  a flag that can enable/disable reverse traversal in the robot.
		- `battery_system`  information about the battery
		- `recharge_threshold` sets a value for minimum charge below which the robot cannot operate.
		- `recharge_soc`  the maximum level to which robots should be charged.
		- `task_capabilities`  the capabilities of the robot
			- `finishing_request`  can be set to `park`, `charge` or `nothing`
- `robots`  information about all the types of robots in this case there is only one robot i.e. delivery bot
	- `tinyRobot1` will have all the information required by the bot.
	- `max_delay`  seconds before interruption occurs and replanning happens
	- `robot_state_update_frequency` how frequently should robot update the fleet
		- `map_name` name of the map
		- `waypoint` target location
		- `orientation` orientation in radians
	- `charger` location of the charger
- `reference_coordinates` The robot and the RMF may use different coordinate system the `reference_coordinates` help in correcting them.

The set-up is complete. 

---
The next important folder is
`rmf_fleet_adapter` folder, the structure looks like this
```bash
.
├── fleet_adapter.py
├── fleet_manager.py
├── __init__.py
├── RobotClientAPI.py
└── RobotCommandHandle.py

```

> Note only function declaration and relevant information is present here

The first thing user should look into is `RobotClientAPI.py`
The [`RobotAPI`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L28) class is a wrapper for API calls to the robot. Here, users
are expected to fill up the implementations of functions which will be used
by the `RobotCommandHandle`. For example, if your robot has a REST API, the user will need to make HTTP request calls to the appropriate endpoints within these functions.


```python
import requests
from urllib.error import HTTPError


class RobotAPI:
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.timeout = 5.0
        self.debug = False
```
User must initialize all the essential parameters in the class constructor required for API calls. Extra fields can be added to the constructor if need be

```python
def data(self, robot_name=None):
        if robot_name is None:
            url = self.prefix + f'/open-rmf/rmf_demos_fm/status/'
        else:
            url = self.prefix +\
                f'/open-rmf/rmf_demos_fm/status?robot_name={robot_name}'
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return None
```
[`data`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L172) function checks if the robot responds to the request. If the robot responds, the response is logged on the console for debugging purpose and returns the response.

```python
    def check_connection(self):
        if self.data() is None:
            return False
        return True
```

[`check_connection`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L39) will check if the robot is responding.

```python
    def position(self, robot_name: str):
        if robot_name is not None:
            url = self.prefix +\
                f'/open-rmf/rmf_demos_fm/status/?robot_name={robot_name}'
        else:
            url = self.prefix + f'/open-rmf/rmf_demos_fm/status'
        try:
            response = requests.get(url, self.timeout)
            response.raise_for_status()
            data = response.json()
            if self.debug:
                print(f'Response: {data}')
            if not data['success']:
                return None
            x = data['data']['position']['x']
            y = data['data']['position']['y']
            angle = data['data']['position']['yaw']
            return [x, y, angle]
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return None
```
[`position`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L45) function does the job returning the position of the robot in following format in robot's coordinate frame
												`[x, y, theta]`

```python
    def navigate(self,
                 robot_name: str,
                 pose,
                 map_name: str,
                 speed_limit=0.0):
        assert(len(pose) > 2)
        url = self.prefix +\
            f'/open-rmf/rmf_demos_fm/navigate/?robot_name={robot_name}'
        data = {}  # data fields: task, map_name, destination{}, data{}
        data['map_name'] = map_name
        data['destination'] = {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
        data['speed_limit'] = speed_limit
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False
```
[`navigate`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L71) Sends an POST request to the robot with the destination coordinates. It returns true if the robot accepts the request, else false.

```python 
    def start_process(self,
                      robot_name: str,
                      process: str,
                      map_name: str):
        url = self.prefix +\
            f"/open-rmf/rmf_demos_fm/start_task?robot_name={robot_name}"
        # data fields: task, map_name, destination{}, data{}
        data = {'task': process, 'map_name': map_name}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False
    def stop(self, robot_name: str):
        url = self.prefix +\
            f'/open-rmf/rmf_demos_fm/stop_robot?robot_name={robot_name}'
        try:
            response = requests.get(url, self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f'Response: {response.json()}')
            return response.json()['success']
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')
        except Exception as err:
            print(f'Other error: {err}')
        return False
```
[`start_process`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L99) function will send a POST request to the robot and will ask it to perform the task for example load/unload for delivery bot.[`stop`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L123) command will simply check if the robot has stopped.

```python
    def navigation_remaining_duration(self, robot_name: str):
        response = self.data(robot_name)
        if response is not None:
            return response['data']['destination_arrival_duration']
        else:
            return 0.0    
    def navigation_completed(self, robot_name: str):
        response = self.data(robot_name)
        if response is not None and response.get('data') is not None:
            return response['data']['completed_request']
        else:
            return False

    def process_completed(self, robot_name: str):
        return self.navigation_completed(robot_name)
    def battery_soc(self, robot_name: str):
        response = self.data(robot_name)
        if response is not None:
            return response['data']['battery']/100.0
        else:
            return None
```

- `navigation_remaining_duration` will return remaining duration
- `process_completed` checks if the robot has completed its navigation using the `navigation_completed` function.
- `battery_soc` will return battery status between 0 and 1.0


---

[`RobotCommandHandle.py`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py) is a handler for robots in fleet adapter

```python
class RobotState(enum.IntEnum):
    IDLE = 0
    WAITING = 1
    MOVING = 2
```
[`RobotState`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L44) is an enum which is used to update current status of the robot.

```python
class PlanWaypoint:
    def __init__(self, index, wp: plan.Waypoint):
        self.index = index
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes
```

[`PlanWaypoint`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L52) is a custom wrapper for Plan::Waypoint. We use this to modify position of
waypoints to prevent backtracking.

```python
class RobotCommandHandle(adpt.RobotCommandHandle):
    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 adapter,
                 api):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        
```

[`RobotCommandHandle` ](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L62)will contain a constructor which will have all the metadata about the robot.

```python
def sleep_for(self, seconds):
        goal_time =\
          self.node.get_clock().now() + Duration(nanoseconds=1e9*seconds)
        while (self.node.get_clock().now() <= goal_time):
            time.sleep(0.001)

    def clear(self):
        with self._lock:
            self.requested_waypoints = []
            self.remaining_waypoints = []
            self.state = RobotState.IDLE

    def stop(self):
        while True:
            self.node.get_logger().info(f"Requesting {self.name} to stop...")
            if self.api.stop(self.name):
                break
            self.sleep_for(0.1)
```

[`sleep_for`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L167) will send bot to sleep
[`clear`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L173) will clear all waypoints
[`stop`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L179) will stop the bot but will retain the waypoints.

```python 
def follow_new_path(
            self,
            waypoints,
            next_arrival_estimator,
            path_finished_callback):

        def _follow_path():
```

[`follow_new_path`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L187) requires the following
- `waypoints` contains the waypoint(coordinates) of the path.
- `next_arrival_estimator` contains the estimated value for arrival.
- `path_finished_callback`  is triggered when path is finished.

```python 

def dock(
            self,
            dock_name,
            docking_finished_callback):
        
        def _dock():

    def get_position(self):
        
    def get_battery_soc(self):


    def update(self):


    def update_state(self):
        

    def get_current_lane(self):
        def projection(current_position,
                       target_position,
                       lane_entry,
                       lane_exit):

    def dist(self, A, B):
        ''' Euclidian distance between A(x,y) and B(x,y)'''
        assert(len(A) > 1)
        assert(len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def get_speed_limit(self, target_waypoint):
        
    def filter_waypoints(self, wps: list, threshold=1.0):
        
    def complete_robot_action(self):
        
    def newly_closed_lanes(self, closed_lanes):
        

    def dock_summary_cb(self, msg):

    def mode_request_cb(self, msg):

```

[`_doc`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L361) will request the robot to start the relevant process
[`get_position`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L414) returns the live position of the robot in RMF coordinate frame.
[`get_battery_soc`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L433) returns battery level from 0 to 1.0

[`update`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L442) will update the position and `battery_soc` it will check if `update_handle` is present, if so it will [`update_state` ](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L451)

[`get_current_lane`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L489) will fetch the current lane
[`get_speed_limit`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L525) will get the max speed limit
[`filter_waypoints`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L536)returns filtered PlanWaypoints
[`complete_robot_action`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L556) returns if robot has completed the action it was performing
[`newly_closed_lanes`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L565) if the lane is closed the robot will reverse back to the start of the lane or update current position and waypoint index. The route will be planned again.

---
```python

def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time):

    def _task_request_check(task_capabilities, msg: TaskProfile):
        if msg.description.task_type in task_capabilities:
            return True
        else:
            return False

    fleet_handle.accept_task_requests(
        partial(_task_request_check, task_capabilities))

    def _consider(description: dict):
        confirm = adpt.fleet_update_handle.Confirmation()
        confirm.accept()
        return confirm

    # Configure this fleet to perform any kind of teleop action
    fleet_handle.add_performable_action("teleop", _consider)

    def _updater_inserter(cmd_handle, update_handle):
   

    def _add_fleet_robots():

    def _lane_request_cb(msg):
 
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("--use_sim_time", action="store_true",
                        help='Use sim time, default: false')
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])

    adapter = initialize_fleet(
        config_yaml,
        nav_graph_path,
        node,
        args.use_sim_time)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
```

[`initialize_fleet`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_adapter.py#L52) will initialize and give values to all the variable that were present in the `config.yaml `file that was created earlier. 

[`_updater_inserter` ](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_adapter.py#L163) will insert a `RobotUpdateHandle`  and set action executor for the robot. It will then initialize the `RobotAPI` with username and password.

[`_add_fleet_robots`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_adapter.py#L210) will try to add the robots mentioned in the yaml file one by one using [`RobotCommandHandle`](https://github.com/open-rmf/rmf_demos/blob/5ab2f4bc789570f49da5204df601189e22944ae5/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotCommandHandle.py#L62) to initialize the handler.

[`_lane_request_cb`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_adapter.py#L309) will publish open and closed lanes and return the adapter. It is subscribed to LaneRequest topic so it can respond whenever LaneRequest is sent.

---
```python
class Request(BaseModel):
    map_name: str
    task: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str

class State:
    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = \
            self.svy_transformer.transform(gps_json['lat'], gps_json['lon'])
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]


class FleetManager(Node):
    def __init__(self, config, nav_path):
            return data

    def robot_state_cb(self, msg):
        if (msg.name in self.robots):
            self.robots[msg.name].state = msg
            # Check if robot has reached destination
            state = self.robots[msg.name]
            if state.destination is None:
                return
            destination = state.destination
            if ((msg.mode.mode == 0 or msg.mode.mode == 1) and
                    len(msg.path) == 0):
                self.robots[msg.name].destination = None

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, state, robot_name):

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet manager...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config, args.nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(app,
                host=config['rmf_fleet']['fleet_manager']['ip'],
                port=config['rmf_fleet']['fleet_manager']['port'],
                log_level='warning')


if __name__ == '__main__':
    main(sys.argv)
```

class [`State`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_manager.py#L72) will initialize the robot state and transform the coordonates from gps location to xy plane.

class [`FleetManger`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_manager.py#L86) will initialize the robot fleet and use FASTAPI to fetch and post the data about the fleet.

[`robot_state_cb`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_manager.py#L294) is a robot state callback
[`dock_summary_cb`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_manager.py#L306) is dock summary callback
[`get_robot_state`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_manager.py#L312) will return state of the robot by inserting all the required values as mentioned in config.yaml




