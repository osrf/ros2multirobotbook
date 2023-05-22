# Fleet Adapter Tutorial (Python)

`fleet_adapter` acts as a bridge between the robots and the core RMF system.

Its responsibilities include but are not limited to:

- Updating the traffic schedule with the fleet robot's positions

- Responding to tasks

- Controlling the vendor robots.

The `fleet_adapter` receives information (position, current ongoing tasks, battery levels etc.) about each robot in the fleet and sends them to the core RMF system for task planning and scheduling.

- When the core RMF system has a task to dispatch, it communicates with the various fleet adapters to check which fleet is suitable for taking this task.

- It sends a request, to which fleet adapters respond by submitting their fleet robots' availability and statuses.

- RMF determines the best fleet for the task and responds to the winning bid, i.e. the fleet that is selected. The response contains navigation commands relevant to the delegated task.

- The fleet adapter will then send the navigation commands to the robot in appropriate API.

> The tutorial provided below is based on the [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) implemented in the [rmf_demos](https://github.com/open-rmf/rmf_demos) repository. This specific implementation is written in Python and uses REST API as an interface between the fleet adapter and fleet manager. You may choose to use other APIs for your own integration.

## 1. Pre-requisites

### Fetch dependencies

Since the `rmf_demos_fleet_adapter` uses REST API as an interface between the fleet adapter and robot fleet manager, we will need to install the required dependencies to use FastAPI.
```bash
pip3 install fastapi uvicorn
```

### Get started with the fleet adapter template

Clone the [fleet_adapter_template](https://github.com/open-rmf/fleet_adapter_template) repository.

```bash
git clone https://github.com/open-rmf/fleet_adapter_template.git
```

This template contains the code for both Full Control and Easy Full Control fleet adapters. Both implementations use API calls in [`RobotClientAPI.py`](https://github.com/open-rmf/fleet_adapter_template/blob/main/fleet_adapter_template/fleet_adapter_template/RobotClientAPI.py) to communicate with the robots.

## 2. Update the `config.yaml` file

The `config.yaml` file contains important parameters for setting up the fleet adapter. Users should start by updating these configurations describing their fleet robots.

It is important to stick to the provided fields in the sample `config.yaml` below, otherwise there will be import errors when parsing this YAML file to the fleet adapter. If you would like to edit any of the field names or value range, or even append additional fields, please ensure that you also modify the part of your fleet adapter code that handles this configuration import accordingly.

Some fields are optional as indicated below.

```yaml
# FLEET CONFIG =================================================================
# RMF Fleet parameters

rmf_fleet:
  name: "DeliveryBot"
  fleet_manager:
    prefix: "http://127.0.0.1:8000"
    user: "some_user"
    password: "some_password"
  limits:
    linear: [0.4, 0.2] # velocity, acceleration
    angular: [0.3, 0.35] # velocity, acceleration
  profile: # Robot profile is modelled as a circle
    footprint: 0.5 # radius in m
    vicinity: 0.6 # radius in m
  reversible: False # whether robots in this fleet can reverse
  battery_system:
    voltage: 24.0 # V
    capacity: 40.0 # Ahr
    charging_current: 26.4 # A
  mechanical_system:
    mass: 80.0 # kg
    moment_of_inertia: 20.0 #kgm^2
    friction_coefficient: 0.20
  ambient_system:
    power: 20.0 # W
  tool_system:
    power: 760.0 # W
  recharge_threshold: 0.20 # Battery level below which robots in this fleet will not operate
  recharge_soc: 1.0 # Battery level to which robots in this fleet should be charged up to during recharging tasks
  publish_fleet_state:  10.0 # Publish frequency for fleet state, ensure that it is same as robot_state_update_frequency
  account_for_battery_drain: True
  task_capabilities: # Specify the types of RMF Tasks that robots in this fleet are capable of performing
    loop: True
    delivery: False
    clean: False
    finishing_request: "nothing" # [park, charge, nothing]
    action: [] # Optional, list of custom actions that the fleet can perform

# DeliveryBot CONFIG =================================================================

robots:
  # Here the user is expected to append the configuration for each robot in the
  # fleet.
  # Configuration for first robot in this fleet
  deliverybot1:
    rmf_config:
      robot_state_update_frequency: 0.5
      start:
        map_name: "L1"
        # waypoint: "charger_deliverybot1" # Optional
        # orientation: 0.0 # Optional, radians
        waypoint: null
        orientation: null
      charger:
        waypoint: "charger_deliverybot1"
  # Configuration for the second robot in this fleet if there is a second robot
  # Uncomment if more than one robot exists.
  # deliverybot2:
  #   rmf_config:
  #     robot_state_update_frequency: 0.5
  #     start:
  #       map_name: "L1"
  #       # waypoint: "charger_deliverybot2" # Optional
  #       # orientation: 0.0 # Optional, radians
  #       waypoint: null
  #       orientation: null
  #     charger:
  #       waypoint: "charger_deliverybot2"

# TRANSFORM CONFIG =============================================================
# For computing transforms between Robot and RMF coordinate systems

# Optional
reference_coordinates:
  rmf: [[20.33, -3.156],
        [8.908, -2.57],
        [13.02, -3.601],
        [21.93, -4.124]]
  robot: [[59, 399],
        [57, 172],
        [68, 251],
        [75, 429]]
```

- `rmf_fleet`: Important fleet parameters including vehicle traits, task capabilities and user information for connecting to the fleet manager.

  - `fleet_manager`: The *prefix*, *user* and *password* fields that can be configured to suit your chosen API. Do make sure to also edit the corresponding fields in `fleet_adapter.py` if you do modify them. These parameters will be brought into `RobotClientAPI.py` for you to set up connection with your fleet manager/robots.

  - `limits`: Maximum values for linear and angular accelerations and velocities.

  - `profile`: Radius of the footprint and personal vicinity of the vehicles in this fleet.

  - `reversible`: A flag to enable/disable reverse traversal in the robot.

  - `battery_system`: Information about the battery's voltage, capacity and charging current.

  - `recharge_threshold`: Sets a value for minimum charge below which the robot must return to its charger.

  - `recharge_soc`: The fraction of total battery capacity to which the robot should be charged.

  - `task_capabilities`: The tasks that the robot can perform between `loop`, `delivery` and `clean`.

  - `finishing_request`: What the robot should do when it finishes its task, can be set to `park`, `charge` or `nothing`.

  - `action` [Optional]: A list of custom performable actions for the fleet.

- `robots`: Information about each individual robot in the fleet. Each item in this section corresponds to the configuration for a single robot in the fleet. You may add more robots accordingly.

  - `deliveryBot1`: Name of the robot.

    - `robot_state_update_frequency`: How frequently should the robot update the fleet.

    - `start`: Specifies the starting map name, initial waypoint (x, y) and orientation (in radians) of the robot.

    - `charger waypoint`: Name of the robot's charging point.

- `reference_coordinates` [Optional]: If the fleet robots are not operating in the same coordinate system as RMF, you can provide two sets of (x, y) coordinates that correspond to the same locations in each system. This helps with estimating coordinate transformations from one frame to another. A minimum of 4 matching waypoints is recommended.

  Note: this is not being implemented in `rmf_demos_fleet_adapter` as the demos robots and RMF are using the same coordinate system.

## 3. Create navigation graphs

A navigation graph is required to be parsed to the fleet adapter so that RMF can understand the robots' environment. They can be created using the [RMF Traffic Editor](https://github.com/open-rmf/rmf_traffic_editor.git) and the [`building_map_generator nav`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/parse_graph.cpp) CLI provided. Refer to the traffic editor repo's README for installation and map generation instructions.

You may also want to look through the [Traffic Editor](./traffic-editor.md) section of this Book for detailed information and instructions on creating your own digital maps. 

You should now have a YAML file with information about the lanes and waypoints (among other information) that describe the paths your robot fleet can take.


## 4. Fill in your `RobotAPI`

[`RobotClientAPI.py`](https://github.com/open-rmf/fleet_adapter_template/blob/main/fleet_adapter_template/fleet_adapter_template/RobotClientAPI.py) provides a set of methods being used by the fleet adapter. These callbacks are triggered when RMF needs to send or retrieve information via the fleet adapter to/from the managed robots. To cater to the interface of your choice, you need to fill in the missing code blocks marked with `# IMPLEMENT YOUR CODE HERE #` within `RobotAPI` with logics to send or retrieve the corresponding information. For example, if your robot uses REST API to interface with the fleet adapter, you will need to make HTTP request calls to the appropriate endpoints within these functions. 

Parts of the [`RobotAPI`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py) class implementated for `rmf_demos_fleet_adapter` is further elaborated below to illustrate how you should fill in the missing code:

- `position`: Retrieves the robot's current position in its coordinate frame in the format `[x, y, theta]` via a GET request.

```python
    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
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

- `navigate`: Sends a POST request to the robot's fleet manager with the destination coordinates. Returns true if the robot successfully accepts the request, else false.
```python
    def navigate(self,
                 robot_name: str,
                 pose,
                 map_name: str,
                 speed_limit=0.0):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
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

- `start_process`: Sends a POST request to the robot's fleet manager asking it to perform a task.

```python
    def start_process(self,
                      robot_name: str,
                      process: str,
                      map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
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
```

- `stop`: Commands the robot to stop moving via a POST request.
```python
    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
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

- `navigation_remaining_duration`: Retrieves the remaining duration for the robot to complete its current navigation.

```python
    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        response = self.data(robot_name)
        if response is not None:
            return response['data']['destination_arrival_duration']
        else:
            return 0.0
```

- `process_completed`: Checks if the robot has completed the ongoing process or task. This implementation uses an additional `navigation_completed` method to perform a GET request on the process status.

```python
    def navigation_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        response = self.data(robot_name)
        if response is not None and response.get('data') is not None:
            return response['data']['completed_request']
        else:
            return False

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        return self.navigation_completed(robot_name)
```

Further parameters may be added to `RobotAPI` to be used in these callbacks if required, such as authentication details and task IDs. You may also wish to write additional methods in `RobotAPI` and call them in your `fleet_adapter.py` for specific use cases.

## 5. Create your fleet adapter!

Now that we have our components ready, we can start writing our fleet adapter. You may either

- Use the Full Control Python fleet adapter as a starting point and optionally add customization to its logic by modifying the RobotCommandHandle code, or
- Use our out-of-the-box fleet adapter logic with Easy Full Control with the [`easy_fleet_adapter`]() template provided in `fleet_adapter_template`.

The following steps elaborate on how an **Easy Full Control** adapter can be written. It uses the same C++ API as the Full Control fleet adapter, with an additional layer of the [`EasyFullControl`](https://github.com/open-rmf/rmf_ros2/blob/feature/easy_full_control/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/EasyFullControl.hpp) class that helps to set up the fleet adapter's internal logic without user intervention. You will only need to parse your `config.yaml` and navigation graphs, as well as have some required callback functions ready, to create and start using your fleet adapter.

Some code snippets from the [`easy_fleet_adapter.py`](https://github.com/open-rmf/rmf_demos/blob/feature/easy_full_control/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/easy_fleet_adapter.py) implemented in [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) are provided as examples.

### 5a. Create a `FleetAdapter` class

For ease of implementation, we create a `FleetAdapter` class to keep track of some fleet parameters and to spin up our adapter node easily.

```python
class FleetAdapter:

    def __init__(self, config_path, nav_graph_path, node, use_sim_time):
        # Load config yaml
        with open(config_path, "r") as f:
            config_yaml = yaml.safe_load(f)
        # Initialize robot API for this fleet
        fleet_config = config_yaml['rmf_fleet']
        prefix = 'http://' + fleet_config['fleet_manager']['ip'] + \
                 ':' + str(fleet_config['fleet_manager']['port'])
        self.api = RobotAPI(
            prefix,
            fleet_config['fleet_manager']['user'],
            fleet_config['fleet_manager']['password'])

        node.declare_parameter('server_uri', rclpy.Parameter.Type.STRING)
        server_uri = node.get_parameter(
            'server_uri').get_parameter_value().string_value
        if server_uri == "":
            server_uri = None
```

Here we load the fleet configuration and navigation graph YAML files, and set up access to the `RobotAPI` class that we updated previously. The `server_uri` parameter is also initialized here to be used with RMF Web.

```python
        # Create EasyFullControl adapter
        self.configuration = adpt.easy_full_control.Configuration.make(
            config_path, nav_graph_path, server_uri)
        self.adapter = self.initialize_fleet(
            self.configuration, config_yaml['robots'], node, use_sim_time)
```

We then create the `Configuration` object that will be required for the instantiation of the fleet adapter. This object encapsulates all the information about your fleet configuration and navigation graph and passes them on easily to the fleet adapter. 

Now we are ready to make the Easy Full Control adapter, which will be done in the `initialize_fleet` method.

### 5b. Make the adapter

```python
    def initialize_fleet(self, configuration, robots_yaml, node, use_sim_time):
        # Make the easy full control
        easy_full_control = adpt.EasyFullControl.make(configuration)

        if use_sim_time:
            easy_full_control.node.use_sim_time()
```

Inside the `initialize_fleet` method, we create the fleet adapter by passing our `Configuration` object to `EasyFullControl`'s `make(~)` function.

### 5c. Create individual robot callbacks

Before we can add our robots to the fleet, we need to define callbacks for each of them to perform various functions. These callbacks are needed for RMF to
- Retrieve the robot's state information, such as charger name, current location, battery state of charge, etc. (`GetStateCallback`)
- Send navigation (`NavigationRequest`), docking (`DockRequest`) or action (`ActionExecutor`) commands to the robot
- Request for the robot to stop moving (`StopRequest`)

They are defined in the [`EasyFullControl`](https://github.com/open-rmf/rmf_ros2/blob/feature/easy_full_control/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/EasyFullControl.hpp) class, take a moment to look at the functions and the arguments they should take before implementing your own.

If you have multiple robots to be added to your fleet that use the same callbacks, you can opt to create a generic function that takes in an identification argument (e.g. robot name) to avoid re-writing similar code. Some examples from `rmf_demos_fleet_adapter` below show how they can be implemented. Here we use parameters like `self.cmd_id` and `self.actions` to keep track of tasks and processes being carried out for the demos integration.

**RobotState**

The `GetStateCallback` function will be called at every update interval for RMF to keep tabs on the robot's whereabouts and availability for task allocation. It returns a [`RobotState`](https://github.com/open-rmf/rmf_ros2/blob/feature/easy_full_control/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/EasyFullControl.hpp#L230) object or `None` if the robot state cannot be queried.

```python
        def _robot_state(robot_name):
            # Use the RobotAPI to retrieve the robot's state information
            data = self.api.data(robot_name)

            # If there is no information available, return None
            if data is None or data['success'] is False:
                return None

            # Return the robot information in a RobotState object
            pos = data['data']['position']
            action = False
            if robot_name in self.actions and \
                    self.actions[robot_name] is not None:
                action = True
            state = adpt.easy_full_control.RobotState(
                robot,
                robot_config['charger']['waypoint'],
                data['data']['map_name'],
                [pos['x'], pos['y'], pos['yaw']],
                data['data']['battery'],
                action)
            self.last_map[robot_name] = data['data']['map_name']
            return state
```

**Navigation**

The `NavigationRequest` callback requires users to trigger a robot-specific navigation command, and call `execution.finished()` when the robot has reached its target waypoint.

```python
        def _navigate(robot_name, map_name, goal, execution):
            # Clear current ongoing navigations if any
            if robot_name in self.nav_threads:
                if self.nav_threads[robot_name] is not None:
                    if self.nav_threads[robot_name].is_alive():
                        self.nav_threads[robot_name].join()

            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id

            # Use RobotAPI to send navigation commands
            self.api.navigate(robot_name, cmd_id, goal, map_name)

            # Internal tracking that calls execution.finished()
            # when the robot reaches the destination
            with self._lock:
                self.nav_threads[robot_name] = None
                self.nav_threads[robot_name] = threading.Thread(
                    target=self.start_navigation,
                    args=(robot_name, execution))
                self.nav_threads[robot_name].start()
```

In `rmf_demos_fleet_adapter`, this is done by passing the execution object to an internal function `start_navigation()` that continually checks the robot's navigation progress using RobotAPI's `navigate()` method. You can also use `execution.update_request()` to ask RMF to replan its trajectory and update the remaining duration for the navigation task.

```python
    def start_navigation(self,
                         robot_name,
                         execution):
        while not self.api.process_completed(robot_name,
                                             self.cmd_ids[robot_name]):
            remaining_time = self.api.navigation_remaining_duration(
                robot_name, self.cmd_ids[robot_name])
            if remaining_time:
                remaining_time = datetime.timedelta(seconds=remaining_time)
            request_replan = self.api.requires_replan(robot_name)
            execution.update_request(request_replan, remaining_time)
        # Navigation completed
        execution.finished()
```

**DockRequest**

Similarly, the `DockRequest` callback should call RobotAPI's `dock()` method and keep track of the robot's docking progress, and trigger `execution.finished()` when it is done with the task.

```python
        def _dock(robot_name, dock_name, execution):
            if dock_name not in self.docks:
                node.get_logger().info(
                    f'Requested dock {dock_name} not found, '
                    f'ignoring docking request')
                return

            # Clear current ongoing docking if any
            if robot_name in self.traj_threads:
                if self.traj_threads[robot_name] is not None:
                    if self.traj_threads[robot_name].is_alive():
                        self.traj_threads[robot_name].join()

            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            self.api.start_process(
                robot_name, cmd_id, dock_name, self.last_map[robot_name])

            positions = []
            for wp in self.docks[dock_name]:
                positions.append([wp.x, wp.y, wp.yaw])
            task_completed_cb = self.api.navigation_completed
            node.get_logger().info(
                f"Robot {robot_name} is docking at {dock_name}...")

            with self._lock:
                self.traj_threads[robot_name] = None
            self.traj_threads[robot_name] = threading.Thread(
                target=self.start_trajectory,
                args=(task_completed_cb, robot_name, positions,
                      execution.handle(), execution))
            self.traj_threads[robot_name].start()
```

In this example, we have an internal function `start_trajectory()` to check on the docking process. We also made use of RMF adapter's scheduling tools to create trajectories for this request, allowing for additional monitoring of the robot.

```python
    # Track trajectory of docking or custom actions
    def start_trajectory(self,
                         task_completed_cb,
                         robot_name,
                         positions,
                         update_handle,
                         execution):
        while not task_completed_cb(robot_name, self.cmd_ids[robot_name]):
            now = datetime.datetime.fromtimestamp(0) + \
                self.adapter.node.now()
            traj = schedule.make_trajectory(
                self.configuration.vehicle_traits(),
                now,
                positions)
            itinerary = schedule.Route(self.last_map[robot_name], traj)
            if update_handle is not None:
                participant = update_handle.get_unstable_participant()
                participant.set_itinerary([itinerary])
        execution.finished()
```

**Custom Actions**

If you added custom actions to your `config.yaml`, you can specify the logic and trigger for the action using the `ActionExecutor` callback. As usual, you will have to call `execution.finished()` to signal the end of the performed action. The action name that you added will be carried here under `category`. This example stores the `ActionExecution` object in `self.actions`, and then uses an additional `toggle_action()` API to relay to the fleet manager that it can start on the given action.

```python
        def _action_executor(
                robot_name: str,
                category: str,
                description: dict,
                execution: adpt.robot_update_handle.ActionExecution):
            self.actions[robot_name] = execution
            self.api.toggle_action(robot_name, True)
```

The sections [User-defined Task](./task_userdefined.md) and [Supporting a New Task](./task_new.md) of this Book explain in further detail how an `ActionExecutor` function can be written and how a custom task can be created respectively.

**Stop**

`StopRequest` simply calls RobotAPI's corresponding method to request for the robot to stop moving.

```python
        def _stop(robot_name):
            cmd_id = self.next_id
            self.next_id += 1
            self.cmd_ids[robot_name] = cmd_id
            return self.api.stop(robot_name, cmd_id)
```

### 5d. Add robot to the fleet adapter

To add robots to the fleet adapter, you will need to pass the robot's starting state and the callbacks defined above to `EasyFullControl`'s `add_robot(~)` method. Here we keep adding the robots listed in our `config.yaml` until we found all of them.

```python
        # Add the robots
        missing_robots = robots_yaml
        while len(missing_robots) > 0:
            for robot in list(missing_robots.keys()):
                node.get_logger().info(f'Connecting to robot [{robot}]')
                robot_config = robots_yaml[robot]['rmf_config']
                state = _robot_state(robot)
                if state is None:
                    node.get_logger().info(f'Unable to find robot [{robot}], trying again...')
                    time.sleep(0.2)
                    continue
                # Found robot, add to fleet
                easy_full_control.add_robot(
                    state,
                    partial(_robot_state, robot),
                    partial(_navigate, robot),
                    partial(_stop, robot),
                    partial(_dock, robot),
                    partial(_action_executor, robot))
                node.get_logger().info(f'Successfully added new robot: [{robot}]')
                del missing_robots[robot]

        return easy_full_control
```

### 5e. Initialize the `FleetAdapter`

With the `FleetAdapter` set up, we can create our adapter instance. We take in the configuration and navigation graph file paths as command line arguments and pass them to the `FleetAdapter` class along with a ROS 2 node for the command handle. Additionally you might want to use the `use_sim_time` parameter if you would like to run the adapter in simulation.

```python
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
    parser.add_argument("-sim", "--use_sim_time", action="store_true",
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

    adapter = FleetAdapter(
        config_path,
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