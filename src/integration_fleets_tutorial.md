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
  max_delay: 10.0 # Allowed seconds of delay of the current itinerary before it gets interrupted and replanned
  publish_fleet_state:  10.0 # Publish frequency for fleet state, ensure that it is same as robot_state_update_frequency
  account_for_battery_drain: True
  task_capabilities: # Specify the types of RMF Tasks that robots in this fleet are capable of performing
    loop: True
    delivery: False
    clean: False
    finishing_request: "nothing" # [park, charge, nothing]
    action: [] # Optional, list of performable action names

# DeliveryBot CONFIG =================================================================

robots:
  # Here the user is expected to append the configuration for each robot in the
  # fleet.
  # Configuration for first robot in this fleet
  deliverybot1:
    robot_config:
      max_delay: 10.0 # Allowed seconds of delay of the current itinerary before it gets interrupted and replanned
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
  #   robot_config:
  #     max_delay: 10.0 # allowed seconds of delay of the current itinerary before it gets interrupted and replanned
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

  - `max_delay`: Allowed seconds of delay of the current itinerary before it gets interrupted and replanned. Note: currently used in `easy_fleet_adapter.py`.

  - `task_capabilities`: The tasks that the robot can perform between `loop`, `delivery` and `clean`.

  - `finishing_request`: What the robot should do when it finishes its task, can be set to `park`, `charge` or `nothing`.

  - `action` [Optional]: A list of custom performable actions for the fleet.

- `robots`: Information about each individual robot in the fleet. Each item in this section corresponds to the configuration for a single robot in the fleet. You may add more robots accordingly.

  - `deliveryBot1`: Name of the robot.

    - `max_delay`: Allowed seconds before interruption occurs and replanning happens. Note: currently used in `fleet_adapter.py`, to be moved to the `rmf_fleet` section.

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
