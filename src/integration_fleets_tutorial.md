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

> The tutorial provided below is based on the [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) implemented in the [rmf_demos](https://github.com/open-rmf/rmf_demos) repository. This specific implementation is written in Python and uses REST API as an interface between the fleet adapter and fleet manager.You may choose to use other APIs for your own integration.

## Fetch dependencies

```bash

pip3 install fastapi uvicorn

```

Clone the [fleet_adapter_template](https://github.com/open-rmf/fleet_adapter_template) repository.

Users can use the template and fill in the missing blocks of code in [`RobotClientAPI.py`](https://github.com/open-rmf/fleet_adapter_template/blob/main/fleet_adapter_template/fleet_adapter_template/RobotClientAPI.py) marked with `# IMPLEMENT YOUR CODE HERE #`. This sets up the API between the fleet adapter and the user's robots.

> The code given below serves as an example for implementing your own fleet adapter using `RobotClientAPI.py`.

## Update the `config.yaml` file

The `config.yaml` file contains important parameters for setting up the fleet adapter. Users should start by updating these configurations describing their fleet robots.

```yaml
# FLEET CONFIG =================================================================
# RMF Fleet parameters

rmf_fleet:
  name: "tinyRobot"
  fleet_manager:
    ip: "127.0.0.1"
    port: 22011
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
  recharge_threshold: 0.10 # Battery level below which robots in this fleet will not operate
  recharge_soc: 1.0 # Battery level to which robots in this fleet should be charged up to during recharging tasks
  publish_fleet_state: 10.0 # Publish frequency for fleet state, ensure that it is same as robot_state_update_frequency
  account_for_battery_drain: True
  task_capabilities: # Specify the types of RMF Tasks that robots in this fleet are capable of performing
    loop: True
    delivery: True
    clean: False
    finishing_request: "park" # [park, charge, nothing]

# TinyRobot CONFIG =================================================================

robots:
  # Here the user is expected to append the configuration for each robot in the
  # fleet.
  # Configuration for tinyRobot1
  tinyRobot1:
    robot_config:
      max_delay: 15.0 # allowed seconds of delay of the current itinerary before it gets interrupted and replanned
    rmf_config:
      robot_state_update_frequency: 10.0
      start:
        map_name: "L1"
        waypoint: "tinyRobot1_charger"
        orientation: 0.0 # radians
      charger:
        waypoint: "tinyRobot1_charger"
  # Configuration for tinyRobot2
  tinyRobot2:
    robot_config:
      max_delay: 15.0 # allowed seconds of delay of the current itinerary before it gets interrupted and replanned
    rmf_config:
      robot_state_update_frequency: 10.0
      start:
        map_name: "L1"
        waypoint: "tinyRobot2_charger"
        orientation: 0.0 # radians
      charger:
        waypoint: "tinyRobot2_charger"

reference_coordinates:
  rmf: [[20.33, -3.156], [8.908, -2.57], [13.02, -3.601], [21.93, -4.124]]
  robot: [[59, 399], [57, 172], [68, 251], [75, 429]]
```

- `rmf_fleet` important fleet parameters including vehicle traits, task capabilities and user information for connecting to the fleet manager.

- `fleet_manager` the prefix, user and password fields that can be configured to suit your chosen API. These parameters will be brought into `RobotClientAPI.py` for you to set up connection with your fleet manager/robots.

- `limits` maximum values for linear and angular accelerations and velocities.

- `profile` radius of the footprint and personal vicinity of the vehicles in this fleet.

- `reversible` a flag that can enable/disable reverse traversal in the robot.

- `battery_system` information about the battery

- `recharge_threshold` sets a value for minimum charge below which the robot must return to its charger.

- `recharge_soc` the fraction of total battery capacity to which the robot should be charged.

- `task_capabilities` the tasks that the robot can perform between `loop`, `delivery` and `clean`

- `finishing_request` what the robot should do when it finishes its task, can be set to `park`, `charge` or `nothing`

- `robots` information about individual fleet robots

- `tinyRobot1`, `tinyRobot2` name of the robot.

- `max_delay` seconds before interruption occurs and replanning happens

- `robot_state_update_frequency` how frequently should the robot update the fleet

- `start` specify the starting map name, initial waypoint (x, y) and orientation (in radians) of the robot

- `charger` waypoint name of the robot's charging point

- `reference_coordinates` if the fleet robots are not operating in the same coordinate system as RMF, you can provide two sets of (x, y) coordinates that correspond to the same locations in each system. This helps with estimating coordinate transformations from one frame to another. A minimum of 4 matching waypoints is recommended. Note: this is not being implemented in `rmf_demos_fleet_adapter` as the demos robots and RMF are using the same coordinate system.

## RobotClientAPI.py

Users can fill in the appropriate API inside [`RobotClientAPI.py`](https://github.com/open-rmf/fleet_adapter_template/blob/main/fleet_adapter_template/fleet_adapter_template/RobotClientAPI.py), which will be used by the `RobotCommandHandle` to make calls to the fleet robots. For example, if your robot uses REST API to interface with the fleet adapter, you will need to make HTTP request calls to the appropriate endpoints within these functions.

```python

import requests
from urllib.error import HTTPError


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.timeout = 5.0
        self.debug = False

```

User must initialize all the essential parameters in the class constructor required for API calls. Extra fields can be added to the constructor if need be

```python

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        if self.data() is None:
            return False
        return True

```

[`check_connection`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L39) will check if connection to the robot was successful

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

[`position`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L45) function returns the `[x, y, theta]` position of the robot in its coordinate frame

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

[`navigate`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L71) Sends an POST request to the robot with the destination coordinates. It returns true if the robot accepts the request, else false.

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

[`start_process`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L99) sends a POST request to the robot asking it to perform a task. [`stop`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L123) command tells the robot to stop moving.

```python

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        response = self.data(robot_name)
        if response is not None:
            return response['data']['destination_arrival_duration']
        else:
            return 0.0

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

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        response = self.data(robot_name)
        if response is not None:
            return response['data']['battery']/100.0
        else:
            return None

```

- `navigation_remaining_duration` will return the remaining duration for the robot to complete its current navigation

- `process_completed` checks if the robot has completed its navigation using the `navigation_completed` function.

- `battery_soc` will return battery status between 0 and 1.0
