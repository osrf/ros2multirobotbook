# Fleet Adapter Tutorial
`fleet_adapter` acts as a bridge between the robots and the core RMF system.

Its responsibilities include but are not limited to:

- Updating the traffic schedule with the fleet robot's positions

- Responding to tasks

- Controlling the vendor robots.

  

The `fleet_adapter` receives information (position, current ongoing tasks, battery levels etc.) about each robot in the fleet and sends them to the core RMF system to for task planning and scheduling.

  

- When the core RMF system has a task to dispatch, it communicates with the various fleet adapters to check which fleet is suitable for taking this task.

- It sends a request, to which fleet adapters respond by sending robot availability and statuses.

- RMF determines the best fleet for the task and responds to the winning bid, i.e. the fleet that is selected. The response contains navigation commands relevant to the delegated task.

- The fleet adapter will then send the navigation commands to the robot in appropriate API.

  

> The tutorial provided below is based on the [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) implemented in the [rmf_demos](https://github.com/open-rmf/rmf_demos) repository. This specific implementation uses REST API as an interface between the fleet adapter and fleet manager. You may choose to use other APIs for your own integration.

  

## Fetch dependencies

```bash

pip3 install fastapi uvicorn

```

Clone the  [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter)  repository.

Users can use the template and only have to fill in [`RobotClientAPI.py`](https://github.com/open-rmf/fleet_adapter_template/blob/main/fleet_adapter_template/fleet_adapter_template/RobotClientAPI.py) to get rmf_fleet adapter running.

> The code giving below serves as an example for RobotClientAPI.py implementation. Code can be found [here](https://github.com/thedevmanek/rmf_demos/tree/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter).


## Understanding `config.yaml` file
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
  recharge_threshold: 0.20 
  recharge_soc: 1.0 
  publish_fleet_state: True
  account_for_battery_drain: True
  task_capabilities: 
    loop: True
    delivery: False
    clean: False
    finishing_request: "nothing" # [park, charge, nothing]

# DeliveryBot CONFIG =================================================================

robots:
  deliverybot1:
    robot_config:
      max_delay: 10.0 
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

  

- `rmf_fleet` information about the fleet and the bots in the fleet.

- `fleet_manager` prefix, username and password

- `limits` maximum values for linear and angular accelerations and velocities.

- `profile` footprint and vicinity.

- `reversible` a flag that can enable/disable reverse traversal in the robot.

- `battery_system` information about the battery

- `recharge_threshold` sets a value for minimum charge below which the robot cannot operate.

- `recharge_soc` the maximum level to which robots should be charged.

- `task_capabilities` the capabilities of the robot

- `finishing_request` can be set to `park`, `charge` or `nothing`

- `robots` information about all the types of robots in this case there is only one robot i.e. delivery bot

- `tinyRobot1` will have all the information required by the bot.

- `max_delay` seconds before interruption occurs and replanning happens

- `robot_state_update_frequency` how frequently should robot update the fleet

- `map_name` name of the map

- `waypoint` target location

- `orientation` orientation in radians

- `charger` location of the charger

- `reference_coordinates` The robot and the RMF may use different coordinate system the `reference_coordinates` help in correcting them.


## RobotClientAPI.py 
Users can start by filling in the appropriate API inside [`RobotClientAPI.py`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L28), which will be used by the `RobotCommandHandle` to make calls to the fleet robots. For example, if your robot uses REST API to interface with the fleet adapter, you will need to make HTTP request calls to the appropriate endpoints within these functions.

  
  

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

  

[`check_connection`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L39) will check if the robot is responding.

  

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

[`position`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L45) function does the job returning the position of the robot in following format in robot's coordinate frame

`[x, y, theta]`

  

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

[`start_process`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L99) function will send a POST request to the robot and will ask it to perform the task for example load/unload for delivery bot.[`stop`](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/RobotClientAPI.py#L123) command will simply check if the robot has stopped.

  

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

- `navigation_remaining_duration` will return remaining duration

- `process_completed` checks if the robot has completed its navigation using the `navigation_completed` function.

- `battery_soc` will return battery status between 0 and 1.0

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
  
  