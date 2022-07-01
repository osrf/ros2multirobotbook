# User-defined Custom Tasks in RMF Task

>Note: User-defined custom tasks are currently experimental


When dealing with rmf_task, there are two layers. 
- rmf_task layer
- rmf_fleet_adapter

The `rmf_task` layer is very general purpose and extremely flexible. Users of `rmf_task` have extreme freedom to define tasks however they'd like to. `rmf_task_sequence` is provided to easily define tasks as a set of activities. However, users at `rmf_task` are allowed to use any kind of logic they want. 
The `rmf_fleet_adapter` layer acts as an API that users can use and it supports customised behaviour on only `perform_action` in already existing  events mentioned [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas)
Currently, users can only add custom tasks in `perform_action`. RMF passes the command to the platform-specific end of the fleet adapter integration and formally releases control of the bot until the action is finished.
To use a custom task in `perform_action` users need to use two parts of the API. 
1. [FleetUpdateHandle::add_performable_action](https://github.com/open-rmf/rmf_ros2/blob/8440488d5583edc5a5b7226326aa2a8d41dad975/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/FleetUpdateHandle.hpp#L188-L201)
This consists of two things one is the `category` of the action and the second is the `consider` based on which it would be decided whether to accept the action or not.
2. [RobotUpdateHandle::set_action_executor](https://github.com/open-rmf/rmf_ros2/blob/8440488d5583edc5a5b7226326aa2a8d41dad975/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/RobotUpdateHandle.hpp#L178-L179) This is where you tell the fleet adapter how to instruct your robot to begin performing an action. This callback to this function consists of 
- `category(string)`  type of action
- `description(JSON)`  message which contains details about how the action should be performed
- `execution(object)` object which the platform-specific side of the fleet adapter must hold onto while the action is being performed, ideally giving periodic updates for remaining time estimates.

The robot will not participate in the traffic negotiations while using a custom task in perform_action. That means that the robot will be allowed to report its trajectory to the traffic schedule thus making it possible for other robots to avoid the bot however the bot would not be able to accommodate other bots until the task is complete.