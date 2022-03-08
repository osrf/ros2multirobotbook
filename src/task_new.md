# Supporting a new Task in RMF

With the release of [RMF Task V2](https://github.com/open-rmf/rmf_task/pull/39), users can now construct custom tasks according to their specific needs. Different combination or sequence of robotic tasks can be dispatched to a specified robot or to the best available fleet based on the users' preferences.

The new flexible task system introduces the concept of a Phase. A task is an object that generates phases. In other words, a task is typically made up of a series or combination of phases as its building blocks. For example, a delivery task would require a robot to complete the following steps:
1. Move from its current waypoint to a pick-up location
2. Pick up the delivery payload
3. Move from the pick up location to the drop-off location
4. Drop off the payload
5. Move back to the initial starting waypoint

Each of these steps can be considered a Phase. Users can use the following public API phases to construct their own tasks:
- [`GoToPlace`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/event_description__go_to_place.json)
- [`PickUp`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/event_description__pickup.json)
- [`DropOff`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/event_description__dropoff.json)
- [`PerformAction`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/event_description__perform_action.json)

Additional phase descriptions, including those supporting the public API phases, are defined and listed [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas). They will be useful for building your own custom task.

Certain tasks may require specific phases that are not mentioned above. For example, if a delivery task involves the robot moving from the first to second level, it would require a `RequestLift` phase. Such phases are used by RMF internally and automatically added to a task when necessary, so users do not need to worry about them when creating their custom tasks.

## Building a Custom Task

Users can build and send their own tasks by publishing [`ApiRequest`](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_task_msgs/msg/ApiRequest.msg) messages. You will need to fill in the `request_id` and `json_msg` fields according to the types of phases that make up the task, as well as whether the task is intended for a specific robot or the best available fleet. You may follow these steps to construct your own task:

1. Create an `ApiRequest` publisher that sends task requests via the `/task_api_requests` topic.
2. Fill in the `request_id` field with a unique string ID that can be used to identify the task.
3. For the `json_msg` field,
    - Use the [`robot_task_request`](https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/robot_task_request.json) schema and fill in the JSON payload type with `"robot_task_request"` to send a task request to a specific robot
    - Use the [`dispatch_task_request`](https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/dispatch_task_request.json) schema and fill in the JSON payload type with `"dispatch_task_request"` to send a task request to the best available fleet
    - The `request` fields for these objects follow the [`task_request`](https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/task_request.json) schema
4. Populate the object fields with the required information.
    - The `category` and `description` fields under the `task_request` schema take in the string name of the task and the task description respectively. The JSON schema for these descriptions can be found [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas). There are currently four task descriptions available:
      - [**Clean**](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/task_description__clean.json): create your own clean task, requires the [`Clean`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/event_description__clean.json) phase description
      - [**Compose**](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/task_description__compose.json): create your own custom task that may comprise of a sequence of phases, requires descriptions for the relevant phases
      - [**Delivery**](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/task_description__delivery.json): create your own delivery task, requires the [`PickUp`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/event_description__pickup.json) and [`DropOff`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/event_description__dropoff.json) phase descriptions
      - [**Patrol**](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/task_description__patrol.json): create your own patrol task, requires the [`Place`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/schemas/place.json) description to indicate where you would like your robot to go to
5. Publish the `ApiRequest`!

#### Examples of JSON Task Requests
For a **Clean** `dispatch_task_request`:
```
{
  "type": "dispatch_task_request",
  "request": {
    "unix_millis_earliest_start_time": start_time,
    "category": "clean",
    "description": {
      "zone": "clean_lobby"
    }
  }
}
```

For a **Compose** `robot_task_request` that commands a specific robot to go to a place, followed by performing a `teleop` action:
```
{
  "type": "robot_task_request",
  "robot": "tinyRobot1",
  "fleet": "tinyRobot",
  "request": {
    "category": "compose",
    "description": {
      "category": "teleop",
      "phases": [
        {"activity": {
          "category": "sequence",
          "description": {
            "activities": [
              {"category": "go_to_place",
               "description": "coe"
              },
              {"category": "perform_action",
                "description": {"category": "teleop", "description": "coe"}
              }
            ]
          }
        }}
      ]
    }
  }
}
```

For a **Delivery** `dispatch_task_request`:
```
{
  "type": "dispatch_task_request",
  "request": {
    "category": "delivery",
    "description": {
      "pickup": {
        "place": "pantry",
        "handler": "coke_dispenser",
        "payload": [
          {"sku": "coke",
           "quantity": 1}
        ]
      },
      "dropoff": {
        "place": "hardware_2",
        "handler": "coke_ingestor",
        "payload": [
          {"sku": "coke",
           "quantity": 1}
        ]
      }
    }
  }
}
```

For a **Patrol** `robot_task_request`:
```
{
  "type": "robot_task_request",
  "robot": "tinyRobot1",
  "fleet": "tinyRobot",
  "request": {
    "category": "patrol",
    "description": {
      "places": ["pantry", "lounge"],
      "rounds": 2
    }
  }
}
```

Some examples of composed task requests can be found [here](https://github.com/open-rmf/rmf_demos/pull/122) as reference. They can be used with `rmf_demos`. Feel free to modify these files according to your own application.

## Task Management Control

You may take additional control over your tasks by sending requests to RMF to cancel a task or skip a phase. A full list of JSON schemas for such requests are defined [here](https://github.com/open-rmf/rmf_api_msgs/tree/main/rmf_api_msgs/schemas).