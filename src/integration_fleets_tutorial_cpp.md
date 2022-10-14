# Fleet Adapter C++ Tutorial

> The files mentioned in this tutorial can be found [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv)
> For more information about various functions please visit the hyperlinks in the header files

## [Adapter](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/Adapter.hpp)

The `Adapter` class helps you to communicate your fleet with other core RMF systems. You may use the function `add_fleet` to register your fleet with RMF.

## [FleetUpdateHandle](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/FleetUpdateHandle.hpp)



The `FleetUpdateHandle` works as handler between the fleet adapter and RMF. It tells RMF what are the tasks, requests and actions that the fleet can accept. Users can use the `FleetUpdateHandle` to register their robots to the fleet using `add_robot`, as well as add task capabilities (e.g. delivery, patrol, cleaning) and performable actions so that RMF can delegate them to the appropriate fleets accordingly. More information can be found in the API documentation.


## [RobotUpdateHandle](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/RobotUpdateHandle.hpp)

The `RobotUpdateHandle` contains important callbacks to keep RMF updated about the robot's status.

## [RobotCommandHandle](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/RobotCommandHandle.hpp)

This is an abstract class that users have to implement in order for their fleet adapters to receive commands from RMF successfully. Users should implement the functions `follow_new_path`, `stop` and `dock`, then fill in the internal logic accordingly to relay the navigation commands to their robot.

- Question: [How do I tell the fleet adapter if the map has changed?](https://github.com/open-rmf/rmf/discussions/180)

  The waypoints argument that will be given to the `follow_new_path` function of the `RobotCommandHandle` contains a list of `rmf_traffic::agv::Plan::Waypoint` objects that have a `graph_index` function.When that `graph_index` is non-null it can be passed to `Graph::get_waypoint` and then `Graph::Waypoint::get_map_name` can be called to check what the current map is for that waypoint. If there is a change in map name, then you robot should change its map.We don't provide an out-of-the-box hook for this because map switching logic is likely to vary too much between robots and deployments. For example, in a certain deployment you might need the robot to decompose a very large floor into multiple maps internally, but inside the RMF traffic system those multiple maps would all be part of one single map. In that case you'd need a dictionary of nav graph index -> robot's internal map name.

## Steps to use rmf_fleet_adapter:

### Step 1

Make an Adapter instance using

```cpp
static std::shared_ptr<Adapter> make(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(),
    std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt);
```

### Step 2

Initializing the fleet.

- First add fleet to adapter

```cpp
  _fleet_handle = adapter->add_fleet(
    _fleet_name, *_traits, *_graph, _server_uri);
```

- Setting up LaneRequests by setting up some publishers and subscriptions

```cpp
 _closed_lanes_pub =
    node->create_publisher<rmf_fleet_msgs::msg::ClosedLanes>(
    rmf_fleet_adapter::ClosedLaneTopicName,
    rclcpp::SystemDefaultsQoS().reliable().keep_last(1).transient_local());

  // Create subscription to lane closure requests
  _lane_closure_request_sub =
    node->create_subscription<rmf_fleet_msgs::msg::LaneRequest>(
    rmf_fleet_adapter::LaneClosureRequestTopicName,
    rclcpp::SystemDefaultsQoS(),
    [this](
      rmf_fleet_msgs::msg::LaneRequest::UniquePtr request_msg)
    {

      if (request_msg->fleet_name != this->_fleet_name &&
      !request_msg->fleet_name.empty())
        return;

      this->_fleet_handle->open_lanes(request_msg->open_lanes);
      this->_fleet_handle->close_lanes(request_msg->close_lanes);

      std::unordered_set<std::size_t> newly_closed_lanes;
      for (const auto& l : request_msg->close_lanes)
      {
        if (_closed_lanes.count(l) == 0)
          newly_closed_lanes.insert(l);

        _closed_lanes.insert(l);
      }

      for (const auto& l : request_msg->open_lanes)
        _closed_lanes.erase(l);

      for (auto& [_, robot] : this->_robots)
      {
        robot->newly_closed_lanes(newly_closed_lanes);
      }

      rmf_fleet_msgs::msg::ClosedLanes state_msg;
      state_msg.fleet_name = this->_fleet_name;
      state_msg.closed_lanes.insert(
        state_msg.closed_lanes.begin(),
        _closed_lanes.begin(),
        _closed_lanes.end());

      _closed_lanes_pub->publish(state_msg);
    });
```

- Setting important parameters for the task planner. Users can create the respective objects for these parameters and pass them to the fleet handle via `set_task_planner_param`. The header files for these objects can be found in the [`rmf_battery`](https://github.com/open-rmf/rmf_battery/tree/main/rmf_battery/include/rmf_battery/agv) repository.
  For example, to set the battery system parameters:

```cpp
auto battery_system_optional = rmf_battery::agv::BatterySystem::make(
    voltage, capacity, charging_current);
auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *battery_system_optional);
```
- Question: [How does the automatic go to battery task work?](https://github.com/open-rmf/rmf/discussions/184)

  When the robot is idle, a timer periodically computes the estimated battery drain if the robot were to head back to the charger now. If the battery level after the estimated drain is within a safety factor applied to the `recharge_threshold`, the `TaskManager` will automatically begin a task to take the robot back to its charging station.
  The `recharge_threshold` is used in a similar manner during task allocation. When deciding the order of tasks performed, the `TaskPlanner` will estimate the battery level of the robot after performing a _task A_. If the estimate is below the `recharge_threshold`, it will then check if the robot can perform _task A_ if it performed a recharge task before this. If this is the case then it will automatically include a recharge task before task A. If it still cannot perform _task A_ even after recharging, it will discard the possibility of doing _task A_ in that sequence.

- Setting the fleet's finishing request
  You may choose a finishing task for your fleet's robots between `park`, `charge` and `nothing`. You will need to create a `rmf_task::ConstRequestFactoryPtr` object and set them to the right finishing request.
  The header files for the finishing requests can be found [here](https://github.com/open-rmf/rmf_task/tree/main/rmf_task/include/rmf_task/requests) in the `rmf_task` repository.

For example, to set the finishing request to `charge`:

```cpp
rmf_task::ConstRequestFactoryPtr finishing_request = nullptr;
finishing_request = std::make_shared<rmf_task::requests::ChargeBatteryFactory>();
```

Users can now set the initialized task planner parameters with the fleet handle using `FleetUpdateHandle::set_task_planner_params`.

```cpp
_fleet_handle->set_task_planner_params(
    battery_system,
    motion_sink,
    ambient_sink,
    tool_sink,
    recharge_threshold,
    recharge_soc,
    account_for_battery_drain,
    finishing_request
)
```

Use the `FleetUpdateHandle` to add task capabilities and performable actions to your fleet. For example, if your fleet is able to carry out patrol tasks and teleop actions, you can add them as such:

```cpp
const auto consider =
    [](const nlohmann::json& description,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      confirm.accept();
    };
_fleet_handle->consider_patrol_requests(consider);
_fleet_handle->add_performable_action("teleop", consider);
```
- Question: [How do I create multiple PerformActions?](https://github.com/open-rmf/rmf/discussions/145)

  It's intentionally left up to the system integrators to decide how to structure their implementation when the user might receive different types of `perform_action` requests.
  A simple example for creating multiple PerformActions can be found below

  ```cpp
  using ActionExecutor = rmf_fleet_adapter::agv::RobotUpdateHnadle::ActionExecutor;
  std::shared_ptr<std::unordered_map<std::string, std::function<ActionExecutor>>> executor_map;
  ActionExecutor dispatching_executor = [executor_map](
    const std::string& category,
    const nlohmann::json& description,
    ActionExecution execution)
  {
    executor_map->at(category)(category, description, std::move(execution));
  };
  robot_update_handle->set_action_executor(dispatching_executor);
  ```

  `executor_map` can be given a different executor for each `perform_action` category that is expected.

### Step 3

- Compute `Planner::StartSet` for each robot
  The fleet adapter will require a `Planner::StartSet` to start planning for each robot. This object can be obtained via an `Eigen::Vector3d` of the robot's starting pose using [`rmf_traffic::agv::compute_plan_starts`](https://github.com/open-rmf/rmf_traffic/blob/main/rmf_traffic/include/rmf_traffic/agv/Planner.hpp#L916-L923):

```cpp
starts = rmf_traffic::agv::compute_plan_starts(
      *_graph, map_name, {p.x(), p.y(), p.z()},
      rmf_traffic_ros2::convert(_adapter->node()->now()));
```

- Add each robot to the fleet using [`FleetUpdateHandle::add_robot`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/FleetUpdateHandle.hpp#L71-L76)

```cpp
_fleet_handle->add_robot(
    command, robot_name, _traits->profile(), starts,
    [command](const RobotUpdateHandlePtr& updater)
    {
        command->set_updater(updater);
    });
```
