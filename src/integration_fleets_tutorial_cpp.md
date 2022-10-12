# Fleet Adapter C++ Tutorial

> The files mentioned in this tutorial can be found [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv)
> For more information about various functions please visit the hyperlinks in the header files

## [Adapter](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/Adapter.hpp)

The `Adapter` class provides functions using which new robot fleets can be added.The class provides two major functions one is `add_fleet` which allows the user to add a new fleet and second is `add_easy_traffic_light` which allows the user to add a easy traffic light system.

## [FleetUpdateHandle](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/FleetUpdateHandle.hpp)

> Note:- It's intentionally left up to the system integrators to decide how to structure your implementation when the user might receive different types of perform_action requests.
> Currently there is no support for deleting the robot.

The Update Handle works as handler between the fleet robots and the fleetadapter.The work of the handler is to act
`add_robot` allows user to add robot to the fleet.Users can use `FleetUpdateHandle` to allow the Adapter to consider requests like delivery,cleaning,patrol etc.More information is present in the API documentation.Users can also provide set of open or closed lanes.

### Information regarding charging of the robot

When the robot is idle, a timer periodically computes the estimated battery drain if the robot were to head back to the charger now. If the battery level after the estimated drain is within a safety factor applied to the recharge_threshold, the TaskManager will automatically begin a task to take the robot back to its charging station.
The recharge_threshold is used in a similar manner during task allocation. When deciding the order of tasks performed, the TaskPlanner will estimate the battery level of the robot after performing a task A. If the estimate is below the recharge_threshold, it will then check if the robot can perform task A if it performed a recharge task before this. If this is the case then it will automatically include a recharge task before task A. If it still cannot perform task A even after recharging, it will discard the possibility of doing task A in that sequence.

## [RobotUpdateHandle](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/RobotUpdateHandle.hpp)

The `RobotUpdateHandle` helps in sending updates to the robot.An instance of this class will be given everytime a new robot is added to the fleet.Users can use this instance of the class to send updates to RoMi-H about the robot's state.

## [RobotCommandHandle](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/RobotCommandHandle.hpp)

This class works as a bridge and acts as a handler for all the commands that need to be sent to the robot.

> Note:- The waypoints argument that will be given to the follow_new_path function of the RobotCommandHandle contains a list of `rmf_traffic::agv::Plan::Waypoint` objects that have a graph_index function.When that graph_index is non-null it can be passed to `Graph::get_waypoint` and then `Graph::Waypoint::get_map_name` can be called to check what the current map is for that waypoint. If there is a change in map name, then you robot should change its map.We don't provide an out-of-the-box hook for this because map switching logic is likely to vary too much between robots and deployments. For example, in a certain deployment you might need the robot to decompose a very large floor into multiple maps internally, but inside the RMF traffic system those multiple maps would all be part of one single map. In that case you'd need a dictionary of nav graph index -> robot's internal map name.

## Stepst to use rmf_fleet_adapter:

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

- Initializing some parameters for the fleet (battery, mechanical system, power, etc)

```cpp

  // Set the fleet state topic publish period
  const double fleet_state_frequency =
    rmf_fleet["publish_fleet_state"].as<double>();
  _fleet_handle->fleet_state_topic_publish_period(
    rmf_traffic::time::from_seconds(1.0/fleet_state_frequency));

  // Set up parameters required for task planner
  // Battery system
  const YAML::Node battery = rmf_fleet["battery_system"];
  const double voltage = battery["voltage"].as<double>();
  const double capacity = battery["capacity"].as<double>();
  const double charging_current = battery["charging_current"].as<double>();

  auto battery_system_optional = rmf_battery::agv::BatterySystem::make(
    voltage, capacity, charging_current);
  auto battery_system = std::make_shared<rmf_battery::agv::BatterySystem>(
    *battery_system_optional);

  // Mechanical system
  const YAML::Node mechanical = rmf_fleet["mechanical_system"];
  const double mass = mechanical["mass"].as<double>();
  const double moment_of_inertia = mechanical["moment_of_inertia"].as<double>();
  const double friction = mechanical["friction_coefficient"].as<double>();

  auto mechanical_system_optional = rmf_battery::agv::MechanicalSystem::make(
    mass, moment_of_inertia, friction);
  rmf_battery::agv::MechanicalSystem& mechanical_system =
    *mechanical_system_optional;

  std::shared_ptr<rmf_battery::agv::SimpleMotionPowerSink> motion_sink =
    std::make_shared<rmf_battery::agv::SimpleMotionPowerSink>(
    *battery_system, mechanical_system);

  // Ambient power system
  const YAML::Node ambient_system = rmf_fleet["ambient_system"];
  const double ambient_power_drain = ambient_system["power"].as<double>();
  auto ambient_power_system = rmf_battery::agv::PowerSystem::make(
    ambient_power_drain);
  if (!ambient_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for ambient power system");

    return false;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> ambient_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *ambient_power_system);

  // Tool power system
  const YAML::Node tool_system = rmf_fleet["tool_system"];
  const double tool_power_drain = ambient_system["power"].as<double>();
  auto tool_power_system = rmf_battery::agv::PowerSystem::make(
    tool_power_drain);
  if (!tool_power_system)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid values supplied for tool power system");

    return false;
  }
  std::shared_ptr<rmf_battery::agv::SimpleDevicePowerSink> tool_sink =
    std::make_shared<rmf_battery::agv::SimpleDevicePowerSink>(
    *battery_system, *tool_power_system);

  // Drain battery
  const bool drain_battery = rmf_fleet["account_for_battery_drain"].as<bool>();
  // Recharge threshold
  const double recharge_threshold =
    rmf_fleet["recharge_threshold"].as<double>();
  // Recharge state of charge
  const double recharge_soc = rmf_fleet["recharge_soc"].as<double>();

```

- Setting the fleet's finishing request

```cpp

 // Finishing tasks
  const YAML::Node task_capabilities = rmf_fleet["task_capabilities"];
  const std::string finishing_request_string =
    task_capabilities["finishing_request"].as<std::string>();
  rmf_task::ConstRequestFactoryPtr finishing_request = nullptr;
  if (finishing_request_string == "charge")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ChargeBatteryFactory>();
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ChargeBattery as finishing request");
  }
  else if (finishing_request_string == "park")
  {
    finishing_request =
      std::make_shared<rmf_task::requests::ParkRobotFactory>();
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is configured to perform ParkRobot as finishing request");
  }
  else if (finishing_request_string == "nothing")
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Fleet is not configured to perform any finishing request");
  }
  else
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Provided finishing request [%s] is unsupported. The valid "
      "finishing requests are [charge, park, nothing]. The task planner will "
      " default to [nothing].",
      finishing_request_string.c_str());
  }

```

- Adding task capabilities and performable actions to the fleet

```cpp
 // Set task planner params
  if (!_fleet_handle->set_task_planner_params(
      battery_system,
      motion_sink,
      ambient_sink,
      tool_sink,
      recharge_threshold,
      recharge_soc,
      drain_battery,
      finishing_request))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to initialize task planner parameters");

    return false;
  }

  // Currently accepting any tasks as long as they are in the config
  const auto consider =
    [](const nlohmann::json& description,
      rmf_fleet_adapter::agv::FleetUpdateHandle::Confirmation& confirm)
    {
      confirm.accept();
    };

  const bool perform_loop = task_capabilities["loop"].as<bool>();
  if (perform_loop)
  {
    _fleet_handle->consider_patrol_requests(consider);
  }

  const bool perform_delivery = task_capabilities["delivery"].as<bool>();
  if (perform_delivery)
  {
    _fleet_handle->consider_delivery_requests(
      consider, consider);
  }

  const bool perform_cleaning = task_capabilities["clean"].as<bool>();
  if (perform_cleaning)
  {
    _fleet_handle->consider_cleaning_requests(consider);
  }

  // Currently accepting any actions as long as they are in the config
  if (task_capabilities["action"])
  {
    std::vector<std::string> action_strings =
      task_capabilities["action"].as<std::vector<std::string>>();
    for (const auto& action : action_strings)
    {
      _fleet_handle->add_performable_action(action, consider);
    }
  }
```

- Calculate the Planner::StartSet for each robot using compute_plan_starts with the robot's starting pose

```cpp
// Use Start or compute plan starts
  if (std::holds_alternative<Planner::Start>(pose))
  {
    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Using provided Planner::Start to initialize starts "
      " for robot %s.",
      robot_name.c_str());
    const auto p = std::get<Planner::Start>(pose);
    starts.push_back(p);
  }
  else if (std::holds_alternative<Eigen::Vector3d>(pose))
  {
    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Running compute_plan_starts for robot %s.",
      robot_name.c_str());
    const auto p = std::get<Eigen::Vector3d>(pose);

    RCLCPP_INFO(_pimpl->_adapter->node()->get_logger(),
      "Robot %s pose is [%.2f, %.2f, %.2f]",
      robot_name.c_str(),
      p.x(), p.y(), p.z());

    // Use compute plan starts to estimate the start
    starts = rmf_traffic::agv::compute_plan_starts(
      *_pimpl->_graph, map_name, {p.x(), p.y(), p.z()},
      rmf_traffic_ros2::convert(_pimpl->_adapter->node()->now()));
  }

  if (starts.empty())
  {
    RCLCPP_ERROR(_pimpl->_adapter->node()->get_logger(),
      "Unable to determine StartSet for %s", robot_name.c_str());

    return false;
  }
```

- Add each robot to the fleet using FleetUpdateHandle::add_robot()

```cpp
    _pimpl->_fleet_handle->add_robot(
      command, robot_name, _pimpl->_traits->profile(), starts,
      [w = this->weak_from_this(), command, robot_name = std::move(robot_name)](
        const RobotUpdateHandlePtr& updater)
      {
        const auto self = w.lock();
        if (!self)
          return;
        command->set_updater(updater);
        self->_pimpl->_robots[robot_name] = command;
      });
```
