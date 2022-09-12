# Fleet Adapter C++ Tutorial

> The files mentioned in this tutorial can be found [here](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv)

## [Adapter.hpp](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/Adapter.hpp)

```cpp
#ifndef RMF_FLEET_ADAPTER__AGV__ADAPTER_HPP
#define RMF_FLEET_ADAPTER__AGV__ADAPTER_HPP

#include <rmf_fleet_adapter/agv/FleetUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/EasyTrafficLight.hpp>

#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <rclcpp/node.hpp>

namespace rmf_fleet_adapter {
namespace agv {
class Adapter : public std::enable_shared_from_this<Adapter>
{
public:

  static std::shared_ptr<Adapter> init_and_make(
    const std::string& node_name,
    std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt);

  static std::shared_ptr<Adapter> make(
    const std::string& node_name,
    const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions(),
    std::optional<rmf_traffic::Duration> discovery_timeout = std::nullopt);

  std::shared_ptr<FleetUpdateHandle> add_fleet(
    const std::string& fleet_name,
    rmf_traffic::agv::VehicleTraits traits,
    rmf_traffic::agv::Graph navigation_graph,
    std::optional<std::string> server_uri = std::nullopt);

  using Blockers = std::vector<EasyTrafficLight::Blocker>;

  void add_easy_traffic_light(
    std::function<void(EasyTrafficLightPtr handle)> handle_callback,
    const std::string& fleet_name,
    const std::string& robot_name,
    rmf_traffic::agv::VehicleTraits traits,
    std::function<void()> pause_callback,
    std::function<void()> resume_callback,
    std::function<void(Blockers)> deadlock_callback = nullptr);


  /// Get the rclcpp::Node that this adapter will be using for communication.
  std::shared_ptr<rclcpp::Node> node();

  /// const-qualified node()
  std::shared_ptr<const rclcpp::Node> node() const;

  /// Begin running the event loop for this adapter. The event loop will operate
  /// in another thread, so this function is non-blocking.
  Adapter& start();

  /// Stop the event loop if it is running.
  Adapter& stop();

  /// Wait until the adapter is done spinning.
  ///
  /// \sa wait_for()
  Adapter& wait();

  /// Wait until the adapter is done spinning, or until the maximum wait time
  /// duration is reached.
  ///
  /// \sa wait()
  Adapter& wait_for(std::chrono::nanoseconds max_wait);

  class Implementation;
private:
  Adapter();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using AdapterPtr = std::shared_ptr<Adapter>;
using ConstAdapterPtr = std::shared_ptr<const Adapter>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__ADAPTER_HPP

```

[`init_and_make`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/Adapter.hpp#L51) will initialize rclcpp context and makes an adapter instance.You can add the fleets to be adapted.
takes two arguments

- node_name - The name of rclcpp node that will be produced for this adapter.
- discovery_timeout - The minimum time it will wait to discover the schedule node before giving up. If the default `rmf_utils:nullopt` is used it will use `discovery_timeout`node parameter or wait for 1 minute if`discovery_timeout` is not defined.

[`make`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/Adapter.hpp#L75) instantiates an rclcpp::Node like `init_and_make` however it is more customisable.

- node_name - The name of rclcpp node that will be produced for this adapter.
- node_options - It can be used to customize the node.
- discovery_timeout - The minimum time it will wait to discover the schedule node before giving up. If the default `rmf_utils:nullopt` is used it will use `discovery_timeout` node parameter or wait for 1 minute if `discovery_timeout` is not defined.

`add_fleet` allows to add a fleet that needs to be adapted.

- fleet_name - name of the fleet
- traits - specify approximate traits of the vehicle.
- navigation_graph - specify navigation graph used by the vehicles in the fleet
- server_uri - uri of the websocket server that receives updates on the tasks and states. If the default `rmf_utils:nullopt` is used it will not publish any data.

`add_easy_traffic_light` helps you create simple version of traffic light which allows to manage robot that can only support pause or resume commands.This API is simpler to use than the standard traffic light API but it provides less information about the exact timing needed for the starts and stops.This API must only be used if system integrators can ensure very low-latency and reliable connections to the robots to ensure that commands arrive on time.

- handle_callback -The callback that will be triggered when the EasyTrafficLight handle is ready to be used. This callback will only be triggered once.

- fleet_name - The name of the fleet

- robot_name -The name of the robot

- traits -The traits of the robot

- pause_callback -The callback that should be triggered by the traffic light when an immediate pause is needed.

- resume_callback -The callback that will be triggered by the traffic light when the robot may resume moving forward.

- deadlock_callbackThe callback that will be triggered by the traffic light if there is a permanent blocker disrupting the ability of this vehicle to proceed.Manual intervention may be required in this circumstance. A callback does not need to be provided for this. Either way, an error message will be printed to the log.

## [FleetUpdateHandle.hpp](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/FleetUpdateHandle.hpp)

The Update Handle works as handler between the fleet robots and the fleetadapter. 

```c++
#ifndef RMF_FLEET_ADAPTER__AGV__FLEETUPDATEHANDLE_HPP
#define RMF_FLEET_ADAPTER__AGV__FLEETUPDATEHANDLE_HPP

#include <rmf_fleet_adapter/agv/RobotUpdateHandle.hpp>
#include <rmf_fleet_adapter/agv/RobotCommandHandle.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/task_profile.hpp>

#include <rmf_battery/agv/BatterySystem.hpp>
#include <rmf_battery/DevicePowerSink.hpp>
#include <rmf_battery/MotionPowerSink.hpp>

#include <rmf_task/RequestFactory.hpp>

#include <nlohmann/json.hpp>
#include <rmf_task/Activator.hpp>
#include <rmf_task_sequence/Phase.hpp>
#include <rmf_task_sequence/Event.hpp>

namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
class FleetUpdateHandle : public std::enable_shared_from_this<FleetUpdateHandle>
{
public:

  void add_robot(
    std::shared_ptr<RobotCommandHandle> command,
    const std::string& name,
    const rmf_traffic::Profile& profile,
    rmf_traffic::agv::Plan::StartSet start,
    std::function<void(std::shared_ptr<RobotUpdateHandle> handle)> handle_cb);

  /// Confirmation is a class used by the task acceptance callbacks to decide if
  /// a task description should be accepted.
  class Confirmation
  {
  public:

    /// Constructor
    Confirmation();

    /// Call this function to decide that you want to accept the task request.
    /// If this function is never called, it will be assumed that the task is
    /// rejected.
    Confirmation& accept();

    /// Check whether
    bool is_accepted() const;

    /// Call this function to bring attention to errors related to the task
    /// request. Each call to this function will overwrite any past calls, so
    /// it is recommended to only call it once.
    Confirmation& errors(std::vector<std::string> error_messages);

    /// Call this function to add errors instead of overwriting the ones that
    /// were already there.
    Confirmation& add_errors(std::vector<std::string> error_messages);

    /// Check the errors that have been given to this confirmation.
    const std::vector<std::string>& errors() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  using ConsiderRequest =
    std::function<void(
        const nlohmann::json& description,
        Confirmation& confirm)
    >;

  FleetUpdateHandle& consider_delivery_requests(
    ConsiderRequest consider_pickup,
    ConsiderRequest consider_dropoff);


  FleetUpdateHandle& consider_cleaning_requests(ConsiderRequest consider);

  FleetUpdateHandle& consider_patrol_requests(ConsiderRequest consider);

  FleetUpdateHandle& consider_composed_requests(ConsiderRequest consider);


  FleetUpdateHandle& add_performable_action(
    const std::string& category,
    ConsiderRequest consider);

  /// Specify a set of lanes that should be closed.
  void close_lanes(std::vector<std::size_t> lane_indices);

  /// Specify a set of lanes that should be open.
  void open_lanes(std::vector<std::size_t> lane_indices);


  bool set_task_planner_params(
    std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> ambient_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> tool_sink,
    double recharge_threshold,
    double recharge_soc,
    bool account_for_battery_drain,
    rmf_task::ConstRequestFactoryPtr finishing_requst = nullptr);

  using AcceptTaskRequest =
    std::function<bool(const rmf_task_msgs::msg::TaskProfile& profile)>;

  using AcceptDeliveryRequest =
    std::function<bool(const rmf_task_msgs::msg::Delivery& request)>;

  /// Specify the default value for how high the delay of the current itinerary
  /// can become before it gets interrupted and replanned. A nullopt value will
  /// allow for an arbitrarily long delay to build up without being interrupted.
  FleetUpdateHandle& default_maximum_delay(
    std::optional<rmf_traffic::Duration> value);

  /// Get the default value for the maximum acceptable delay.
  std::optional<rmf_traffic::Duration> default_maximum_delay() const;

  /// The behavior is identical to fleet_state_topic_publish_period
  [[deprecated("Use fleet_state_topic_publish_period instead")]]
  FleetUpdateHandle& fleet_state_publish_period(
    std::optional<rmf_traffic::Duration> value);

  /// Specify a period for how often the fleet state message is published for
  /// this fleet. Passing in std::nullopt will disable the fleet state message
  /// publishing. The default value is 1s.
  FleetUpdateHandle& fleet_state_topic_publish_period(
    std::optional<rmf_traffic::Duration> value);

  /// Specify a period for how often the fleet state is updated in the database
  /// and to the API server. This is separate from publishing the fleet state
  /// over the ROS2 fleet state topic. Passing in std::nullopt will disable
  /// the updating, but this is not recommended unless you intend to provide the
  /// API server with the fleet states through some other means.
  ///
  /// The default value is 1s.
  FleetUpdateHandle& fleet_state_update_period(
    std::optional<rmf_traffic::Duration> value);

  /// Set a callback for listening to update messages (e.g. fleet states and
  /// task updates). This will not receive any update messages that happened
  /// before the listener was set.
  FleetUpdateHandle& set_update_listener(
    std::function<void(const nlohmann::json&)> listener);

  // Do not allow moving
  FleetUpdateHandle(FleetUpdateHandle&&) = delete;
  FleetUpdateHandle& operator=(FleetUpdateHandle&&) = delete;

  class Implementation;
private:
  FleetUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using FleetUpdateHandlePtr = std::shared_ptr<FleetUpdateHandle>;
using ConstFleetUpdateHandlePtr = std::shared_ptr<const FleetUpdateHandle>;

} // namespace agv
} // namespace rmf_fleet_adapter

#endif // RMF_FLEET_ADAPTER__AGV__FLEETUPDATEHANDLE_HPP

```

`add_robot`

- command
  A reference to a command handle for this robot.
- name
  The name of this robot.
- profile
  The profile of this robot. This profile should account for the largest possible payload that the robot might carry.
- start
  The initial location of the robot, expressed as `Plan::StartSet`. Multiple Start objects might be needed if the robot is not starting precisely on a waypoint.The function `rmf_traffic::agv::compute_plan_starts()` may help with this.
- handle_cb
  This callback function will get triggered when the RobotUpdateHandle is ready to be used by the Fleet API side of the Adapter. Setting up a new robot requires communication with the Schedule Node, so there may be a delay before the robot is ready to be used.

- `add_performable_action`
  Allow this fleet adapter to execute a PerformAction activity of specified
  category which may be present in sequence event.

  - `category`
    A string that categorizes the action. This value should be used when
    filling out the category field in event_description_PerformAction.json
    schema.

  - `consider`
    Decide whether to accept the action based on the description field in
    event_description_PerformAction.json schema.

- `set_task_planner_params`
  Set the parameters required for task planning. Without calling this
  function, this fleet will not bid for and accept tasks.

  - `battery_system`
    Specify the battery system used by the vehicles in this fleet.

  - `motion_sink`
    Specify the motion sink that describes the vehicles in this fleet.

  - `ambient_sink`
    Specify the device sink for ambient sensors used by the vehicles in this fleet.

  - `tool_sink`
    Specify the device sink for special tools used by the vehicles in this fleet.

  - `recharge_threshold`
    The threshold for state of charge below which robots in this fleet
    will cease to operate and require recharging. A value between 0.0 and
    1.0 should be specified.

  - `recharge_soc`
    The state of charge to which robots in this fleet should be charged up
    to by automatic recharging tasks. A value between 0.0 and 1.0 should be
    specified.

  - `account_for_battery_drain`
    Specify whether battery drain is to be considered while allocating tasks.
    If false, battery drain will not be considered when planning for tasks.
    As a consequence, charging tasks will not be automatically assigned to
    vehicles in this fleet when battery levels fall below the
    recharge_threshold.

  - `finishing_request`
    A factory for a request that should be performed by each robot in this
    fleet at the end of its assignments.
