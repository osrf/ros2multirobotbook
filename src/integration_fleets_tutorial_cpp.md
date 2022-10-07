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

`init_and_make` will initialize rclcpp context and makes an adapter instance.You can add the fleets to be adapted.
takes two arguments

- node_name - The name of rclcpp node that will be produced for this adapter.
- discovery_timeout - The minimum time it will wait to discover the schedule node before giving up. If the default `rmf_utils:nullopt` is used it will use `discovery_timeout`node parameter or wait for 1 minute if`discovery_timeout` is not defined.

`make` instantiates an rclcpp::Node like `init_and_make` however it is more customisable.

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
This function will allow you to add a robot to the fleet.

> Note currently there is no support for deleting the robot

- `command`
  A reference to a command handle for this robot.
- `name`
  The name of this robot.
- `profile`
  The profile of this robot. This profile should account for the largest possible payload that the robot might carry.
- `start` The initial location of the robot, expressed as `Plan::StartSet`. Multiple Start objects might be needed if the robot is not starting precisely on a waypoint.The function `rmf_traffic::agv::compute_plan_starts()` may help with this.
- `handle_cb`
  This callback function will get triggered when the RobotUpdateHandle is ready to be used by the Fleet API side of the Adapter. Setting up a new robot requires communication with the Schedule Node, so there may be a delay before the robot is ready to be used.

`add_performable_action`
Allow this fleet adapter to execute a PerformAction activity of specified
category which may be present in sequence event.

> Note:- It's intentionally left up to the system integrators to decide how to structure your implementation when the user might receive different types of perform_action requests.

- `category`
  A string that categorizes the action. This value should be used when
  filling out the category field in event_description_PerformAction.json
  schema.
- `consider`
  Decide whether to accept the action based on the description field in
  event_description_PerformAction.json schema.

`set_task_planner_params`
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
  When the robot is idle, a timer periodically computes the estimated battery drain if the robot were to head back to the charger now. If the battery level after the estimated drain is within a safety factor applied to the recharge_threshold, the TaskManager will automatically begin a task to take the robot back to its charging station.
  The recharge_threshold is used in a similar manner during task allocation. When deciding the order of tasks performed, the TaskPlanner will estimate the battery level of the robot after performing a task A. If the estimate is below the recharge_threshold, it will then check if the robot can perform task A if it performed a recharge task before this. If this is the case then it will automatically include a recharge task before task A. If it still cannot perform task A even after recharging, it will discard the possibility of doing task A in that sequence.
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

[`RobotUpdateHandle.hpp`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/RobotUpdateHandle.hpp)
The `RobotUpdateHandle` helps in sending updates to the robot.

```cpp
namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// You will be given an instance of this class every time you add a new robot
/// to your fleet. Use that instance to send updates to RoMi-H about your
/// robot's state.
class RobotUpdateHandle
{
public:

  [[deprecated("Use replan() instead")]]
  void interrupted();


  void replan();

  void update_position(
    std::size_t waypoint,
    double orientation);

  void update_position(
    const Eigen::Vector3d& position,
    const std::vector<std::size_t>& lanes);

  void update_position(
    const Eigen::Vector3d& position,
    std::size_t target_waypoint);

  void update_position(
    const std::string& map_name,
    const Eigen::Vector3d& position,
    const double max_merge_waypoint_distance = 0.1,
    const double max_merge_lane_distance = 1.0,
    const double min_lane_length = 1e-8);

  void update_position(rmf_traffic::agv::Plan::StartSet position);

  RobotUpdateHandle& set_charger_waypoint(const std::size_t charger_wp);

  void update_battery_soc(const double battery_soc);

  void override_status(std::optional<std::string> status);

  RobotUpdateHandle& maximum_delay(
    rmf_utils::optional<rmf_traffic::Duration> value);


  rmf_utils::optional<rmf_traffic::Duration> maximum_delay() const;

  /// The ActionExecution class should be used to manage the execution of and
  /// provide updates on ongoing actions.
  class ActionExecution
  {
  public:
    /// Update the amount of time remaining for this action
    void update_remaining_time(rmf_traffic::Duration remaining_time_estimate);

    void underway(std::optional<std::string> text);

    void error(std::optional<std::string> text);


    void delayed(std::optional<std::string> text);


    void blocked(std::optional<std::string> text);

    void finished();

    bool okay() const;

    class Implementation;
  private:
    ActionExecution();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  using ActionExecutor = std::function<void(
        const std::string& category,
        const nlohmann::json& description,
        ActionExecution execution)>;

  /// Set the ActionExecutor for this robot
  void set_action_executor(ActionExecutor action_executor);


  void submit_direct_request(
    nlohmann::json task_request,
    std::string request_id,
    std::function<void(nlohmann::json response)> receive_response);

  /// An object to maintain an interruption of the current task. When this
  /// object is destroyed, the task will resume.
  class Interruption
  {
  public:
    /// Call this function to resume the task while providing labels for
    /// resuming.
    void resume(std::vector<std::string> labels);

    class Implementation;
  private:
    Interruption();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };

  Interruption interrupt(
    std::vector<std::string> labels,
    std::function<void()> robot_is_interrupted);


  void cancel_task(
    std::string task_id,
    std::vector<std::string> labels,
    std::function<void(bool task_was_found)> on_cancellation);

  void kill_task(
    std::string task_id,
    std::vector<std::string> labels,
    std::function<void(bool task_was_found)> on_kill);

  enum class Tier
  {
    /// General status information, does not require special attention
    Info,

    /// Something unusual that might require attention
    Warning,

    /// A critical failure that requires immediate operator attention
    Error
  };

  /// An object to maintain an issue that is happening with the robot. When this
  /// object is destroyed without calling resolve(), the issue will be
  /// "dropped", which issues a warning to the log.
  class IssueTicket
  {
  public:

    /// Indicate that the issue has been resolved. The provided message will be
    /// logged for this robot and the issue will be removed from the robot
    /// state.
    void resolve(nlohmann::json msg);

    class Implementation;
  private:
    IssueTicket();
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
  };


  IssueTicket create_issue(
    Tier tier, std::string category, nlohmann::json detail);

  // TODO(MXG): Should we offer a "clear_all_issues" function?

  /// Add a log entry with Info severity
  void log_info(std::string text);

  /// Add a log entry with Warning severity
  void log_warning(std::string text);

  /// Add a log entry with Error severity
  void log_error(std::string text);

  /// Toggle the responsive wait behavior for this robot. When responsive wait
  /// is active, the robot will remain in the traffic schedule when it is idle
  /// and will negotiate its position with other traffic participants to
  /// potentially move out of their way.
  ///
  /// Disabling this behavior may be helpful to reduce CPU load or prevent
  /// parked robots from moving or being seen as conflicts when they are not
  /// actually at risk of creating traffic conflicts.
  ///
  /// By default this behavior is enabled.
  void enable_responsive_wait(bool value);

  class Implementation;

  /// This API is experimental and will not be supported in the future. Users
  /// are to avoid relying on these feature for any integration.
  class Unstable
  {
  public:
    /// True if this robot is allowed to accept new tasks. False if the robot
    /// will not accept any new tasks.
    bool is_commissioned() const;

    /// Stop this robot from accepting any new tasks. It will continue to
    /// perform tasks that are already in its queue. To reassign those tasks,
    /// you will need to use the task request API to cancel the tasks and
    /// re-request them.
    void decommission();

    /// Allow this robot to resume accepting new tasks.
    void recommission();

    /// Get the schedule participant of this robot
    rmf_traffic::schedule::Participant* get_participant();

    /// Change the radius of the footprint and vicinity of this participant.
    void change_participant_profile(
      double footprint_radius,
      double vicinity_radius);

    /// Override the schedule to say that the robot will be holding at a certain
    /// position. This should not be used while tasks with automatic schedule
    /// updating are running, or else the traffic schedule will have jumbled up
    /// information, which can be disruptive to the overall traffic management.
    void declare_holding(
      std::string on_map,
      Eigen::Vector3d at_position,
      rmf_traffic::Duration for_duration = std::chrono::seconds(30));

    /// Get the current Plan ID that this robot has sent to the traffic schedule
    rmf_traffic::PlanId current_plan_id() const;

    /// Hold onto this class to tell the robot to behave as a "stubborn
    /// negotiator", meaning it will always refuse to accommodate the schedule
    /// of any other agent. This could be used when teleoperating a robot, to
    /// tell other robots that the agent is unable to negotiate.
    ///
    /// When the object is destroyed, the stubbornness will automatically be
    /// released.
    class Stubbornness
    {
    public:
      /// Stop being stubborn
      void release();

      class Implementation;
    private:
      Stubbornness();
      rmf_utils::impl_ptr<Implementation> _pimpl;
    };

    /// Tell this robot to be a stubborn negotiator.
    Stubbornness be_stubborn();

    enum class Decision
    {
      Undefined = 0,
      Clear = 1,
      Crowded = 2
    };

    /// A callback with this signature will be given to the watchdog when the
    /// robot is ready to enter a lift. If the watchdog passes in a true, then
    /// the robot will proceed to enter the lift. If the watchdog passes in a
    /// false, then the fleet adapter will release its session with the lift and
    /// resume later.
    using Decide = std::function<void(Decision)>;

    using Watchdog = std::function<void(const std::string&, Decide)>;

    /// Set a callback that can be used to check whether the robot is clear to
    /// enter the lift.
    void set_lift_entry_watchdog(
      Watchdog watchdog,
      rmf_traffic::Duration wait_duration = std::chrono::seconds(10));

  private:
    friend Implementation;
    Implementation* _pimpl;
  };

  /// Get a mutable reference to the unstable API extension
  Unstable& unstable();
  /// Get a const reference to the unstable API extension
  const Unstable& unstable() const;

private:
  RobotUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

using RobotUpdateHandlePtr = std::shared_ptr<RobotUpdateHandle>;
using ConstRobotUpdateHandlePtr = std::shared_ptr<const RobotUpdateHandle>;

} // namespace agv
} // namespace rmf_fleet_adapter



```

`update_position`
This function updates the position of the robot by specifying -`waypoint`-The waypoint the robot is on.

- `orientation` - Current orientation of the robot.'

  or

- `position` -position of the robot and one or more lanes that the robot is occupying.
- `lanes` - number of lanes(minimum 1 required)

  or

- `position` -position of the robot and one or more lanes that the robot is occupying.
- `target_waypoint` - number of lanes(minimum 1 required)
  This should be used if the robot has diverged significantly from its
  course but it is merging back onto a waypoint.

  or

- `map_name` - The map robot is on
- `position` -position of the robot and one or more lanes that the robot is occupying.
  This option should be used if the robot has diverged significantly from its course.
  The parameters for this function are passed along to `rmf_traffic::agv::compute_plan_starts()`.

  or

- `position` The plan set for the robot

`replan`
The function will tell the RMF schedule that the robot requires a new plan. The new plan will have the starting position of the robot as the last position the robot had updated using `update_position()`.

`set_charger_waypoint`
Set the waypoint where the charger for this robot is located.If not specified, the nearest waypoint in the graph with the is_charger()
property will be assumed as the charger for this robot.

- `charger_wp` The waypoint where the charger for this robot is located.

`update_battery_soc`
Update the current battery level of the robot by specifying its state of charge as a fraction of its total charge capacity.

`override_status` This function overrides the robot status.

- `status`The string provided must
  be a valid enum as specified in the robot_state.json schema.
  Pass std::nullopt to cancel the override and allow RMF to automatically
  update the status. The default value is std::nullopt.

`maximum_delay`
Specify how high the delay of the current itinerary can become before it gets interrupted and replanned. A nullopt value will allow for an
arbitrarily long delay to build up without being interrupted.

`ActionExecution`

- `underways`
  Set task status to error and optionally log a message (error tier)

- `errors`
  Set the task status to delayed and optionally log a message
  (warning tier)
- `delayeds`
  Set the task status to blocked and optionally log a message
  (warning tier)
- `blockeds`
  Trigger this when the action is successfully finished
- `finished`
  Returns false if the Action has been killed or cancelled

`submit_direct_request`

- `task_request` A JSON description of the task request. It should match the task_request.json schema of rmf_api_msgs, in particular it must contain
- `request_id` The unique ID for this task request.
- `receive_response` Provide a callback to receive the response. The response will be a robot_task_response.json message from rmf_api_msgs (note: this message

`ActionExecutor`
Signature for a callback to request the robot to perform an action

- `category`
  A category of the action to be performed

- `description`
  A description of the action to be performed

- `execution`
  An ActionExecution object that will be provided to the user for
  updating the state of the action.

`set_action_executor` Set the ActionExecutor for this robot

`submit_direct_request` Send a request to the robot.

- `task_request` The json task request
- `request_id` The request ID for the task request
- `receive_response` The response callback

`interrupt` Interrupt (pause) the current task, yielding control of the robot away
from the fleet adapter's task manager.

- `labels`
  Labels that will be assigned to this interruption. It is recommended to
  include information about why the interruption is happening.

`cancel_task`
Cancel a task, if it has been assigned to this robot

- `task_id`
  The ID of the task to be canceled

- `labels`
  Labels that will be assigned to this cancellation. It is recommended to
  include information about why the cancellation is happening.

- `on_cancellation`
  Callback that will be triggered after the cancellation is issued.
  task_was_found will be true if the task was successfully found and
  issued the cancellation, false otherwise.

`kill_task` Kill a task, if it has been assigned to this robot

- `task_id`
  The ID of the task to be canceled

- `labels`
  Labels that will be assigned to this cancellation. It is recommended to
  include information about why the cancellation is happening.

- `on_kill`
  Callback that will be triggered after the cancellation is issued.
  task_was_found will be true if the task was successfully found and
  issued the kill, false otherwise.

`create_issue` Create a new issue for the robot.

- `tier`
  The severity of the issue

- `category`
  A brief category to describe the issue

- `detail`
  Full details of the issue that might be relevant to an operator or
  logging system.

[`RobotCommandHandle`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/RobotCommandHandle.hpp)
This class works as a bridge and acts as a handler for all the commands that need to be sent to the robot.

```cpp
namespace rmf_fleet_adapter {
namespace agv {

//==============================================================================
/// Implement this class to receive robot commands from RMF
class RobotCommandHandle
{
public:

  using Duration = rmf_traffic::Duration;

  using ArrivalEstimator =
    std::function<void(std::size_t path_index, Duration remaining_time)>;

  /// Trigger this callback function when the follow_new_path request has been
  /// completed. It should only be triggered that one time and then discarded.
  using RequestCompleted = std::function<void()>;


  virtual void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    ArrivalEstimator next_arrival_estimator,
    RequestCompleted path_finished_callback) = 0;

  /// Have the robot come to an immediate stop.
  virtual void stop() = 0;


  virtual void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) = 0;

  // Virtual destructor
  virtual ~RobotCommandHandle() = default;
};

} // namespace agv
} // namespace rmf_fleet_adapter

```

`ArrivalEstimator`
Use this callback function to update the fleet adapter on how long the
robot will take to reach its next destination.

- `path_index`
  The index of the path element that the robot is currently heading
  towards.

- `remaining_time`
  An estimate of how much longer the robot will take to arrive at
  `path_index`.

`follow_new_path` Have the robot follow a new path. If it was already following a path, then
it should immediately switch over to this one.

> Note:- The waypoints argument that will be given to the follow_new_path function of the RobotCommandHandle contains a list of `rmf_traffic::agv::Plan::Waypoint` objects that have a graph_index function.When that graph_index is non-null it can be passed to `Graph::get_waypoint` and then `Graph::Waypoint::get_map_name` can be called to check what the current map is for that waypoint. If there is a change in map name, then you robot should change its map.

We don't provide an out-of-the-box hook for this because map switching logic is likely to vary too much between robots and deployments. For example, in a certain deployment you might need the robot to decompose a very large floor into multiple maps internally, but inside the RMF traffic system those multiple maps would all be part of one single map. In that case you'd need a dictionary of nav graph index -> robot's internal map name.

- `waypoints`
  The sequence of waypoints to follow. When the robot arrives at a
  waypoint in this sequence, it should wait at that waypoint until the
  clock reaches the time() field of the waypoint. This is important
  because the waypoint timing is used to avoid traffic conflicts with
  other vehicles.

- `next_arrival_estimator`
  Use this callback to give estimates for how long the robot will take to
  reach the path element of the specified index. You should still be
  calling RobotUpdateHandle::update_position() even as you call this
  function.

- `path_finished_callback`
  Trigger this callback when the robot is done following the new path.

You do not need to trigger waypoint_arrival_callback when triggering
this one.

`dock` Have the robot begin a pre-defined docking procedure. Implement this
function as a no-op if your robots do not perform docking procedures.

- `dock_name`
  The predefined name of the docking procedure to use.

- `docking_finished_callback`
  Trigger this callback when the docking is finished.

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
