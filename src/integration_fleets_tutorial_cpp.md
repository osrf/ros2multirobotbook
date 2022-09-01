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

[`make`](https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/Adapter.hpp#L75) instantiates an rclcpp::Noe like `init_and_make` however it is more customisable.

- node_name - The name of rclcpp node that will be produced for this adapter.
-
- discovery_timeout - The minimum time it will wait to discover the schedule node before giving up. If the default `rmf_utils:nullopt` is used it will use `discovery_timeout`node parameter or wait for 1 minute if`discovery_timeout` is not defined.
