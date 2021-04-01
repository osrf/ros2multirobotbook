# Tasks in RMF

RMF simplifies task allocation and management across multi-fleet systems.
When a user submits a new task request, RMF will intelligently assign it to the robot in the fleet that can best perform the task. When 


RMF supports three types of task requests out of the box:
* Clean: For robots capable of cleaning floor spaces in facilities
* Delivery: For robots capable of delivering items between locations in facilities
* Loop: For robots capable to navigating back and forth between locations in facilities
> Note: A single robot may be capable of performing one of more of the above tasks and fleet adapters can be configured to reflect the capability of its robots.
For more information on the supported task types, [click here](./task_types.md)


In RMF version 21.04 and above, tasks are awarded to robot fleets based on the outcome of a bidding process that is orchestrated by a Dispatcher node, `rmf_dispatcher_node`.
When the Dispatcher receives a new task request from a dashboard or terminal, it sends out a `rmf_task_msgs/BidNotice` message to all the fleet adapters. If a fleet adapter is able to process that request, it submits a `rmf_task_msgs/BidProposal` message back to the Dispatcher with a cost to accommodate the task. An instance of `rmf_task::agv::TaskPlanner` is used by the fleet adapters to determine how best to accommodate the new request. For more information on the task planner, [click here](./task_planner.md)

The Dispatcher then compares all the `BidProposals` received and submits a `rmf_task_msgs/DispatchRequest` message with the fleet name of the robot that the bid is awarded to. There are a couple different ways the Dispatcher evaluates the proposals such as fastest to finish, lowest cost, etc which can be configured.

Battery recharging is tightly integrated with the new task planner. `ChargeBattery` tasks are optimally injected into a robot's schedule when the robot has insufficient charge to fulfill a series of tasks. Currently we assume each robot in the map has a dedicated charging location as annotated with the `is_charger` option in the traffic editor map.

![RMF Bidding Diagram](images/rmf_core/rmf_bidding.png)