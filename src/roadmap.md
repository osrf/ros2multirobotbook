# Roadmap

This list describes topics, in no particular order, that we are currently working on and expect to advance in the next 12 months.
We can’t commit to specific development timelines, but community feedback on which topics are of highest interest is one factor that can help influence the prioritization of our work queue.
As in any R&D project, we will react to how things evolve in many dimensions.

RMF is an open-source project.
We will keep developing in the open so that community members can see real-time what is happening.
Let's work on it together!
We encourage (and really love to see!) contributions from the community in addition to our own efforts, so if you see something that interests you, please collaborate with us through GitHub!

## Topics under active or near-future development:

* Deployment Environment
    * add [Red Hat Enterprise Linux 8 (RHEL 8)](https://www.redhat.com/en/enterprise-linux-8) support, including binary packaging
    * Add examples and tooling to simplify instantiating and interacting with RMF running in AWS using WireGuard tunnels to local robots and doors/lifts/chargers
* `rmf_core`
    * A turnkey free space planning library that's compatible with the traffic management utilities using [`RobotFleetAdapterHandle`](https://github.com/osrf/rmf_core/blob/master/rmf_fleet_adapter/include/rmf_fleet_adapter/agv/FleetUpdateHandle.hpp) (not restricting to [`rmf_fleet_msgs`](https://github.com/osrf/rmf_core/tree/master/rmf_fleet_msgs/msg) anymore) which incorporates all predicted future motions of other robots
    * "Valet": dynamic allocation of "parking spaces" for idle robots
A reusable library for distributed (multi-fleet) task dispatching based on customizable evaluation criteria
    * Live task management: support cancellation of running tasks
    * Many improvements to runtime efficiency, latency, memory usage, etc.
* User Interface Toolkit: [`rmf-web`](https://github.com/osrf/rmf-web)
    * a browser-based [tiling/mosaic](https://github.com/nomcopter/react-mosaic) type user interface framework and a mechanism to create pre-arranged and user-customized panel collections, much like [`rviz`](http://wiki.ros.org/rviz) and similar configurable visualization-engine systems
    * a collection of [React](https://reactjs.org/) components for this tiling/mosaic framework:
        * 2D map views of current and scheduled robot motions
        * tables showing statuses of robots / lifts / doors / chargers
        * task creation, monitoring, and cancellation
        * emergency alarm monitoring and (for testing) simulated activation
        * and many more!
    * a backend in [FastAPI](https://fastapi.tiangolo.com/) to feed the frontend and bridge it to the ROS 2 nodes
    * a user management system with roles, based on [Keycloak](https://www.keycloak.org/)
    * examples of how to deploy this UI toolkit to build end-to-end applications
    * log management frontend and backend
    * Infrastructure adapters
        * an intelligent lift (elevator) adapter that can optimally coordinate how robots share lifts (with each other and with humans)
        * documented examples of using OPC-UA [PLC's](https://en.wikipedia.org/wiki/Programmable_logic_controller) to control infrastructure such as doors and lifts.
    * Chargers
        * integration with multi-robot charger hardware
        * iteration of `rmf_charger_msgs` as needed
        * a negotiation system to dynamically allocate multi-robot chargers on demand
    * Robot fleet adapters
        * adding new types of robots as relevant to our sponsors
        * customized fleet adapters and map ingestion scripts as needed
        * continual improvement to the open-source [MiR fleet adapter](https://github.com/osrf/fleet_adapter_mir)
        * improving existing robot fleet adapters and generalizing/refactoring those improvements as possible into the open-source libraries
    * Documentation
        * Improve package-level documentation
        * Improve the long-form narrative documentation in The Book
    * Testing
        * Continue to improve unit and end-to-end tests of all components (this task never ends)

## Wish list

Beyond that list of topics, we have a giant and ever growing "wish list" of things to work on, pending time and resource availability!
These include (in no particular order):
* browser-based traffic editor and backend extensions as needed
* improvements in how maps are ingested and aligned from existing robot systems
* dynamic traffic lane availability (add/remove lane segments from the planning graphs)
* dynamic temporary traffic blockout zones/areas for parallel activities (i.e. zone-based cleaning robot activities, temporary construction, etc.)
* many more web UI panels
* [FreeFleet](https://github.com/osrf/free_fleet) demonstrations
* FreeFleet clients in as many other robotics frameworks as possible
* drastic simulation appearance improvement
    * improve world-creation tools (either inside traffic editor or outside of it) using new features in Ignition Gazebo for physically-based rendering (PBR) texture support, lighting models, and so on.
