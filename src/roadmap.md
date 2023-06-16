# Roadmap

This page describes topics that we are currently working on and expect to advance in the next 12 months.
We can’t commit to specific development timelines, but community feedback on which topics are of highest interest is one factor that can help influence the prioritization of our work queue.
As in any R&D project, we will react to how things evolve in many dimensions.

Open-RMF is an open-source project.
We will keep developing in the open so that community members can see real-time what is happening.
Let's work on it together!
We encourage (and really love to see!) contributions from the community in addition to our own efforts, so if you see something that interests you, please [collaborate with us through GitHub](https://github.com/open-rmf/rmf/discussions)!

## Scalability

When it comes to multi-robot coordination, scalability is a complex topic with many separate facets, and every one of those facets is crucial to the overall story.
Concerns about scalability can be viewed in terms of **variables** and their resulting **costs**.

#### Variables
- **A.** Number of mobile robots sharing space
- **B.** Capabilities of each mobile robot
- **C.** Size of the space being shared
- **D.** Severity of narrow passages and bottlenecks
- **E.** System network topology (LAN, wired, Wi-Fi, remote/cloud)

#### Costs
- **1.** Computing resources (CPU load, RAM)
- **2.** Time to find solutions
- **3.** Quality of solutions (how close to being globally optimal, [Pareto efficient](https://en.wikipedia.org/wiki/Pareto_efficiency), or another criteria)
- **4.** Quality of operation (responsiveness, smooth execution)

This table breaks down ongoing efforts to improve scalability, specifying which variables will scale better and which resulting costs will be improved. See below the table for detailed descriptions of each improvement.

<!--
    It would be nice to have supercolumns for Variables and Costs in this table,
    but that is not currently supported by mdBook: https://github.com/rust-lang/mdBook/issues/1916
-->
|Improvement               | A | B | C | D | E | 1 | 2 | 3 | 4 |
|--------------------------|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| Reservation System       |✅ |   |   |   |   |✅ |✅ |✅ |✅ |
| Queuing System           |✅ |   |   |✅ |   |✅ |✅ |✅ |✅ |
| Hierarchical Planning    |✅ |   |✅ |✅ |   |✅ |✅ |   |   |
| Custom Negotiation       |   |✅ |✅ |✅ |   |   |   |✅ |   |
| Zenoh                    |✅ |   |   |   |✅ |✅ |✅ |   |✅ |

#### Reservation System

Initially described [here](https://github.com/open-rmf/rmf_traffic/issues/12) the reservation system would assign relevant resources to robots in a globally optimal way. Originally conceived for parking spaces, it will also be applicable for elevator and workcell usage.

#### Queuing System

Initially described [here](https://github.com/open-rmf/rmf_ros2/issues/214), when multiple robots need to use one resource in a sequence (e.g. passing through a door), the queuing system will manage a rigidly defined queuing procedure instead of relying on the traffic negotiation system to resolve the use of the narrow corridor.

#### Hierarchical Planning

Hierarchical traffic planning would decompose the overall planning space into regions separated by chokepoints. Each chokepoint would be managed by a queue. The traffic negotiation system would only account for traffic conflicts that happen while robots transition from one queue to another queue. Reducing the scope of conflicts will make negotiations faster, less frequent, and less CPU intensive.

#### Custom Negotiation

Not all mobile robots or uses cases will fit neatly into the three out-of-the-box categories of "Full Control", "Traffic Light", and "Read-Only". Letting system integrators fully customize the way the fleet adapter plans and negotiates for their robot will allow Open-RMF to support more types of robots with better outcomes.


#### Zenoh

There are various inefficiencies in the ROS 2 implementation of Open-RMF related to DDS discovery mechanisms and the absence of message key support within ROS 2. Using Zenoh, either directly or as an RMW implementation with ROS 2, would allow for significantly more efficient communication, especially over Wi-Fi networks and for large numbers of agents and subsystems.

## Tools

Managing and understanding large-scale complex systems requires tools that are able to provide comprehensive insight into how the system is operating and why. We are pushing forward efforts to develop the [Site Editor](https://github.com/open-rmf/rmf_site) into a baseline for a visualization, event simulation, and digital twin tool that can help developers and end-users alike see what is going in an Open-RMF system (whether simulated or deployed) and why. The planned capabilities are:

* Large-scale event-driven simulation (simulating hundreds of mobile robots and other Open-RMF systems in real time or faster-than-real time)
* Digital twin 3D view of a live deployment, with visualizations of decisions that are being made
* Debugging tools to look into plans and negotiations, with visualizations that explain why each solution was produced

## Capabilities

To expand the scope of where and how Open-RMF can be used, we are also planning new capabilities:

* Free space traffic negotiation
* Describing tasks by drawing behavior diagrams
* Support multi-agent tasks with parallel threads of activity
* Allow constraints to be defined between tasks

## Developer Experience

For wider adoption and quicker deployment, we want to improve the developer experience for system integrators.

* Simpler APIs that allow greater customization of robot behaviors
* Easier integration options, such as a "native" integration with the ROS 2 Nav Stack
    * We will provide a ROS 2 nav stack plugin that gives Open-RMF compatibility to any robot using it
* More documentation, including updating this book
* Interactive tutorials in the site editor
