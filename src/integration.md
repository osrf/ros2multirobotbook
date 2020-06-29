<!-- # Requirements -->

<!-- robot, door, lift, workcell, etc. integration with RMF

    I have a door door
    I have an elevator / I have a lift arrow_up_down
    I have a workcell robot mechanical_arm
    I have a loose mobile robot and would like to use FreeFleet (F5)
        robot runs ROS 1
        robot runs ROS 2
        robot runs something that I wrote
        robot runs something somebody else wrote and I can't change
    I have some mobile robots with their own fleet manager(s)
        it has a REST API or some other formal API (XMLRPC)
        it has some other communication mechanism (SQL database, etc.) -->

# Integration

In this chapter, we will describe the integration requirements and basic steps to have hardware working with RMF. These include [mobile robots](#mobile-robots), [doors](#doors), [elevators](#elevators) and [workcells](#workcells). In each section, we will go through how to build the necessary ROS 2 packages and interfaces that are used by `rmf_core`, as well as possible scenarios where such interactions occur.

In general, all the interactions can be summed up with this system architecture diagram,

![RMF Diagram](https://raw.githubusercontent.com/osrf/rmf_core/master/docs/rmf_core_integration_diagram.png)

RMF uses ROS 2 messages and topic interfaces. Hence, in most cases we use components called Adapters to bridge between the hardware interfaces and RMF with some exception for robot fleets which also have fleet drivers as part of the process. Robot fleets will be further elaborated in the next few sections.
