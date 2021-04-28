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

This chapter describes the requirements and basic steps to integrate hardware with RMF. These include [mobile robots](#mobile-robots), [doors](#doors), [elevators](#elevators) and [workcells](#workcells).
In each section, we will go through how to build and use the necessary ROS 2 packages and interfaces, as well as possible scenarios where such interactions occur.

RMF uses ROS 2 messages and topic interfaces to communicate between different components in the overall RMF system.
In most cases we use components called Adapters to bridge between the hardware-specific interfaces and the general purpose interfaces of RMF.
This chapter will discuss how to develop an RMF Adapter for different types of hardware components.
