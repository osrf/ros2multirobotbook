# SOSS

This chapter describes the system-of-systems synthesizer (SOSS), a tool which provides protocol translation between different subsystems.
Such composite systems can be called The ROS-SOSS.
To see the current implementation status, see the [SOSS repository](https://github.com/osrf/soss).

## Motivation and Introduction

The ecosystem of different message passing systems is vast and diverse.
Without any one single system that is unanimously considered the best for all applications, we are left to consider how we can tie together disparate message passing systems to bridge the gap between the different kinds of applications that all play a critical role in a modern, intelligent robotics solution.
The best protocols to use for inter-robot communication might not be the best for remote operator communication or for end-user communication.

This creates a scalability problem.
If there are `N` different message passing frameworks being used in a robot deployment with `M` different message types being passed between them, then manually creating inter-compatibility bridges between them could become an `O(MN^2)` complexity problem.
This motivates us to use a highly modular, user-friendly integration system that allows the interoperability to be as automatic as possible for as many different message-passing frameworks as we can tie together.
The `O(MN^2)` problem can be reduced to `O(N)` complexity, where a plugin is written for each `N` framework, and all `M` message types are automatically converted between their `N` different representations.

The integration service we use for this is called System-of-Systems Synthesizer, or SOSS.
The base SOSS package is simply some abstract interfaces defined in a C++ library, along with a single `soss` application.
Each different message-passing system will have its own plugin library, e.g. DDS-SOSS, Websocket-SOSS, ROS-SOSS, that implements the abstract interfaces of the base SOSS.
When you run the `soss` application, you provide a single configuration file that describes how you want your different message-passing systems to bridge with each other.
The `soss` application can then find plugins that meet the requirements of your configuration file and load those plugins as it starts up.
When messages start to move within each message-passing system, the `soss` application will grab, translate, and push the messages across the different system boundaries according to the configuration file that was given to it.
Any number of `soss` instances can be run at once, but they will run independently of each other, so it is important to make sure that their configurations do not overlap.

The base SOSS package also provides some CMake tools to help with auto-generating message definitions for message-passing systems that require static message definitions at compile-time.
For message-passing systems with dynamic message types, the plugin can automatically take care of the translation at runtime, so auto-generation is unnecessary.

For a deeper look at how to use SOSS, we recommend reading [the documentation provided by eProsima](https://integration-service.docs.eprosima.com/en/latest/index.html).
