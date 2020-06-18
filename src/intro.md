# Introduction

In this section, we shall describe the motivation for ROS 2 and the RMF
system for integrating multiple robots.

# ROS 2

The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it’s all open source.

Before we talk specifically about ROS 2, let's review some history. Since ROS was started in 2007, a lot has changed in the robotics and ROS community. ROS 1, which at that time was just "ROS", began life as the development environment for the Willow Garage PR2 robot. The primary goal was to provide the software tools that users would need to undertake novel research and development projects with the PR2. At the same time, the ROS 1 development team knew that the PR2 would not be the only, or even the most important, robot in the world, and they wanted ROS 1 to be useful on other robots. The original focus was on defining levels of abstraction (usually through message interfaces) that would allow much of the software to be reused elsewhere.

It is fair to say that ROS 1 satisfied the PR2 use case, but also overshot by becoming useful on a surprisingly wide variety of robots. ROS 1 was used on robots that are similar to the PR2, but also on wheeled robots of all sizes, legged humanoids, industrial arms, outdoor ground vehicles (including self-driving cars), aerial vehicles, surface vehicles, and more. In addition, ROS 1 adoption was happening in domains beyond the mostly academic research community that was the initial focus. ROS 1-based products were coming to market, including manufacturing robots, agricultural robots, commercial cleaning robots, and others. Government agencies were also looking more closely at ROS for use in their fielded systems; e.g., NASA expected to be running ROS on the Robonaut 2 that was deployed to the International Space Station.

With all these new uses of ROS, the platform became stretched in unexpected ways. While it was holding up well, the ROS 1 team believed that they could better meet the needs of the broader ROS community by tackling their new use cases head-on. And so ROS 2 was born.

The initial goal of the ROS 2 project was to adapt to the changing landscape, leveraging what is great about ROS 1 and improving what isn’t. But there was also a desire to keep ROS 1 as it existed to keep working and be unaffected by the development of ROS 2. So ROS 2 has been built as a parallel set of packages that can be installed alongside and interoperate with ROS 1 (e.g., through message bridges).

As of this writing, we have reacehd the 13th and last official ROS 1 release, [Noetic Ninjemys](https://www.openrobotics.org/blog/2020/5/23/noetic-ninjemys-the-last-official-ros-1-release), and the first LTS release of ROS 2, [Foxy Fitzroy](https://www.openrobotics.org/blog/2020/6/5/ros-2-foxy-fitzroy-release). :tada:

There is a large and growing amount of ROS 2 resources which can be found on the web. A great place to start for ROS 2 is on the ROS Index page for [ROS 2](https://index.ros.org/doc/ros2/) and farther along in this book in the ROS 2 chapter. Enjoy your journey!

# Robotics Middleware Framework (RMF)

## Motivation for RMF

Imagine with us a world where the incredible robots that are available on the market are now able to co-exist in the same facility; gracefully sharing critical resources such as corridors, elevators/lifts, doors and other infrastructure to enable a more efficient overall system. Imagine integrating an elevator/lift for robots only once and this elevator/lift is now enabled to be used for any robot that needs to use the shared resource in a controlled and safe manner. Imagine a world free of robot deadlocks in a shared corridor. These ideas are achievable today using an amazing system called RMF.

Many companies adopt a strategy of operational efficiency and use the lever of technology, thru the deployment of robots and other smart devices, to help realize their goals. The current generation of robots in production environments today are able to provide services including both bulk and single piece flow delivery, cleaning, disinfecting, security, monitoring, and much more. The diversity in robotic use cases most likely means the best in class robots for each task will come from different robot providers or system integrators. This modern reality makes it critical for a common software framework to be in place in order to manage these heterogeneous resources and to ensure that information is being used effectively from different platforms to promote overall system efficiency.

Without an initial goal and plan for a holistically efficient robotics system, there can be a significant but hidden risk for end users when committing to a single system or platform provider. The hidden "walled gardens" are likely to force an end user to limit their selection of future solutions from that particular provider to minimize opearational risk and avoid redundant integration costs. As the scope and scale of robotic deployments increase this problem is exacerbated, leaving the customer with a perception of no good option except to stay with their current provider.

Beyond the increase cost risk of scaling deployment with different providers, there is also the inherent conflict over shared resources such as elevators, doorways, corridors, network bandwidth, chargers, operations-center screen “real estate,” and human resources such as IT personnel and maintenance technicians. As robotic scale increases, it would become more cumbersome for an operations team to consider managing a large, heterogeneous, multi-vendor robot environment.

These problem statements were the foundational motivations for the development of RMF. To unlock the end user's options and increase robotic system selection ultimately means the entire robotic ecosystem can grow. We can collectively grow the pie.

(insert re: funders and collaborators)

Historically ROS development has focused heavily on the software running on or near individual robots. RMF is designed to operate at a higher abstraction layer to create networked fleets of robots that interoperate with building infrastructure systems, enterprise services, IOT devices, and human interfaces. Unlock your facility and your future with RMF.

## So what is RMF?

First and foremost, RMF is a collection of reusable, sclable libraries and tools building on top of ROS 2 which enable the interoperability of heterogeneous fleets of any type of robotic systems. RMF utilizes standardized communication protocals to infrastructure, environments and automation where robots are deployed to optimize the use of critical resources (i.e. robots, elevators/lifts, doors, passageways, etc). It adds intelligence to the system through resource allocation and by preventing conflicts over shared resources through the RMF Core which will be described in detail later in this book. RMF is flexible and robust enough to operate over virtually any communications layer and integrate with any number of IOT devices. The architecture of RMF is designed in such a way to allow scalability as the level of automation in an environment increases. There are various ways for systems and users to interact with RMF via APIs and customizable user interfaces. Once deployed in an environment, RMF will save cost by allowing resources to be shared and integrations to be minimized. In a nutshell, it is what robotic developers and robot customers have been looking for and here it is:

![RMF Book Architecture Diagram](https://user-images.githubusercontent.com/43839559/84993483-e89e3e80-b17b-11ea-8bc9-0a0d559de3ba.png)

## How does RMF make the magic happen?

One of the principles of RMF's design is to simplify and standardaize messaging as much as possible. You can see in the below diagram of the RMF Core that all of the complex interactions and coordination is currently boiled down to only fourteen standard messages. 

<img src="https://raw.githubusercontent.com/osrf/rmf_core/master/docs/rmf_core_integration_diagram.png">

We will explore each of these functional areas in more detail in later chapters of this book but for now we'd like to also introduce some of the other utilities that you will find helpful when developing and integrating with RMF.

### RMF Demos

[Demonstrations](https://github.com/osrf/rmf_demos) of the capabilities of RMF in various environments. This repository serves as a starting point for working and integrating with RMF.

### Traffic Editor

[Traffic Editor](https://github.com/osrf/traffic_editor) is a GUI for creating and annotating floorplans to be used in RMF. Thru Traffic Editor you are able create traffic patterns to be used in RMF and introduce simulation models to enhance your virtual simulation environments. The `.yaml` files can be easily exported for use in Gazebo.

### Free Fleet

[Free Fleet](https://github.com/osrf/free_fleet) is an open-source robot fleet management system. For those robot developers who do not have their own fleet manager or who would prefer to use and contribute to an open-source fleet management utility.

### Systems of Systems Synthesizer (SOSS)

The [SOSS](https://github.com/osrf/soss) you've been missing! SOSS can be used to easily pass messages between various message formats/types including ROS 1, ROS 2, WebSocket, REST, FiWare, DDS, OPC-UA, and more.

### RMF Schedule Visualizer

This [visualizer](https://github.com/osrf/rmf_schedule_visualizer) is an rviz-based rmf_core visualizer and control panels ("what is it thinking") aimed towards developers

### RoMi Dashboard

The Robotics Middleware for Healthcare (RoMi-H) is a healthcare specific implementation of RMF. This [dashboard](http://github.com/osrf/romi-dashboard) is a web application that provides overall visualization and control over the RoMi-H system. The dashboard is by design more "operator-friendly" compared to the previously mentioned schedule visualizer which is intended to be more functional for RMF or RoMi-H developers.

### Simulation Assets

Open-source and freely distributable [assets](https://app.ignitionrobotics.org/fuel) being created and shared to accelerate simulation efforts.

## Jump in, the water is fine!

So now you have an idea of what RMF is all about, it's time to jump in. We would suggest if you have not already that you take the time to review the [RMF Demos](https://github.com/osrf/rmf_demos) repository and if you want a really quick overview of RMF then take a look at this [Mock Airport Terminal video demo](https://vimeo.com/405803151) (Short film Oscar nominations most welcome). We hope you find RMF to be a useful tool to help you scale your robot deployments and operations and we look forward to the many improvements and contributions to come!
