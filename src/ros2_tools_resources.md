## Meta-discussion of the tools and resources available to help you in the learning process.

In this chapter we will cover the basics of Robot Operating System (ROS) and
give you all the tools you need to build, debug, and understand robotic
applications. This chapter is laid out from the most general concepts, necessary
for decision makers to make sound decisions, to specific API references needed
by engineers to develop new robotic applications. Somewhere in between high
level concepts, and low level API commands lives the knowledge necessary for
those maintaining and supporting multi-robot deployments in the field. 

A good analogy to learning about ROS is the process of learning about motor
vehicles. At the practical, day-to-day level, most people will learn how to
start a vehicle and safely use it on a motorway. For these individuals, learning
about the high level concepts behind ROS, along with application-specific
commands is probably sufficient. Those who enjoy driving often choose to learn
how to repair and maintain their vehicle. If this is your interest level we
recommend learning the basics of the ROS command line interface. This will allow
you to "check the oil" of your robotic systems and make sure everything is
functioning correctly. Finally, if you are the type that would like to swap out
the engine of your vehicle with something more powerful, or potentially build a
wholly new vehicle from scratch, then the ROS API is the set of tools that will
make this possible. Generally speaking, automotive engineers don't appear into
the world fully formed, and the same is true for robotocists. It is advisable to
work through each phase of understanding as you develop your skills with ROS.  

Following from our analogy above the process of learning how to use robotic
systems built on ROS can be divided roughly into four parts. This chapter works
through these four parts of the process, using ROS 2. Subsequent chapters then
build upon this knowledge and discuss the subtleties of specific
applications. The four parts of this chapter are as follows.

* Meta-discussion of the tools and resources available to help you in the
  learning process.
  
* A high level discussion to the design patterns use in ROS. These patterns are
  roughly analogous to the subsystems you would find in a vehicle (engine,
  breaks, safety, climate control, etc). 
  
* A treatment of the command line interface (CLI) for ROS. The CLI is a set of
  programs for starting, inspecting, controlling, and monitoring a ROS
  robot. You can think of this topic as teaching you how check a robot's oil,
  and read the instrument panel. 

* An introduction to the ROS application programming interface. This section
  will show you how to create your own applications and modify existing software
  for your specific application. 
  
While this books aims to cover the basics it should be made clear that ROS, like
almost all software is a moving target. Technology moves quickly, and while
print media is helpful and delivering high fidelity instruction, that
instruction can become rapidly outdated. For this reason we start this chapter
with a meta-discussion of ROS resources that can be used to help you in your
learning process. 

== ROS Resources == 

The most up to date to date information about ROS can be found on the web and
there are a myriad of resources on-line to help you out in your educational or
practical journey. One thing to keep in mind is that ROS, like most software,
has different versions, and the format and structure of commands and API calls
may differ slightly between versions (although the developers try to keep things
as stable as possible). This book is specifically written for _ROS 2, Eloquent
Elusor_, or ROS Eloquent to be terse. While newer or older versions of ROS will
be generally helpful it is worth paying attention to the version number as there
can be minor changes between versions. ROS has both major versions (i.e. ROS
1, and ROS 2) and minor versions denoted by a pair of matching letter adjectives
and specific nouns related to specific genus and species of turtles
(e.g. Eloquent Elusor, or Foxy Fitzroy). The biggest difference in the CLI and
API come between the major versions, i.e. ROS 1 and ROS 2. There may be minor
changes between minor versions, and usually it is the addition of features, not
their modification or removal. It is worth noting that ROS versions are usually
pegged to specific version of Ubuntu Linux. If your search engine results are
specific enough for your particular problem it is a good practice to append your
ROS version to your search. Moreover, when seeking help or posting questions
online you should always specify the version of ROS version you are using. 


ROS grew up with the modern web, and as such it has a variety of tools and
forums to help you solve problems and learn about the API and tools. Some of our
web resources actually pre-date more widely used systems, so it helps to know
where they are and how to use them. Probably the most important resource on the
web for ROS users is [answers.ros.org](http://anwers.ros.org). Answers is a Q&A
website similar to StackOverflow. Once you register for Answers you can ask or
answer any question ROS related. Be aware that asking a question well can be
difficult. You should include as much information as possible to help others
answer your question. This means you should include the ROS version, any
debugging or stack trace information you have, and the offending source code. 

Aside from ROS Answers you should check out both the ROS 2 tutorials and API
documentation and the ROS 1 wiki. The ROS 1 wiki can be found at
[<http://wiki.ros.org/>](http://wiki.ros.org/) and while it is specifically
dedicated to ROS 1, much of the information is still relevant to ROS 2. If you
are searching for up to date ROS 2 information your go to source for this
information is the ROS 2 tutorials and API documents located at 
[<https://index.ros.org/doc/ros2/>](https://index.ros.org/doc/ros2/). Many of
the tutorials you will find in this book pull directly from this body of
work. If you would like to find the latest ROS news and discuss various ROS
features the ROS Discourse forum at
[<https://discourse.ros.org/>](https://discourse.ros.org/) is your best bet. ROS
discourse is the community hub where developers discuss their latest projects
and debate the finer points of ROS development. 


For ROS application developers there are number of tools to help you connect
with the broader ROS developer community. Open Robotics supports
[index.ros.org](https://index.ros.org/) which is an extended list of ROS
packages sorted by version. If you are searching for a ROS driver for a
particular piece of hardware then the index is a great place to start. If you
find a package with failing tests, or would like to know the build status of any
ROS package at [build.ros.org](http://build.ros.org/). Similarly, for un-indexed
packages [GitHub maintains a ROS code
tag](https://github.com/topics/ros?o=desc&s=updated). This tag will allow you to
search all of the tagged repositories that are publicly listed. At the time of
writing there were close to 4000 repositories listed on github, so there is a
pretty good chance you'll find what you need. 


There are also a variety of unofficial ROS resources 

-   The ROS / Robotics Sub Reddits are Great!
-   There is an "unofficial" [ROS
    Discord](https://discord.com/invite/HnVcz5a).
    -   Please try using ROS Answers first.
-   We have a yearly ROS developers conference
    [ROSCon.](https://roscon.ros.org/2020/)
    -   Most of old talks are free on the web.
-   We're not big on social media but we're busy on the twitter.
    -   [@OpenRoboticsOrg](https://twitter.com/openroboticsorg) is a bit
        more active.
    -   [@ROSOrg](https://twitter.com/rosorg) "Official" ROS
        announcements.
-   [Open Robotics](https://www.openrobotics.org/) is the non-profit
    that administers ROS and Ignition Gazebo.
    -   We take donations and take contract work from time to time.

