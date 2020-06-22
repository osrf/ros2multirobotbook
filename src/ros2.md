# An Introduction to ROS 2

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
the world fully formed, and the same is true for roboticists. It is advisable to
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
  brakes, safety, climate control, etc).

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
