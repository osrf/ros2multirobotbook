# Frequently Asked Questions

### Why is this traffic management system so complicated?

RMF has a number of system design constraints that create unique challenges for
traffic management. The core goal of RMF is to facilitate system integration
for heterogeneous mobile robot fleets that may be provided by different vendors
and may have different technical capabilities.

Vendors tend to want to keep their computing systems independent from other
vendors. Since vendors are often responsible for ensuring uptime and
reliability on their computing infrastructure, they may view it as an
unacceptable liability to share computing resources with another vendor. This
means that the traffic management system must be able to function while being
distributed across different machines on a network.

Different robot platforms may have different capabilities. Many valuable AGV
platforms that are currently deployed are not able to change their itineraries
dynamically. Some AGV platforms can change course when instructed to, as long
as they stick to a predefined navigation graph. Some AMR platforms can
dynamically navigate themselves around unanticipated obstacles in their
environment. Since RMF is meant to be an enabling technology, it is important
that we design a system that can maximize the utility of all these different
types of systems without placing detrimental constraints on any of them.

These considerations led to the current design of distributed conflict
prevention and distributed schedule negotiation. There is plenty of space
within the design to create simpler and more efficient subsets for categories
of mobile robots that fit certain sets of requirements, but these optimizations
can be added later, building on top of the existing completely generalized
framework.

### Who opens and closes doors and operates the lifts? The robot or RMF? Or both?

The responsibility of knowing when a door needs to be opened and then sending
the command to open it belongs to the "fleet adapter". The basic design is:

 * The fleet adapter keeps track of the robot's progress
 * When the robot needs to go through a door, the fleet adapter will recognize this
 * The fleet adapter will send a signal to the door to open
 * Once the door is open, the fleet adapter will command the robot to proceed
 * Once the robot is through the door, the fleet adapter will command the robot wait until the door is closed
 * The fleet adapter will command the door to close
 * Once the door is closed, the fleet adapter will command the robot to proceed

The way a fleet adapter knows about the doors is by parsing the navigation
graph that is provided to it. The navigation graph is a required parameter for
the `full_control` type of fleet adapter. `rmf_demos` shows an example of
providing a navigation graph to the fleet adapter.

The recommended way to construct a navigation graph is to use the
`traffic-editor` tool. The `rmf_demos` repos shows some examples of
`traffic-editor` project files.

However, it's entirely possible to construct your own navigation graphs. They
use YAML format.

### Are lifts supported?

Proper lift support (meaning, specifying an actual lift that can move between
floors, and exporting that information into the navigation graph) is not
something that has been developed yet.

However, for testing and demonstration purposes, there are two special
navigation graph edge properties that can allow a RMF fleet adapter to emulate
lift usage. This is meant for demo scenarios where a "mock lift" has been
created that receives lift commands and transmits lift states but does not
actually move between any different floors in a building. For example, tape
on the floor of a lab to indicate the "lift cabin" box, to allow development
and testing without occupying the actual building lift.

These properties were initially included for demonstration purposes, but they
are proving useful enough that we might make them officially supported
properties. Due to the cost and scarcity of "real" lifts, there seems to be
broad interest in having single-floor hardware test setups that emulate
multi-floor scenarios.

The edge properties are:

 * `demo_mock_floor_name`: The name of the floor that the robot is on while traversing the edge
 * `demo_mock_lift_name`: The name of the lift that is being entered or exited while the robot traverses the edge

The idea is that if you have a single floor demonstration environment but want
to demonstrate interaction with a lift, then you can set up a mock "lift" and
imagine that each side of the "lift" opens to a different floor, and the robot
is only allowed to enter/exit that side of the "lift" when the "lift" believes
it is on that floor. This emulates lift cabins with two sets of doors.

To make this idea more concrete, imagine you have a single-floor hardware
testing area, and a box is drawn on the ground with an LED display next to it
that reads off pretend floor names. The mock lift will transmit lift state
messages that match up with whatever floor the LED is displaying. There is also
some indication of whether the lift doors are open or closed. You can further
imagine that entering or exiting from west side of the "lift" is only allowed
when the lift believes it is on floor L1 whereas entering or exiting the "lift"
from the east side is only allowed when it believes it is on floor L3.

In that setup, for a robot to "correctly" navigate from a waypoint on L1 to a
waypoint on L3, the robot needs to:

 * Approach the "lift" from the west side
 * Call the "lift" down to L1
 * Wait until the lift state has it on floor L1 with the doors open
 * Move into the "lift" (i.e. the box drawn on the ground) and request that it "moves" to L3
 * Wait until the "lift" indicates that it has reached L3 and that its doors are open
 * Exit the "lift" on the east side

A rough ASCII diagram would look like this (numbers are waypoints and letters
are edges):

```
1 <---a---> 2 <---b---> 3
```

 * Waypoint 1 is on floor L1
 * Waypoint 2 is inside the "lift" named LIFT001
 * Waypoint 3 is on floor L3
 * The properties of edge `a `are:
   * bidirectional: true
   * demo_mock_floor_name: L1
   * demo_mock_lift_name: LIFT001
 * The properties of edge `b` are:
   * bidirectional: true
   * demo_mock_floor_name: L3
   * demo_mock_lift_name: LIFT001

### If multiple fleets can do the same task, which one is one chosen?

Though not implemented yet, there is a design worked out for a bidding system
where a task request will be converted to a bid request. The bid request will
be sent to each fleet adapter, and each fleet adapter that can perform the task
will report its best estimate for how soon it would be able to have the task
finished. The fleet adapter that offers the lowest bid will be assigned the
task.

The API and implementation are awaiting finalization of some critical components.

### Can some robots have priority over other robots?

The negotiation system concept does support prioritization for which robot will
accommodate the other robot(s). Any arbitrary metric or weighting system can be
used when resolving a negotiation. But in the current implementation that we
are using, we treat all vehicles as equal and choose the resolution
that minimizes the net delay across all the robots, without any prioritization
or weighting.

Since this codebase is open source, you can easily fork the code and modify it
to use any prioritization system that you'd like. Specifically, replace
`rmf_traffic::schedule::QuickestFinishEvaluator()` with your own
`Negotiation::Evaluator` class that behaves in whatever way you would like.

### What distance is maintained between two robots?

This is configurable. There are two relevant parameters: `footprint_radius` and
`vicinity_radius`. The `footprint_radius` represents an estimate of the
vehicle's physical footprint. The `vicinity_radius` represents an estimate of
the region which the robot needs other vehicles to stay clear of. A "schedule
conflict" is defined as an instance where one vehicle's "footprint" is
scheduled to enter another vehicle's "vicinity". The job of the negotiation
system is to come up with a fix to the schedule that keeps all vehicles'
"footprints" out of all other vehicles' "vicinities".
