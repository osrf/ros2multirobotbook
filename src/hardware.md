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

# Hardware

In this chapter, we will describe the integration requirements and steps to have hardware working with RMF. These include [robots](#robots), [doors](#doors), [elevators](#elevators) and [workcells](#workcells). In each section, we will go through how to build the necessary ROS 2 packages and interfaces that are used by `rmf_core`, as well as possible scenarios where such interactions are occur.

## Robots

We have identified a number of different scenarios where 

Robots in industrial settings are often controlled and monitored as a fleet, with a central fleet management system keeping things in check. 

## Doors

## Elevators

## Workcells

# Map data requirements for integration with RoMi-H / rmf_core

## Motivation
RoMi-H / RMF uses robot route maps to be able to predict the future motions of robots in the building. RoMi-H generates "multi-fleet" predictions which can be used to help avoid conflicts between robot fleets and individual robots, provide multi-fleet visualization to building staff, and improve scheduling of resources, among other benefits.

Robot route maps in large buildings are complex and may evolve over time in response to customer requests and building renovations. As a result, RoMi-H works best when scripts can automatically import robot route maps, and re-import them in the future after changes are made.

## Minimum Map Information Required
- list of waypoints or nodes
  - name of waypoint
  - level name (B1, L1, L2, etc.)
  - (x, y) location in meters within the level
  - any special properties or flags, such as:
    - is this a dropoff/pickup parking point?
    - is this a charger?
    - is this a safe parking spot during an emergency alarm?
- list of edges or "travel lanes" between nodes
  - (start, end) waypoint names
  - two-way or one-way traffic?
    - if one-way, identify direction of travel
  - any other information, such as speed limit along this segment

## Format requirements
We can write import scripts to handle virtually any "open" file format that contains the required information. This includes, in order of preference:
  - YAML
  - XML
  - plain text (space or comma-separated ASCII, etc.)
  - DXF
  - DWG
  - SVG

## Comments
If the map data is provided in textual form, screenshots are helpful for "sanity-checking" the coordinate system and alignment with building features.
