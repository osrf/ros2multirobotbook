# Hardware

In this chapter, we will describe the process for adding hardware to RMF,
describing how to build the necessary ROS 2 packages and interfaces that
are used by `rmf_core`.

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
