# Route Map Data Requirements for Integration with RMF

## Motivation
RMF uses robot route maps to predict the navigation paths of robots working in the environment. RMF generates path predictions for all active robots in the environment which can be used to proactively avoid conflicts between the various robot path plans. This is often referred to as "traffic management" in RMF. Along with the traffic management, RMF can help enable multi-fleet visualization to building/robot operations staff, improve scheduling of resources (such as lifts and corridors), reduce robot deadlock and more.

Robot route maps in large buildings are complex and may evolve over time in response to customer requests and building renovations. As a result, RMF works best when scripts can automatically import robot route maps, and re-import them in the future after changes are made.

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

## Format Requirements
We can write import scripts to handle virtually any "open" file format that contains the required information. This includes, in order of preference:
  - YAML
  - XML
  - plain text (space or comma-separated ASCII, etc.)
  - DXF
  - DWG
  - SVG

Note that if the map data is provided in textual form, screenshots are helpful for "sanity-checking" the coordinate system and alignment with building features.

## Traffic Editor
If the robot route map does not exist yet, then the [traffic editor tool](./traffic-editor.md) can be used to help create one. The traffic editor tool will also export the route map in an RMF-friendly format.
