# Fleet Integration

Here we will cover integrating a mobile robot fleet that offers the **Path** control category of fleet adapter. This means we assume the mobile robot fleet manager allows us to specify explicit paths for the robot to follow, and that the path can be interrupted at any time and replaced with a new path. Furthermore, each robot's position will be updated live as the robots are moving.

## Route Map

Before such a fleet can be integrated, you will need to procure or produce a route map as described in the [previous section](./integration_nav-maps.md).