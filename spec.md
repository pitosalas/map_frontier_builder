# Map Frontier Builder Specification

## Overview
* An app that will make the robot move in certain ways to achieve coverage of the space and allow a map to be built
* Motion is slow and deliberate
* LIDAR data is used to make sure the robot does not get closer to an obstacle than a certain constant in the config.yaml file
* We access the current map showing the area around the robot
* We determine what parts of the map are as yet unexplored
* We call that the frontier
* We look for a point that can be reached by a straight forward motion that is on the frontier and that is as far as possible
* We rotate the robot so that it is pointed in that direction
* And we command it to move in that direction
* At all times we will not allow the robot to get closer than a constant distance to an obstacle in the direction of travel

## Confirmed Design Decisions

### SLAM and Mapping
* Using slam_toolbox in mapping mode for real-time map building
* Subscribe to `/map` topic (nav_msgs/OccupancyGrid) published by slam_toolbox
* Real-time operation while slam_toolbox is building the map
* Standard ROS2 coordinate frames: `map`, `odom`, `base_link`

### Map Interpretation
* OccupancyGrid values:
  * -1 = unknown/unexplored
  * 0 = free space
  * 100 = occupied
* Frontier = boundary between explored (0 or 100) and unknown (-1)

### Navigation Approach
* Use Nav2 action interface for goal-based navigation
* Set goals just inside known free space near frontier points
* As robot approaches, LIDAR reveals more space and slam_toolbox extends the map
* Iteratively set new goals deeper into newly-revealed areas
* Benefits: Nav2 provides obstacle avoidance, path planning, and recovery behaviors for free
* Nav2 limitation: Can only plan paths through known free space, won't navigate into unmapped regions
* Rely entirely on Nav2's costmap-based obstacle avoidance (no direct LIDAR monitoring needed)

### Motion Strategy (Event-Driven)
* Send Nav2 goal to best frontier point
* Wait for Nav2 action to complete (success/failure/canceled)
* When goal completes, re-evaluate map and pick next best frontier
* Repeat until no more frontiers or user stops
* Simple state machine: picking goal → navigating → goal done → repeat

### Configuration Parameters (config.yaml)
* frontier_search_radius: Maximum distance to search for frontier points from current position
* min_frontier_size: Minimum number of contiguous unexplored cells to consider as valid frontier
* goal_offset_distance: Distance to offset goal point inside known free space from frontier boundary
* Note: Obstacle avoidance distance configured in Nav2 costmap parameters, not here

### Node Architecture
* Single ROS2 node design
* Node name: frontier_explorer
* All functionality (map subscription, frontier detection, Nav2 client, control loop) in one node
* Run standalone with: ros2 run map_frontier_builder frontier_explorer

### Control Flow
* Run continuously and autonomously until terminated
* Termination conditions:
  * No more frontiers found (map exploration complete)
  * User interrupt (Ctrl+C)
  * Optional: Nav2 reports goal failed multiple times consecutively
* Recovery behaviors: Rely on Nav2's built-in recovery (backup, spin, wait behaviors)
* No reachable frontiers: Stop and report exploration complete

## Open Questions

None - specification is complete and ready for implementation.

## Implementation Plan

### Step 1: Basic Map Subscriber
- Create simple ROS2 node
- Subscribe to `/map` topic
- Log map dimensions, resolution when received
- **Validation:** Run with slam_toolbox, verify map data logged

### Step 2: Config File
- Create config.yaml with our 3 parameters
- Load config in node
- Log parameter values
- **Validation:** Check parameters load correctly

### Step 3: Frontier Detection
- Implement algorithm to find frontier cells (unknown adjacent to free)
- Log number of frontier cells found
- **Validation:** Print frontier count, verify it changes as map grows

### Step 4: Frontier Visualization
- Publish visualization markers showing frontiers
- **Validation:** View in RViz, see frontier points highlighted

### Step 5: Best Frontier Selection
- Apply filters (search radius, min size)
- Select farthest reachable frontier
- Log selected frontier coordinates
- **Validation:** Verify selection makes sense in RViz

### Step 6: Nav2 Integration
- Add Nav2 action client
- Send ONE goal to selected frontier
- Log action result
- **Validation:** Watch robot navigate to frontier once

### Step 7: Event-Driven Loop
- On goal completion, repeat frontier selection
- **Validation:** Watch autonomous exploration

### Step 8: Termination & Polish
- Add termination conditions
- Clean shutdown handling
- **Validation:** Verify stops when done or interrupted
