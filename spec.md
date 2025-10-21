# Map Frontier Builder Specification

## Overview
Autonomous frontier exploration system that enables a robot to systematically explore and map unknown environments. The system identifies unexplored areas (frontiers), clusters them to find significant regions, and navigates to the center of the largest cluster for efficient exploration.

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
* frontier_search_radius: Maximum distance to search for frontier points from current position (5.0m)
* min_frontier_size: Minimum number of contiguous unexplored cells to consider as valid frontier (10 cells)
* goal_offset_distance: Distance to offset goal point inside known free space from frontier boundary (0.5m)
* max_translation_speed: Maximum linear velocity (0.2 m/s)
* max_rotation_speed: Maximum angular velocity (0.5 rad/s)
* nav_to_target: Boolean flag - if true, automatically navigate to frontiers; if false, visualize only
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

### Frontier Selection Strategy
* Groups frontier cells into clusters using 8-connectivity BFS algorithm
* Selects the largest contiguous frontier cluster (most unexplored area)
* Targets the center point of the largest cluster for balanced exploration
* Benefits: Prioritizes large unexplored regions over small isolated pockets

### Visualization
* Publishes MarkerArray to `/frontier_markers` topic
* Cyan points: All detected frontier cells
* Orange sphere: Selected target frontier (center of largest cluster)
* Target sphere is larger (0.3m minimum, 5x map resolution) and elevated (z=0.1) for visibility

## Implementation Status

### ✅ Completed Steps

**Step 1-2: Basic Map Subscriber & Config**
- ROS2 node subscribes to `/map` topic
- Config file loads all parameters
- Logs map dimensions and config values

**Step 3: Frontier Detection**
- Detects frontier cells (unknown cells adjacent to free space)
- Uses 8-connectivity to check neighbors
- Logs frontier cell count

**Step 4: Frontier Visualization**
- Publishes cyan point markers for all frontiers
- Publishes orange sphere marker for selected target
- Visible in RViz on `/frontier_markers` topic

**Step 5: Best Frontier Selection**
- Implements frontier clustering using BFS
- Selects center of largest cluster as target
- Improved from simple "first frontier" to intelligent cluster-based selection

**Step 6: Nav2 Integration**
- Nav2 action client integrated
- Sends NavigateToPose goals to selected frontiers
- Tracks navigation state (navigating flag)
- Handles goal responses and results

### 🚧 Remaining Work

**Step 7: Event-Driven Loop**
- Currently sends goal on each map update if not navigating
- Need to add proper event-driven loop that waits for goal completion
- Should re-evaluate frontiers only after navigation completes

**Step 8: Termination & Polish**
- Add "no frontiers found" detection
- Implement clean shutdown
- Add recovery behavior for failed navigation
- Test complete exploration scenarios

## Open Questions

* Should search_radius filter be applied to limit frontier distance from robot?
* How to handle navigation failures (retry same frontier, skip to next, abort)?
* Should min_frontier_size filter out small clusters before selection?
