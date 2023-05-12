# Local Goal

Greedy goal setting algorithm based on maximizing open space in a `CostMap`.

We don't know ahead of time what the track looks like, so the kart has to compute its next waypoints in real time. Since this stack currently
does not perform accurate global mapping, goal selection is limited to local obstacle data. An extra difficulty is that the track is not fully enclosed by
obstacles but weakly defined by cones (which have large gaps in them). So a free space finding algo, which is not easily affected by gaps, is our solution.

## Files

`amp_local_goal/follow_waypoint.py`: _Node and algorithm implementation_
`params/local_goal_params.yaml`: _Parameters for the algo with descriptions_

## Parameters

See `local_goal_params.yaml`

## Topics

When a costmap is received, the most recent value of `odom` is used without further synchronization.

Subscribed

- `/local_costmap/costmap_raw`: nav2_msgs/CostMap: Costmap used for goal selection, including metadata on position and scaling

- `/odom`: nav2_msgs/Odometry: Used to find the kart's current yaw

- `/initial`: nav2_msgs/PoseWithCovarianceStamped: Used to find the initial yaw, must be available before algo can start

Published

- `/goal_pose`:
