# Kart Commander Testing

I didn't get around to implementing [launch testing](https://github.com/ros2/launch/tree/rolling/launch_testing) for kart_commander, but here are some manual tests that should be integrated into a future implementation.

## Auto Works

- Launch `Nav2` and `amp_bringup/launch/twist_mux.launch`
- Verify that the default state allows for nav2 commands to show in `cmd_vel`

## RC Works

- Launch `Nav2` and `amp_bringup/launch/twist_mux.launch`
- Send msgs over `joy_teleop`
- Verify that the default state allows teleop to show in `cmd_vel`
- Send `ros2 service call /track_state amp_msgs/srv/TrackState "{state: orange}"` (try `checkered` too)
- Verify that the info output shows the diagnostic from kart_commander that we are in RC state.
- Verify thet nav2 commands no longer show in cmd_vel
- Send msgs over `joy_teleop`
- Verify that the RC statesallows teleop to show in `cmd_vel`

## Emergency Stop Works

- Launch `Nav2` and `amp_bringup/launch/twist_mux.launch`
- Send `ros2 service call /track_state amp_msgs/srv/TrackState "{state: red}"` (try `black` too)
- Verify that the info output shows the diagnostic from kart_commander that we are in STOP state.
- Verify thet nav2 commands don't show in cmd_vel
- Send msgs over `joy_teleop`
- Verify thet joy commands don't show in cmd_vel

# Not allowed to go STOP -> Auto directly

- Following the previous test, send `ros2 service call /track_state amp_msgs/srv/TrackState "{state: green}"` (try `yellow` too)
- Verify that no nav2 comes through 'cmd_vel'
- Switch to Teleop and then back to Auto `ros2 service call /track_state amp_msgs/srv/TrackState "{state: orange}"`, `ros2 service call /track_state amp_msgs/srv/TrackState "{state: green}"`
- - Verify that nav comes through 'cmd_vel'
