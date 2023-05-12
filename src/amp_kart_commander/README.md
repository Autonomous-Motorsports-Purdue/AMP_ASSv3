# Kart Commander

Kart Commander hosts a Service Server that listens for TrackState changes from the RCS node and manipulates the switch_mux accordingly.

Might consider moving the mux diagram from `bringup/launch/twist_mux` here. There's more elaboration on this node in particular there. Somebody fix the rest,

Note that while you can reach `Stop` from any other state, returning to `Autonomous` from `Stop` is not allowed; switch to RC first.

## `switch_mux`

An one-in, one-out mux for `Twist` messages, since ROS2 does not support generic message types in Foxy.
Publishes to a `Twist` topic named "cmd_vel" and takes in a configurable list of `Twist` topics

- Parameters
  - `initial_topic`: (optional) name of the initial `Twist` topic to pass through
  - `input_topics`: list of `Twist` topics to listen for
- Subscribed Topics
  - `select_topic`: `String` name of topic from `input_topics` to pass through, or `__none`, ehich explicitly turns off the mux
- Published Topics
  - `cmd_vel`: Twist topic output chosen from the inputs, or nothing at all

## Tests

See the README in the test folder
