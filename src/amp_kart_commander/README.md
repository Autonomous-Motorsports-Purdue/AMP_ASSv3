# Kart Commander

Kart Commander hosts a Service Server that listens for TrackState changes from the RCS node and manipulates the cmd_vel mux accordingly.

Might consider moving the mux diagram from `bringup/launch/twist_mux` here. There's more elaboration on this node in particular there. Somebody fix the rest,

When "teleop_only" is false and "stop" is false, then both joy and nav2 messages can coexist.
Note that while you can reach `Stop` from any other state, returning to `Autonomus` from `Stop` is not allowed; switch to RC first.

## Tests

See the READMe in the test folder
