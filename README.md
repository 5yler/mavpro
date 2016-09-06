# mavpro
The mavpro package provides 3D action implementations for UAVs running [mavros](http://wiki.ros.org/mavros).  See [move_base](http://wiki.ros.org/move_base) for the 2D equivalent.

## Actions and Action Servers

The package provides actions for UAV takeoff, landing, and navigating to a local target. All three action servers are launched at once with `actions.launch`. See [actionlib](http://www.ros.org/actionlib) for an in-depth overview of action servers and clients.

### Takeoff

The UAV autonomously arms and takes off until it reaches the desired altitude.

### FlyToLocal

The UAV flies to a local target. The goal position is represented as a [`geometry_msgs/PoseStamped`](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) message. Supports the `mavros` default `fcu` frame, whose origin is the UAV arming location, as well as other frames if the appropriate transform information is available.

### Landing

The UAV lands and disarms.

## Other Nodes

### `utm_fcu_tf_broadcaster`

This node broadcasts a static transform between the origin of the local UTM frame (default: `local_origin` in `mavros`) and the `fcu` frame. 

### `set_stream_rate`

Set rate for all mavros topics. Without this, all flight controller data does not get published on startup[[1](https://github.com/mavlink/mavros/issues/461)][[2](http://forum.erlerobotics.com/t/ros-topics-not-publishing-anything/142)]. 

### `key_pilot`

Python script for remote controlling drone via keyboard input and the  [`setpoint_position`](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.setpoint_position) plugin.



## Limitations

* Currently only supports APM firmware, PX4 is unsupported.
* In order for the takeoff action server to work, RC overrides for the throttle channel are used. The mavros node `system_id` parameter needs to match the value of the `SYSID_MYGCS` parameter on the flight controller[[3](https://github.com/ArduPilot/ardupilot/blob/43712237381aa57e74fdd3b0857a1de0a2ebbb0a/ArduCopter/GCS_Mavlink.cpp#L1113)], or the RC overrides will fail silently. The default value of `system_id` for mavros is `1`. 

