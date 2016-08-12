## `takeoff_server`

In order for the takeoff action server to work, RC overrides for the throttle channel are used. The mavros node `system_id` parameter needs to match the value of the `SYSID_MYGCS` parameter on the flight controller[[1](https://github.com/ArduPilot/ardupilot/blob/43712237381aa57e74fdd3b0857a1de0a2ebbb0a/ArduCopter/GCS_Mavlink.cpp#L1113)], or the RC overrides will fail silently. The default value of `system_id` for mavros is `1`. 
