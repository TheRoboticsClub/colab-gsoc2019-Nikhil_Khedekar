---
layout: post
title: Why do we fail(safe)?
---

The logical method for commanding the drone to take off is to use the [service call provided by MAVROS](http://wiki.ros.org/mavros#mavcmd) or the publishing local setpoints at an altitude offset as suggested in the [MAVROS OFFBOARD example](https://dev.px4.io/en/ros/mavros_offboard.html). Once we have the drone in the air, we are required to command the velocities (linear and angular) of the drone in the [Body Fixed NED Frame(FRAME_BODY_NED)](https://github.com/mavlink/mavros/blob/master/mavros_msgs/srv/SetMavFrame.srv). Surprisingly, when using any of the two above methods for takeoff, on switching to using `cmd_vel` (Twist) setpoints, the FCU automatically goes into the failsafe mode citing:

```bash
WARN  [commander] Failsafe enabled: no RC and no offboard
[ERROR] [1559902872.143876063, 35.976000000]: FCU: Failsafe enabled: no RC and no offboard
[ INFO] [1559902872.201660093, 36.028000000]: FCU: Failsafe mode activated
```

What's even more surprising is that this persists even on disabling the failsafes for Data Link and RC Link Loss:

```bash
[ WARN] [1559755500.317984418, 809.804000000]: PR: Param NAV_DLL_ACT (359/565): <value><i4>0</i4></value> different index: 365/571
[ WARN] [1559755500.321128431, 809.808000000]: PR: Param NAV_RCL_ACT (380/565): <value><i4>0</i4></value> different index: 386/571
[ WARN] [1559755500.324533916, 809.812000000]: PR: Param NAV_DLL_ACT (359/565): <value><i4>0</i4></value> different index: 372/578
[ WARN] [1559755500.324615102, 809.812000000]: PR: Param NAV_RCL_ACT (380/565): <value><i4>0</i4></value> different index: 393/578
INFO  [logger] Start file log (type: full)
[ INFO] [1559755500.338754612, 809.828000000]: FCU: ARMED by Arm/Disarm component command
INFO  [logger] Opened full log file: ./log/2019-06-05/17_25_00.ulg
[ INFO] [1559755500.390418762, 809.880000000]: FCU: [logger] file: ./log/2019-06-05/17_25_00.ulg
[ INFO] [1559755500.482441549, 809.972000000]: FCU: Failsafe mode deactivated
INFO  [commander] Takeoff detected
[ INFO] [1559755504.168664354, 813.656000000]: FCU: Takeoff detected
WARN  [commander] Failsafe enabled: no RC and no offboard
[ERROR] [1559755514.411656119, 823.892000000]: FCU: Failsafe enabled: no RC and no offboard
```
