# vrpn_client_ros
This for aims at porting the original code from the kinetic-devel branch to ROS2.

## Requirements

The code requires VRPN to work. I fullfilled this requirement by installing the proper ROS1 package:
```
    apt-get install ros-melodic-vrpn
```

## What works?

I only use pose in my project, so I did not port anything else (TF, twist, accel). Also multiple sensors per tracker are not ported.
If there is anyone who would like to use the other features and is willing to test them, I'd be happy to help.
