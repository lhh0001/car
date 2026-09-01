# Workspace architecture

The workspace is split by responsibility.  Packages communicate through ROS 2
interfaces, not through direct imports or a project-specific “interface file”.

```text
vehicle_description   robot geometry and TF frames
vehicle_simulation    Gazebo worlds, simulation-only plugins and spawning
vehicle_hardware      ESP32 Wi-Fi bridge, encoders, IMU and motor transport
lidar_pkg             TCP/serial LiDAR parsing → /scan
vehicle_mapping       slam_toolbox configuration and saved maps
vehicle_navigation    AMCL, map server and Nav2 configuration
vehicle_control       manual teleoperation and diagnostics
vehicle_bringup       user-facing compositions of the packages above
```

The data path is intentionally standard ROS 2:

```text
/scan (sensor_msgs/LaserScan) ─┐
/odom + odom→base_footprint TF ─┴→ slam_toolbox → /map + map→odom TF

/imu (sensor_msgs/Imu) + encoder odometry → future EKF fusion → improved /odom

/map + /scan + /odom + TF → Nav2 → /cmd_vel (geometry_msgs/Twist)
                                       ↓
                                  ESP32 Wi-Fi bridge → motor driver
```

No `vehicle_interfaces` package is needed yet because every cross-package
message is a ROS standard type.  Add one only when a genuinely new project
message is needed, for example battery state, an e-stop service or driver fault
status.
