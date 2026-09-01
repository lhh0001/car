# vehicle_bringup

This package is the scenario-composition layer.  It owns no robot model,
Gazebo world, SLAM or Nav2 parameter file; it only combines the relevant
feature packages into user-facing commands.

| Command | Composed functions |
|---|---|
| `diff_drive_sim.launch.py` | Gazebo + SLAM + optional manual control |
| `diff_drive_nav.launch.py` | Gazebo + online SLAM + Nav2 |
| `diff_drive_nav_map.launch.py` | Gazebo + saved map/AMCL + Nav2 |
| `real_mapping.launch.py` | real robot TF + ESP32 bridge + LiDAR + SLAM |

The common and environment-specific parameter layers now live with the code
that consumes them:

| Function | Common layer | Simulation overlay | Physical-car overlay |
|---|---|---|---|
| 2D SLAM | `vehicle_mapping/config/slam_mapping.yaml` | `slam_sim.yaml` | `slam_real.yaml` |
| Nav2 | `vehicle_navigation/config/nav2_params.yaml` | `nav2_sim.yaml` | `nav2_real.yaml` |

`vehicle_hardware/config/motor_wifi.yaml` deliberately remains in the hardware
package: encoder CPR, wheel dimensions and motor tuning are physical-car data,
not navigation-algorithm data.
