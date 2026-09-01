# vehicle_description

This package contains the model shared by the simulator and physical car:

- `urdf/diff_drive_robot.xacro`: links, joints, collision geometry and the
  `base_footprint → base_link → lidar_link` frame chain.
- `launch/state_publisher.launch.py`: turns a selected Xacro model into
  `robot_description` and publishes the robot TF tree.

Gazebo-only colours, the differential-drive plugin and simulated LiDAR are in
`vehicle_simulation/urdf/diff_drive_robot.gazebo.xacro`.  Keeping them there
means the physical-car launch does not carry simulator behaviour.

The fixed transforms in this package must be measured against the real car
before mapping; a wrong `lidar_link` pose directly distorts a map.
