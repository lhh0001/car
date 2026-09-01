# LiDAR bridge

The LiDAR is connected directly to ESP32 UART2.  The firmware transparently
relays its bytes over TCP to the ROS 2 node, which publishes `/scan` with
`frame_id: lidar_link`.

The default endpoint is `192.168.4.1:8889`.  Launch physical mapping with:

```bash
ros2 launch vehicle_bringup real_mapping.launch.py
```

For a direct USB-TTL diagnostic connection, set `transport: serial` and
`port_name` in `config/lidar_params.yaml`.
