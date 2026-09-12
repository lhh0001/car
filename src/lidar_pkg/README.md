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

To inspect the original USB adapter without starting ROS, run:

```bash
python3 src/lidar_pkg/scripts/lidar_serial_probe.py --port /dev/ttyACM0
```

The probe sends the binary `A5 60` scan command, prints the received bytes in
hexadecimal, and counts checksum-valid `AA 55` packets.
