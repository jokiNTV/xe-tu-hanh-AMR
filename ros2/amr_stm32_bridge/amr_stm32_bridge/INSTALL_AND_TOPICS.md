# AMR STM32 bridge diagnostic patch

This patch enables the STM32 `DRIVE_DIAGNOSTIC` stream and publishes three
`geometry_msgs/msg/Vector3Stamped` topics:

- `/amr/drive/measured_wheel_rad_s`
- `/amr/drive/target_wheel_rad_s`
- `/amr/drive/pwm`

For all three topics:

- `vector.x` is the left wheel.
- `vector.y` is the right wheel.
- `vector.z` is unused and remains zero.

The patch also avoids the duplicate `rclpy.shutdown()` exception when the
bridge is stopped by `timeout` or Ctrl+C.

Replace `bridge_node.py` and `protocol.py` in:

`~/amr_ws/src/amr_stm32_bridge/amr_stm32_bridge/`

Then rebuild:

```bash
cd ~/amr_ws
colcon build --packages-select amr_stm32_bridge --symlink-install
source install/setup.bash
```
