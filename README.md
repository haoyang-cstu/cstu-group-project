# cstu-group-project
City Drive Simulation (integrated with ros2_control)

This folder contains the city drive simulation integrated into the `cstu-group-project` repository.

**Architecture**: Uses the same ros2_control structure as `ros2_ws/src/six_link_gazebo_demo`:
- **URDF**: `urdf/diffbot.urdf.xacro` - Robot model with ros2_control interfaces
- **Controllers**: `config/ros2_controllers.yaml` - Diff-drive controller with joint_state_broadcaster
- **World**: `worlds/city.world` - Yellow lane with turn and red octagonal stop signs

The simulation features a white car that auto-drives down the yellow lane with a right turn, stops for 2 seconds when stop signs are detected via OpenCV, then resumes.

**Features:**
- **Auto-drive**: Differential drive controller auto-moves on launch
- **Stop sign detection**: Red octagonal stop signs detected via OpenCV (red color + octagon shape)
- **Curved lane**: Yellow lane with right turn
- **Live visualization**: Camera feed, CV debug output, and detection mask displayed on launch

Build & run (from the package folder):

```bash
cd ros2_ws/cstu-group-project
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
ros2 launch cstu_group_project sim_launch.py
```

**Visualization for presentations:**
Three image windows will automatically open:
- `/camera/image_raw` - Live camera feed from the car
- `/stop_detection_debug` - Camera with detected stop signs highlighted (red) and all red contours (green)
- `/stop_detection_mask` - Binary mask showing red regions detected

**Nodes:**
- `image_processor`: Subscribes to `/camera/image_raw`, detects stop signs (red + octagonal), publishes `/stop_detected`
- `motion_controller`: Subscribes to `/stop_detected`, publishes `/cmd_vel` (auto-drive 0.5 m/s, stops 2s on detection)
- `robot_state_publisher`: Loads URDF and publishes robot description
- `controller_manager` (spawner): Loads ros2_control diff_drive_controller and joint_state_broadcaster

**Controllers:**
- `diff_drive_controller`: Drives left/right wheels, reads `/cmd_vel` (Twist) messages
- `joint_state_broadcaster`: Publishes joint states to `/joint_states`
