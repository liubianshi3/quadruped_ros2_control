# Gazebo ROS 2 Control Configuration Note

## Current Rule

The authoritative Gazebo control plugin configuration lives only in [`urdf/dog2.urdf.xacro`](./urdf/dog2.urdf.xacro).
Launch files expand that xacro directly into `robot_description`; the repository no longer carries a committed static `dog2.urdf` snapshot as a second source of truth.

## Canonical Plugin Block

```xml
<gazebo>
  <plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>${controllers_yaml}</parameters>
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
  </plugin>
</gazebo>
```

## Why This Matters

If the plugin block in `dog2.urdf.xacro` drifts from the controller config path, Gazebo may spawn the robot but fail to initialize the controllers correctly.

## Inspecting the Expanded URDF

If you need to inspect the final XML that Gazebo sees, export a fresh snapshot from xacro instead of looking for a checked-in `dog2.urdf`:

```bash
xacro src/dog2_description/urdf/dog2.urdf.xacro \
  controllers_yaml:=src/dog2_description/config/ros2_controllers.yaml \
  -o /tmp/dog2.urdf
```

## Related Files

- `src/dog2_description/urdf/dog2.urdf.xacro`
- `src/dog2_description/config/ros2_controllers.yaml`
- launch files that populate `robot_description` from xacro
