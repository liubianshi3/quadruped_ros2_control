# Dog2 Joint Limits and URDF Workflow

## Overview

`dog2_description` now treats [`urdf/dog2.urdf.xacro`](./urdf/dog2.urdf.xacro) as the only authoritative robot description source.
Joint limits, trunk/leg topology, and Gazebo control plugin configuration should all be edited there.

A committed static `dog2.urdf` snapshot is intentionally no longer kept in the repository. When you need an expanded URDF for inspection or external tooling, export it explicitly from the xacro source.

## Source of Truth

- Source: `urdf/dog2.urdf.xacro`
- Runtime: launch files expand xacro directly into `robot_description`
- Snapshot export: run `xacro ... -o /tmp/dog2.urdf` when needed
- Guardrails:
  - `scripts/check_urdf_shift_boundary.py`
  - `scripts/check_joint_semantics.py`

## Current Base Frame Semantics

The current URDF no longer treats `base_link` as a placeholder that happens to
overlap with the CAD shell.

- `base_link`
  - canonical control / dynamics / leg-install root
  - carries trunk inertial
  - carries the current trunk collision primitive
  - carries the trunk visual mesh with its current baked offset
  - parents all four `*_leg_mount` frames

The semantic `base_link` origin is currently defined near the centroid of the
four leg mounts:

```xml
<xacro:property name="base_link_semantic_origin_x" value="0.248975"/>
<xacro:property name="base_link_semantic_origin_y" value="0.122500"/>
<xacro:property name="base_link_semantic_origin_z" value="0.0"/>
```

This means:

- the URDF root is now `base_link`
- control / kinematics should reason in the semantic `base_link` frame
- the trunk mesh is attached directly to `base_link`
- legacy `urdf_shift_*` properties have been removed from the xacro source
- legacy `base_footprint` / `base_offset_joint` / `base_link_cad` are no longer part of the URDF
- if a true planning or ground-projection `base_footprint` is needed later, it
  should be published as a runtime TF outside the URDF

## Modifying Joint Limits

Edit the joint-limit properties near the top of [`urdf/dog2.urdf.xacro`](./urdf/dog2.urdf.xacro):

```xml
<xacro:property name="coxa_lower_limit" value="-2.618"/>
<xacro:property name="coxa_upper_limit" value="2.618"/>
<xacro:property name="femur_lower_limit" value="-2.8"/>
<xacro:property name="femur_upper_limit" value="2.8"/>
<xacro:property name="tibia_lower_limit" value="-2.8"/>
<xacro:property name="tibia_upper_limit" value="2.8"/>
<xacro:property name="prismatic_effort" value="100"/>
<xacro:property name="prismatic_velocity" value="5"/>
```

Keep limit changes in xacro only. `dog2_motion_control` now reads the
authoritative limits from the xacro/URDF path at runtime, so do not maintain a
second rotational-limit source in controller YAML files or generated snapshots.

## Build and Validate

```bash
colcon build --packages-select dog2_description --symlink-install
python3 src/dog2_description/scripts/check_urdf_shift_boundary.py src/dog2_description/urdf/dog2.urdf.xacro
python3 src/dog2_description/scripts/check_joint_semantics.py src/dog2_description/urdf/dog2.urdf.xacro
```

What these checks verify:

- `check_urdf_shift_boundary.py`
  - `base_link` is the only URDF root
  - legacy `base_footprint` / `base_offset_joint` / `base_link_cad` are absent
  - legacy `urdf_shift_*` tokens are absent from the xacro source
  - `base_link` keeps trunk inertial, trunk collision, and the current trunk visual mesh origin
  - leg installation still goes through `*_leg_mount`
  - rail joints still start from those mount frames with zero local origin

- `check_joint_semantics.py`
  - rail / coxa / femur / tibia semantic axes still match the control-side contract

## Export an Expanded URDF Snapshot

```bash
xacro src/dog2_description/urdf/dog2.urdf.xacro \
  controllers_yaml:=src/dog2_description/config/ros2_controllers.yaml \
  -o /tmp/dog2.urdf
```

Use the exported file only as a temporary inspection artifact. If a workflow truly needs a static URDF, regenerate it explicitly from xacro instead of reviving a stale committed copy.

## Runtime Notes

- Gazebo and RViz launch files already use xacro directly.
- `dog2_motion_control` prefers xacro/URDF XML at runtime; `urdf_path` is only an explicit override for pre-expanded snapshots.
- If a test or tool appears to disagree with the xacro source, treat the xacro as authoritative and update the stale consumer.

## Quick Checks After Topology Changes

When trunk or leg-root topology changes, rerun:

```bash
colcon build --packages-select dog2_description dog2_motion_control --symlink-install
python3 src/dog2_description/scripts/check_urdf_shift_boundary.py src/dog2_description/urdf/dog2.urdf.xacro
python3 src/dog2_description/scripts/check_joint_semantics.py src/dog2_description/urdf/dog2.urdf.xacro
python3 -m pytest src/dog2_motion_control/test/test_kinematics.py -x -vv
```

The last command may surface motion-control constants that still need to be realigned with the URDF. Treat that as a consumer-sync issue, not a reason to edit generated URDF snapshots.
