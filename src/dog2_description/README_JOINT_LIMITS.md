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
  - parents all four `*_leg_mount` frames

- `base_link_cad`
  - visual-only CAD shell carrier
  - does not carry trunk inertial or collision

- `base_link_cad_fixed`
  - explicit fixed transform from the semantic trunk root to the CAD shell
  - intentionally non-identity in the current Stage 4 URDF

The semantic `base_link` origin is currently defined near the centroid of the
four leg mounts:

```xml
<xacro:property name="base_link_semantic_origin_x" value="0.248975"/>
<xacro:property name="base_link_semantic_origin_y" value="0.122500"/>
<xacro:property name="base_link_semantic_origin_z" value="0.0"/>
```

This means:

- control / kinematics should reason in the semantic `base_link` frame
- CAD shell placement should be interpreted through `base_link_cad_fixed`
- legacy `urdf_shift_*` properties have been removed from the xacro source
- `base_offset_joint` now explicitly places the semantic `base_link` relative
  to `base_footprint` with:
  - `xyz = (0.2492, 0.12503, -0.2649)`
  - `rpy = (0, 0, pi)`
- the joint name `base_offset_joint` is retained for compatibility, but it is
  no longer a hidden CAD-compensation layer

## Modifying Joint Limits

Edit the joint-limit properties near the top of [`urdf/dog2.urdf.xacro`](./urdf/dog2.urdf.xacro):

```xml
<xacro:property name="hip_lower_limit" value="-2.618"/>
<xacro:property name="hip_upper_limit" value="2.618"/>
<xacro:property name="knee_lower_limit" value="-2.8"/>
<xacro:property name="knee_upper_limit" value="2.8"/>
<xacro:property name="prismatic_effort" value="100"/>
<xacro:property name="prismatic_velocity" value="5"/>
```

Keep limit changes in xacro only. Do not edit generated snapshots by hand.

## Build and Validate

```bash
colcon build --packages-select dog2_description --symlink-install
python3 src/dog2_description/scripts/check_urdf_shift_boundary.py src/dog2_description/urdf/dog2.urdf.xacro
python3 src/dog2_description/scripts/check_joint_semantics.py src/dog2_description/urdf/dog2.urdf.xacro
```

What these checks verify:

- `check_urdf_shift_boundary.py`
  - `base_offset_joint` matches the explicit semantic base placement constants
  - legacy `urdf_shift_*` tokens are absent from the xacro source
  - `base_link` keeps trunk inertial plus the current trunk collision primitive
  - `base_link_cad` keeps the trunk visual shell only
  - `base_link_cad_fixed` matches the current semantic shell offset
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
