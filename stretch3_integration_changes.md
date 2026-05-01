# Stretch3 Integration — Change Log

All changes made to integrate the Hello Robot Stretch 3 into robosuite/robocasa.

---

## 1. NEW FILE — `stretch3_robot.py`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/models/robots/manipulators/stretch3_robot.py`

**Old:** Did not exist.

**New (lines 1–107):**
```python
import numpy as np

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class Stretch3(ManipulatorModel):
    """
    Hello Robot Stretch 3 — a mobile manipulator with a telescoping arm.

    Joint order in robot.xml (18 total, wheels are static visual-only):
      0  joint_lift
      1  joint_arm_l3
      2  joint_arm_l2
      3  joint_arm_l1
      4  joint_arm_l0
      5  joint_wrist_yaw
      6  joint_wrist_pitch
      7  joint_wrist_roll
      8  joint_gripper_slide
      9  joint_gripper_finger_left_open
      10 rubber_left_x
      11 rubber_left_y
      12 joint_gripper_finger_right_open
      13 rubber_right_x
      14 rubber_right_y
      15 joint_head_pan
      16 joint_head_tilt
      17 joint_head_nav_cam
    """

    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/stretch3/robot.xml"), idn=idn)

    @property
    def default_base(self):
        return "NullMobileBase"

    @property
    def default_gripper(self):
        return {"right": None}  # gripper is built into the robot XML

    @property
    def default_controller_config(self):
        return {"right": "default_panda"}

    @property
    def init_qpos(self):
        # lift=0.5m, arm segments each ~0.025m (total ~0.1m), wrist/gripper/head=0
        q = np.zeros(18)
        q[0] = 0.5    # joint_lift
        q[1] = 0.025  # joint_arm_l3
        q[2] = 0.025  # joint_arm_l2
        q[3] = 0.025  # joint_arm_l1
        q[4] = 0.025  # joint_arm_l0
        return q

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.5))

    @property
    def _horizontal_radius(self):
        return 0.4

    @property
    def arm_joints(self):
        # Only include the 6 joints that have corresponding actuators.
        # arm_l1-l3 are tendon-coupled to arm_l0 (all driven by the "arm" actuator).
        # Passive joints (finger, rubber) are excluded — they follow physics.
        p = self.naming_prefix
        return [
            f"{p}joint_lift",
            f"{p}joint_arm_l0",
            f"{p}joint_wrist_yaw",
            f"{p}joint_wrist_pitch",
            f"{p}joint_wrist_roll",
            f"{p}joint_gripper_slide",
        ]

    @property
    def head_joints(self):
        # Exclude joint_head_nav_cam — it's a passive camera mount with no actuator.
        p = self.naming_prefix
        return [
            f"{p}joint_head_pan",
            f"{p}joint_head_tilt",
        ]

    @property
    def arm_type(self):
        return "single"

    @property
    def _eef_name(self):
        return {"right": "right_hand"}
```

---

## 2. NEW FILE — `robot.xml`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/models/assets/robots/stretch3/robot.xml`

**Old:** Did not exist.

**New:** Preprocessed from the source model at
`/home/ros2_stretch/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/dependencies/stretch_mujoco/stretch_mujoco/models/stretch_mj_3.3.0.xml`

Preprocessing transformations applied (in order):

| # | Transformation | Reason |
|---|---|---|
| 1 | Flattened all nested `<default class>` blocks inline onto elements | `robosuite._replace_defaults_inline` only handles top-level defaults |
| 2 | Removed all `childclass="..."` attributes | Left over after defaults removed; caused "unknown default childclass" error |
| 3 | Remapped groups: `2 → 1` (visual), `3 → 0` (collision) | robosuite uses group 0=collision, 1=visual; source used 2/3 |
| 4 | Moved all geom `mass=` values from group=1 to matching group=0 geoms | robosuite's `base.xml` sets `inertiagrouprange="0 0"` — only group=0 contributes to inertia |
| 5 | Added explicit `<inertial>` elements to bodies with joints but no group=0 mass (wheels, gripper_slider, rubber_tips, nav_cam) | MuJoCo 3.x requires mass > mjMINVAL for all moving bodies |
| 6 | Added explicit `<inertial pos="0 0 0" mass="0.15" diaginertia="0.0001 0.0001 0.0001"/>` to static wheel bodies | Same inertiagrouprange issue; static bodies still need declared mass |
| 7 | Removed `<freejoint/>` from `base_link` | robosuite manages root body placement; free joint conflicts with `add_mobile_base()` |
| 8 | Renamed body `link_grasp_center` → `right_hand` | robosuite looks for `_eef_name = {"right": "right_hand"}` |
| 9 | Removed `assetdir="assets"` from `<compiler>`; prepended `assets/` to all mesh/texture `file=` paths | `resolve_asset_dependency` ignores `assetdir`; resolves relative to robot.xml directory |
| 10 | Added explicit `name=` attribute to every nameless `<mesh>` and `<texture>` asset | `add_prefix` prefixes geom `mesh=` refs but skips unnamed assets, causing lookup failures |
| 11 | Removed wheel joints (`joint_mobile_right_wheel`, `joint_mobile_left_wheel`) and their actuators | Wheel joints were classified as base joints (contain "mobile"), creating 5 base joints vs 3 NullMobileBase actuators |
| 12 | Removed `<keyframe>` block | After NullMobileBase merges in 3 more actuators, the keyframe ctrl size (10) became invalid (expected 13) |
| 13 | Added `<site name="right_center" pos="0 0 0.5" size="0.01" rgba="1 0.3 0.3 1" group="2"/>` to `base_link` | Composite controller calls `site_name2id("robot0_right_center")` to locate arm base |
| 14 | Added `assets/` symlink | Points to source model's asset directory for mesh/texture files |

---

## 3. MODIFIED — `manipulators/__init__.py`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/models/robots/manipulators/__init__.py`

**Old line 11 (end of file):**
```python
from .gr1_robot import GR1, GR1FixedLowerBody, GR1ArmsOnly, GR1FloatingBody
```

**New line 12 (added after line 11):**
```python
from .stretch3_robot import Stretch3
```

---

## 4. MODIFIED — `robots/__init__.py`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/robots/__init__.py`

**Old lines 31–32:**
```python
    "PandaDexRH": FixedBaseRobot,
    "PandaDexLH": FixedBaseRobot,
}
```

**New lines 31–33:**
```python
    "PandaDexRH": FixedBaseRobot,
    "PandaDexLH": FixedBaseRobot,
    "Stretch3": WheeledRobot,
}
```

---

## 5. MODIFIED — `null_mobile_base.xml`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/models/assets/bases/null_mobile_base.xml`

**Old lines 9–13:**
```xml
        <body name="support" pos="0 0 0">
            <!-- <joint name="base_joint_z" ... /> -->

            <site name="center" type="sphere" pos="0 0 0" size="0.01" group="1" rgba="0 0 0 0"/>
```

**New lines 9–14 (added line 12):**
```xml
        <body name="support" pos="0 0 0">
            <!-- <joint name="base_joint_z" ... /> -->

            <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
            <site name="center" type="sphere" pos="0 0 0" size="0.01" group="1" rgba="0 0 0 0"/>
```

**Reason:** MuJoCo 3.x requires `mass > mjMINVAL` for any body that has joints. The `support` body has 3 mobility joints but no `<inertial>` element, causing a `ValueError` on model compile.

---

## 6. MODIFIED — `camera_utils.py`

**Path:** `/home/ros2_stretch/robocasa/robocasa/utils/camera_utils.py`

**Old lines 110–112:**
```python
    ### Add robot specific configs here ####
    PandaMobile=dict(),
    GR1FixedLowerBody=dict(),
)
```

**New lines 110–119 (added lines 113–119):**
```python
    ### Add robot specific configs here ####
    PandaMobile=dict(),
    GR1FixedLowerBody=dict(),
    Stretch3=dict(
        robot0_eye_in_hand=dict(
            pos=[0.0, 0.0, 0.0],
            quat=[0, 0.707107, 0.707107, 0],
            parent_body="robot0_right_hand",
        ),
    ),
)
```

---

## 7. MODIFIED — `kitchen.py`

**Path:** `/home/ros2_stretch/robocasa/robocasa/environments/kitchen/kitchen.py`

### Change A — Import (lines 18–22)

**Old line 18:**
```python
from robosuite.models.robots import PandaOmron
```

**New lines 18–22:**
```python
from robosuite.models.robots import PandaOmron
try:
    from robosuite.models.robots import Stretch3
except ImportError:
    Stretch3 = None
```

### Change B — init_qpos setup (lines 580–585)

**Old:** Nothing — no Stretch3 block existed.

**New lines 580–585 (added inside `_setup_model` loop, after PandaOmron block):**
```python
            elif Stretch3 is not None and isinstance(robot.robot_model, Stretch3):
                # lift=0.5m, arm segments ~0.025m each (joints 0-4), wrist/gripper/head=0
                q = np.zeros(18)
                q[0] = 0.5
                q[1] = q[2] = q[3] = q[4] = 0.025
                robot.init_qpos = q
```
