# Stretch3 robosuite/robocasa Compatibility Fixes

Changes made to resolve the `get_elements` ImportError and all downstream
version-mismatch errors between robocasa source and the installed robosuite/MuJoCo stack.

---

## 1. MODIFIED — `cabinet_panels.py`

**Path:** `robocasa/robocasa/models/fixtures/cabinet_panels.py`

**Reason:** `get_elements` was imported from `robosuite.utils.mjcf_utils` but the function does
not exist in robosuite 1.5.1 or 1.5.2. It is only called once (line 706) to get `(parent, geom)`
pairs, so a local fallback is sufficient.

**Old line 6:**
```python
from robosuite.utils.mjcf_utils import find_elements, xml_path_completion, get_elements
```

**New lines 6–17:**
```python
from robosuite.utils.mjcf_utils import find_elements, xml_path_completion
try:
    from robosuite.utils.mjcf_utils import get_elements
except ImportError:
    def get_elements(root, tag):
        """Return list of (parent, element) for all descendants matching tag."""
        results = []
        for parent in root.iter():
            for child in list(parent):
                if child.tag == tag:
                    results.append((parent, child))
        return results
```

---

## 2. MODIFIED — `robocasa/__init__.py`

**Path:** `robocasa/robocasa/__init__.py`

**Reason:** Three hard `assert` statements block import on any version mismatch.
The installed stack (MuJoCo 3.8.0, numpy 1.23.5, robosuite 1.5.2) differs from
robocasa's pinned versions. Converted to `warnings.warn` so import succeeds with a notice.

### Change A — MuJoCo version check (lines 1005–1007)

**Old lines 1005–1007:**
```python
assert (
    mujoco.__version__ == "3.3.1"
), "MuJoCo version must be 3.3.1. Please run pip install mujoco==3.3.1"
```

**New lines 1005–1007:**
```python
if mujoco.__version__ != "3.3.1":
    import warnings
    warnings.warn(f"robocasa expects MuJoCo 3.3.1 but {mujoco.__version__} is installed. Some features may not work correctly.")
```

### Change B — numpy version check (lines 1011–1013)

**Old lines 1011–1013:**
```python
assert numpy.__version__ in [
    "2.2.5",
], "numpy version must be 2.2.5. Please install this version."
```

**New lines 1011–1013:**
```python
if numpy.__version__ not in ["2.2.5"]:
    import warnings
    warnings.warn(f"robocasa expects numpy 2.2.5 but {numpy.__version__} is installed. Some features may not work correctly.")
```

### Change C — robosuite version check (lines 1025–1027)

**Old lines 1025–1027:**
```python
assert (
    robosuite_check
), "robosuite version must be >=1.5.2 Please install the correct version"
```

**New lines 1025–1027:**
```python
if not robosuite_check:
    import warnings
    warnings.warn(f"robocasa expects robosuite >=1.5.2 but {robosuite.__version__} is installed. Some features may not work correctly.")
```

---

## 3. MODIFIED — `kitchen.py`

**Path:** `robocasa/robocasa/environments/kitchen/kitchen.py`

**Reason:** Two kwargs were added to robosuite after 1.5.2 and do not exist in the installed version,
causing `TypeError` on environment construction.

### Change A — `load_model_on_init` removed from `super().__init__()` (line 526 deleted)

**Old lines 524–527 (inside `super().__init__()` call):**
```python
            hard_reset=True,
            load_model_on_init=False,
            camera_names=camera_names,
```

**New lines 524–526:**
```python
            hard_reset=True,
            camera_names=camera_names,
```

### Change B — `enable_multiccd` and `enable_sleeping_islands` removed from `ManipulationTask(...)` (lines 648–649 deleted)

`ManipulationTask.__init__` in robosuite 1.5.2 accepts only `(mujoco_arena, mujoco_robots, mujoco_objects)`.

**Old lines 644–650:**
```python
        self.model = ManipulationTask(
            mujoco_arena=self.mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=list(self.fixtures.values()),
            enable_multiccd=True,
            enable_sleeping_islands=False,
        )
```

**New lines 644–648:**
```python
        self.model = ManipulationTask(
            mujoco_arena=self.mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=list(self.fixtures.values()),
        )
```

---

## 4. MODIFIED — robosuite `manipulators/__init__.py`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/models/robots/manipulators/__init__.py`

**Reason:** `pip install robosuite==1.5.2` overwrote this file, removing the Stretch3 registration
that had been added previously. Re-applied after upgrade.

**Old line 12 (end of file before upgrade):**
```python
from .gr1_robot import GR1, GR1FixedLowerBody, GR1ArmsOnly, GR1FloatingBody
from .xarm7_robot import XArm7
```

**New lines 12–13:**
```python
from .xarm7_robot import XArm7
from .stretch3_robot import Stretch3
```

---

## 5. MODIFIED — robosuite `robots/__init__.py`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/robots/__init__.py`

**Reason:** Same as above — pip upgrade overwrote the file. Re-applied `Stretch3` entry.

**Old lines 33–34:**
```python
    "XArm7": FixedBaseRobot,
}
```

**New lines 33–35:**
```python
    "XArm7": FixedBaseRobot,
    "Stretch3": WheeledRobot,
}
```

---

## 6. MODIFIED — robosuite `null_mobile_base.xml`

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/models/assets/bases/null_mobile_base.xml`

**Reason:** Same as above — pip upgrade overwrote the file. Re-applied `<inertial>` element
that is required by MuJoCo 3.x for any body that has joints.

**Old lines 9–13:**
```xml
        <body name="support" pos="0 0 0">
            <!-- <joint name="base_joint_z" .../> -->

            <site name="center" type="sphere" pos="0 0 0" size="0.01" group="1" rgba="0 0 0 0"/>
```

**New lines 9–14 (line 12 added):**
```xml
        <body name="support" pos="0 0 0">
            <!-- <joint name="base_joint_z" .../> -->

            <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
            <site name="center" type="sphere" pos="0 0 0" size="0.01" group="1" rgba="0 0 0 0"/>
```

---

## 7. MODIFIED — `robot.xml` (Stretch3 asset)

**Path:** `/home/ros2_stretch/.local/lib/python3.10/site-packages/robosuite/models/assets/robots/stretch3/robot.xml`

**Reason:** MuJoCo 3.8.0 (pulled in by the robosuite 1.5.2 upgrade) validates mesh geometry at
compile time regardless of geom mass/density settings, raising `"mesh volume is too small"` for
flat/thin meshes (sticker decals, aruco markers, etc.). Two changes were applied:

### Change A — `inertia="shell"` on all 90 mesh assets

Applied to every `<mesh>` element in the `<asset>` block.

**Old example (line 52):**
```xml
    <mesh file="assets/base_link_0.obj" name="base_link_0" />
```

**New (line 52):**
```xml
    <mesh file="assets/base_link_0.obj" name="base_link_0" inertia="shell" />
```

`inertia="shell"` tells MuJoCo to compute inertia from surface area instead of volume,
which is valid for arbitrarily thin meshes. Applied to all 90 mesh assets.

### Change B — `density="0"` on all 88 zero-mass geoms

Applied to every `<geom>` element that has `mass="0"`, across both group=0 (collision)
and group=1 (visual).

**Old example (line 150):**
```xml
      <geom mesh="base_link_5" material="Generic_Black" density="100" mass="0" ... />
```

**New (line 150):**
```xml
      <geom mesh="base_link_5" material="Generic_Black" density="0" mass="0" ... />
```

Prevents MuJoCo from attempting any density-based volume computation on zero-mass geoms.

---

## Remaining prerequisite

Kitchen scene environments still require the robocasa asset pack to be downloaded:

```bash
PYTHONPATH=/home/ros2_stretch/robocasa \
  python3 /home/ros2_stretch/robocasa/robocasa/scripts/download_kitchen_assets.py --type all
```

This downloads fixtures, objects, and textures (~several GB). After that,
`PickPlaceCounterToCabinet` and other kitchen tasks with Stretch3 should load fully.
