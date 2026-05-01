from robocasa.environments.kitchen.kitchen import *
import numpy as np


class FindAndOpenDrawer(Kitchen):
    """
    The robot spawns at a random fixture far from the target drawer.
    It must navigate to find the drawer (based on simulated camera visibility)
    and open it.

    Uses a scripted policy (run_scripted_policy) for automated testing:
      Phase 1 — explore: random walk until the drawer enters the robot's FOV
      Phase 2 — approach: drive toward the drawer and align heading
      Phase 3 — open: issue arm extension + pull-back actions

    Args:
        drawer_id: FixtureType id for the drawer to target (default: TOP_DRAWER)
        nav_fov_deg (float): half-angle of the simulated camera FOV in degrees
        approach_dist (float): stop driving forward when closer than this (metres)
    """

    def __init__(
        self,
        drawer_id=FixtureType.TOP_DRAWER,
        nav_fov_deg=40.0,
        approach_dist=0.65,
        *args,
        **kwargs,
    ):
        self.drawer_id = drawer_id
        self.nav_fov_rad = np.deg2rad(nav_fov_deg)
        self.approach_dist = approach_dist
        super().__init__(*args, **kwargs)

    # ------------------------------------------------------------------
    # scene setup
    # ------------------------------------------------------------------

    def _setup_kitchen_references(self):
        super()._setup_kitchen_references()

        self.drawer = self.register_fixture_ref("drawer", dict(id=self.drawer_id))

        # Spawn robot at a fixture that is at least 1.5 m from the drawer.
        # init_robot_base_ref must be a string key from self.fixtures.
        drawer_xy = np.array(self.drawer.pos[:2])
        candidates = []
        for name, f in self.fixtures.items():
            if isinstance(f, (Wall, Floor, Accessory, WallAccessory)):
                continue
            if not isinstance(f, Fixture):
                continue
            try:
                dist = np.linalg.norm(np.array(f.pos[:2]) - drawer_xy)
                if dist > 1.5:
                    candidates.append(name)
            except Exception:
                pass
        if candidates:
            start_name = candidates[self.rng.integers(len(candidates))]
        else:
            start_name = list(self.fixtures.keys())[0]
        self.init_robot_base_ref = start_name

    def _setup_scene(self):
        self.drawer.close_door(env=self)
        super()._setup_scene()

    def get_ep_meta(self):
        ep_meta = super().get_ep_meta()
        ep_meta["lang"] = "Find the drawer and open it."
        return ep_meta

    def _get_obj_cfgs(self):
        return []

    def _check_success(self):
        door_state = self.drawer.get_door_state(env=self)
        return all(v >= 0.90 for v in door_state.values())

    # ------------------------------------------------------------------
    # helpers used by the scripted policy
    # ------------------------------------------------------------------

    def _get_base_state(self):
        """Return (xy_pos, yaw) of the mobile base in world frame."""
        bid = self.sim.model.body_name2id("mobilebase0_support")
        pos = np.array(self.sim.data.body_xpos[bid])
        mat = np.array(self.sim.data.body_xmat[bid]).reshape(3, 3)
        yaw = np.arctan2(mat[1, 0], mat[0, 0])
        return pos[:2], yaw

    def _get_drawer_world_pos(self):
        """Return the world-frame xy centre of the drawer body."""
        body_name = self.drawer.name + "_main"
        bid = self.sim.model.body_name2id(body_name)
        return np.array(self.sim.data.body_xpos[bid])[:2]

    def _get_nav_target(self):
        """
        Compute the world-frame (xy, yaw) the robot should stand at to open
        the drawer — i.e. directly in front of the drawer at interaction range.
        Uses the same placement logic as ManipulateDrawer._place_robot().
        Returns (target_xy, target_yaw) or falls back to drawer centre if it fails.
        """
        x_ofs = (self.drawer.width / 2) + 0.20
        for sign in (1, -1):
            try:
                pos, ori = EnvUtils.compute_robot_base_placement_pose(
                    self, ref_fixture=self.drawer, offset=(sign * x_ofs, -0.10)
                )
                return np.array(pos[:2]), ori[2]
            except Exception:
                pass
        return self._get_drawer_world_pos(), 0.0

    def _drawer_in_fov(self):
        """True when the drawer is within nav_fov_rad of the robot's heading."""
        robot_pos, robot_yaw = self._get_base_state()
        drawer_pos = self._get_drawer_world_pos()
        to_drawer = drawer_pos - robot_pos
        dist = np.linalg.norm(to_drawer)
        if dist < 0.1:
            return True, dist
        angle_to_drawer = np.arctan2(to_drawer[1], to_drawer[0])
        angle_error = (angle_to_drawer - robot_yaw + np.pi) % (2 * np.pi) - np.pi
        in_fov = abs(angle_error) < self.nav_fov_rad
        return in_fov, dist


def _warp_robot(env, nav_xy, nav_yaw, verbose=False):
    """
    Teleport the robot to (nav_xy, nav_yaw) in world frame.

    The fwd/side slide joints have axes fixed in the PARENT body frame, not the
    current body frame.  After the explore phase (yaw spinning), body_yaw differs
    from the parent-frame angle θ_parent = initial_body_yaw.  Using body_yaw for
    the axis directions gives the wrong qpos deltas.  We must use θ_parent instead:

        θ_parent  = body_yaw - qpos_yaw    (invariant under spinning)
        fwd axis  → world (cos θ_parent, sin θ_parent)
        side axis → world (-sin θ_parent, cos θ_parent)

    Procedure:
      1. Warp yaw: delta to qpos_yaw so body_yaw → nav_yaw.  The hinge pivot is
         offset 0.21 m from the body origin, which shifts the body position.
      2. Re-read body position after the yaw warp and apply qpos deltas using the
         fixed parent-frame axes to reach nav_xy.
    """
    fwd_addr = env.sim.model.get_joint_qpos_addr("mobilebase0_joint_mobile_forward")
    side_addr = env.sim.model.get_joint_qpos_addr("mobilebase0_joint_mobile_side")
    yaw_addr = env.sim.model.get_joint_qpos_addr("mobilebase0_joint_mobile_yaw")

    # Read current state before any changes
    cur_pos, cur_yaw = env._get_base_state()
    cur_yaw_qpos = env.sim.data.qpos[yaw_addr]
    # Parent-frame yaw: fixed regardless of how much the robot has spun
    theta_parent = cur_yaw - cur_yaw_qpos

    # Step 1: Warp yaw — Z hinge is world-aligned so delta is exact
    yaw_delta = (nav_yaw - cur_yaw + np.pi) % (2 * np.pi) - np.pi
    env.sim.data.qpos[yaw_addr] += yaw_delta
    env.sim.forward()

    # Step 2: Correct XY using fixed parent-frame axes
    new_pos, _ = env._get_base_state()
    dx = nav_xy[0] - new_pos[0]
    dy = nav_xy[1] - new_pos[1]
    cp, sp = np.cos(theta_parent), np.sin(theta_parent)
    env.sim.data.qpos[fwd_addr] += cp * dx + sp * dy
    env.sim.data.qpos[side_addr] += -sp * dx + cp * dy
    env.sim.forward()

    if verbose:
        final_pos, final_yaw = env._get_base_state()
        print(
            f"Warp: target=({nav_xy[0]:.3f},{nav_xy[1]:.3f}) yaw={np.rad2deg(nav_yaw):.1f}° "
            f"→ got=({final_pos[0]:.3f},{final_pos[1]:.3f}) yaw={np.rad2deg(final_yaw):.1f}°"
        )


def run_scripted_policy(env, max_steps=300, verbose=False):
    """
    Scripted policy for FindAndOpenDrawer — for automated headless testing.

    The kitchen obstacle layout makes full navigation unreliable across random
    layouts.  This policy therefore:
      1. Explore: spins in place until the drawer enters the robot's FOV
         (demonstrates the "find" phase with real kinematics).
      2. Warp: teleports the robot to the correct interaction stance in front
         of the drawer (bypasses obstacle navigation for reliable testing).
      3. Open: extends arm toward the drawer handle then pulls back to open it.

    For interactive use, run the demo script instead:
        python3 robocasa/demos/demo_kitchen_scenes.py \\
            --robot Stretch3 --task FindAndOpenDrawer

    Base action indices (verified empirically for Stretch3 in kitchen):
      action[8]  = body-forward velocity  (joint_mobile_forward)
      action[9]  = body-side velocity     (joint_mobile_side)
      action[10] = yaw rate               (counterclockwise positive)

    Returns True if the drawer was successfully opened.
    """
    assert isinstance(env, FindAndOpenDrawer), "env must be a FindAndOpenDrawer instance"

    BASE_YAW = 10

    env.reset()

    # ── Phase 1: Explore — spin until drawer enters FOV ───────────────
    EXPLORE_STEPS = 200
    found = False
    for step in range(EXPLORE_STEPS):
        in_fov, dist = env._drawer_in_fov()
        if verbose:
            print(f"step={step:3d}  phase=explore   dist={dist:.2f}  in_fov={in_fov}")
        if in_fov:
            found = True
            break
        action = np.zeros(env.action_dim)
        action[BASE_YAW] = 0.7
        env.step(action)

    if verbose:
        if found:
            print(f"Drawer found in FOV at step {step}")
        else:
            print("Drawer not found during explore; proceeding to warp anyway")

    # ── Phase 2: Warp — place robot at interaction stance ─────────────
    nav_xy, nav_yaw = env._get_nav_target()
    _warp_robot(env, nav_xy, nav_yaw, verbose=verbose)

    # Arm extension direction: robot's +y body axis in world = (-sin θ, cos θ).
    # compute_robot_base_placement_pose places the robot so the drawer is on this side.
    arm_dx = -np.sin(nav_yaw)
    arm_dy = np.cos(nav_yaw)

    if verbose:
        eef = env.sim.data.get_site_xpos("gripper0_right_grip_site")
        drawer = env._get_drawer_world_pos()
        print(f"EEF={eef[:2]}  drawer={drawer}  arm_dir=({arm_dx:.2f},{arm_dy:.2f})")

    # ── Phase 3: Open — extend arm then pull back ─────────────────────
    for open_step in range(max_steps):
        action = np.zeros(env.action_dim)
        if open_step < 50:
            # Extend toward drawer handle
            action[0] = arm_dx * 0.7   # world x
            action[1] = arm_dy * 0.7   # world y
            action[2] = -0.1           # slight downward
        else:
            # Pull back to drag drawer open
            action[0] = -arm_dx
            action[1] = -arm_dy

        obs, reward, done, info = env.step(action)

        if env._check_success():
            if verbose:
                print(f"Success at open_step {open_step}!")
            return True

    if verbose:
        print("Open phase exhausted without success.")
    return False
