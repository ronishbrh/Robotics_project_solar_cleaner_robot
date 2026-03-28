"""
pybullet_environment.py
Solar Panel Cleaning Robot — Environment only.
No autonomous logic here.
"""

import pybullet as p
import pybullet_data
import math
import os


class SolarPanelEnvironment:
    """
    Creates a 3x4 solar panel array and loads the robot URDF.

    Parameters
    ----------
    gui            : show PyBullet GUI window
    urdf_path      : path to solar_robot_pybullet.urdf  (None = no robot)
    panel_tilt_deg : tilt of every panel around Y-axis  (0 = flat)
    """

    PANEL_LENGTH = 1.60  # metres along X (robot drives in X direction)
    PANEL_WIDTH = 1.00  # metres along Y
    PANEL_THICKNESS = 0.05
    PANEL_GAP = 0.10  # gap between rows (X)
    PANEL_COL_GAP = 0.10  # gap between columns (Y)

    def __init__(self, gui=True, urdf_path=None, panel_tilt_deg=0.0):
        self.physics_client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setPhysicsEngineParameter(enableConeFriction=False)

        self.panel_tilt_deg = float(panel_tilt_deg)
        self.panel_ids = []  # list of dicts
        self.dirt_map = {}  # (row,col) -> [body_id, ...]
        self.cleaned_set = set()
        self.joint_indices = {}
        self.robot_id = None
        self.cup_links = []

        # Ground plane
        p.loadURDF("plane.urdf")

        self._setup_camera()
        self._build_panels()
        self.panel_positions = {(d["row"], d["col"]): d["pos"] for d in self.panel_ids}

        if urdf_path and os.path.exists(urdf_path):
            self._load_robot(urdf_path)
        elif urdf_path:
            print(f"[ENV] URDF not found: {urdf_path}")

        p.setTimeStep(1.0 / 240.0)
        p.setRealTimeSimulation(0)

        print(
            f"[ENV] Ready — {len(self.panel_ids)} panels, "
            f"tilt={self.panel_tilt_deg:.1f}°, "
            f"robot={'yes' if self.robot_id is not None else 'no'}"
        )

    # ------------------------------------------------------------------ #
    #  Camera                                                              #
    # ------------------------------------------------------------------ #
    def _setup_camera(self):
        p.resetDebugVisualizerCamera(
            cameraDistance=5.5,
            cameraYaw=40,
            cameraPitch=-30,
            cameraTargetPosition=[2.0, 0.0, 0.5],
        )

    # ------------------------------------------------------------------ #
    #  Panel array                                                         #
    # ------------------------------------------------------------------ #
    def _build_panels(self):
        tilt = math.radians(self.panel_tilt_deg)
        r_step = self.PANEL_LENGTH * math.cos(tilt) + self.PANEL_GAP
        c_step = self.PANEL_WIDTH + self.PANEL_COL_GAP
        ht = self.PANEL_THICKNESS / 2

        # Surface normal for suction (rotated around Y by tilt)
        self.panel_normal = (
            math.cos(tilt + math.pi / 2),
            0.0,
            math.sin(tilt + math.pi / 2),
        )
        print(f"asporpweoripoweirsldPanel normaa: {self.panel_normal}")

        for row in range(3):
            for col in range(4):
                cx = row * r_step
                cy = col * c_step - 1.5 * c_step
                # raise each successive row by its height on the slope
                cz = ht + 0.002 + cx * math.tan(tilt)
                orn = p.getQuaternionFromEuler([0, -tilt, 0])

                hl, hw = self.PANEL_LENGTH / 2, self.PANEL_WIDTH / 2
                cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hl, hw, ht])
                vs = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[hl, hw, ht],
                    rgbaColor=[0.07, 0.07, 0.38, 1.0],
                    specularColor=[0.5, 0.65, 0.8],
                )
                pid = p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=cs,
                    baseVisualShapeIndex=vs,
                    basePosition=[cx, cy, cz],
                    baseOrientation=orn,
                )
                # Smooth glass-like surface
                p.changeDynamics(
                    pid,
                    -1,
                    lateralFriction=0.4,
                    spinningFriction=0.001,
                    rollingFriction=0.0001,
                )
                self.panel_ids.append(
                    dict(id=pid, row=row, col=col, pos=[cx, cy, cz], cleaned=False)
                )

    def _load_robot(self, urdf_path):
        tilt = math.radians(self.panel_tilt_deg)
        col0_y = -1.80 * (self.PANEL_WIDTH + self.PANEL_COL_GAP)
        p0 = self.panel_ids[0]["pos"]

        # Spawn robot above panel (0,0), aligned to panel slope
        spawn = [
            p0[0] - 0.55 * math.cos(tilt),
            col0_y,
            p0[2] + 0.40 * math.sin(tilt) + 0.16,
        ]

        orn = p.getQuaternionFromEuler([0, -tilt, math.pi/2])

        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=spawn,
            baseOrientation=orn,
            useFixedBase=False,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )

        nj = p.getNumJoints(self.robot_id)
        print(f"[ENV] Robot joints ({nj}):")
        for i in range(nj):
            info = p.getJointInfo(self.robot_id, i)
            name = info[1].decode()
            jtype = info[2]
            self.joint_indices[name] = i
            tstr = {0: "revolute", 1: "prismatic", 4: "fixed"}.get(jtype, "?")
            print(f"  [{i:2d}] {name:<42} ({tstr})")

        # for i in range(nj):
        #     jtype = p.getJointInfo(self.robot_id, i)[2]
        #     if jtype in (0, 1):  # revolute or prismatic
        #         p.setJointMotorControl2(
        #             self.robot_id, i, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0
        #         )

        # for name, idx in self.joint_indices.items():
        #     if p.getJointInfo(self.robot_id, idx)[2] == 1:
        #         p.resetJointState(self.robot_id, idx, 0.0)

        idx = self.joint_indices.get("lift_column_joint", -1)
        if idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=0,
                force=300,
                maxVelocity=0.1,
            )

        for name in (
            "wheel_fl_joint",
            "wheel_fr_joint",
            "wheel_rl_joint",
            "wheel_rr_joint",
        ):
            idx = self.joint_indices.get(name, -1)
            if idx < 0:
                print(f"  [WARN] wheel joint not found: {name}")
                continue
            # Remove all joint damping so wheels spin freely
            p.changeDynamics(
                self.robot_id,
                idx,
                lateralFriction=1.2,
                rollingFriction=0.02,
                spinningFriction=0.01,
            )
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.VELOCITY_CONTROL,
                targetVelocity=0.0,
                force=1200,  # keep same force
            )

        cup_names = [
            "front_left_pad",
            "front_right_pad",
            "rear_left_pad",
            "rear_right_pad",
        ]
        for i in range(nj):
            lname = p.getJointInfo(self.robot_id, i)[12].decode()

            if lname in cup_names:
                print("Found cup")
                self.cup_links.append(i)
                p.changeDynamics(
                    self.robot_id,
                    i,
                    lateralFriction=1.2,
                    spinningFriction=0.05,
                    rollingFriction=0.05,
                )

        # for _ in range(400):
        #    p.stepSimulation()

        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        ncon = len(p.getContactPoints(self.robot_id))
        print(f"[ENV] Settled at {[round(v,3) for v in pos]}, contacts={ncon}")

    def add_dirt_patches(self):
        dirty = [(0, 0), (0, 2), (1, 1), (1, 3), (2, 0), (2, 2), (2, 3)]
        tilt = math.radians(self.panel_tilt_deg)
        for row, col in dirty:
            panel = next(
                d for d in self.panel_ids if d["row"] == row and d["col"] == col
            )
            pp = panel["pos"]
            blobs = []
            for ox, oy in [(-0.28, -0.14), (0.06, 0.22), (0.26, -0.12)]:
                wx = pp[0] + ox * math.cos(tilt)
                wy = pp[1] + oy
                wz = pp[2] + ox * math.sin(tilt) + self.PANEL_THICKNESS / 2 + 0.04
                vis = p.createVisualShape(
                    p.GEOM_SPHERE, radius=0.06, rgbaColor=[0.48, 0.30, 0.16, 0.92]
                )
                bid = p.createMultiBody(
                    baseMass=0, baseVisualShapeIndex=vis, basePosition=[wx, wy, wz]
                )
                blobs.append(bid)
            self.dirt_map[(row, col)] = blobs
        print(f"[ENV] Dirt added on {len(dirty)} panels")

    def clean_panel(self, row, col):

        key = (row, col)
        if key in self.cleaned_set:
            print(f"[ENV] Panel ({row},{col}) already clean")
            return
        for bid in self.dirt_map.get(key, []):
            try:
                p.removeBody(bid)
            except Exception:
                pass
        self.cleaned_set.add(key)
        for d in self.panel_ids:
            if d["row"] == row and d["col"] == col:
                d["cleaned"] = True
        total = len(self.panel_ids)
        print(
            f"[ENV] Panel ({row},{col}) cleaned " f"[{len(self.cleaned_set)}/{total}]"
        )

    def detect_gap(self, range_m=0.3):
        lidar_link_name = "lidar"
        lidar_index = -1

        # Find lidar
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[12].decode("utf-8") == lidar_link_name:
                lidar_index = i
                break

        if lidar_index == -1:
            return False, range_m

        # Get lidar position
        link_state = p.getLinkState(self.robot_id, lidar_index)
        lidar_pos = link_state[0]

        # ✅ USE PANEL NORMAL (THIS IS THE FIX)
        nx, ny, nz = self.panel_normal

        # Shoot INTO the panel (negative normal)
        ray_dir = [-nx, -ny, -nz]

        ray_start = lidar_pos
        ray_end = [
            lidar_pos[0] + ray_dir[0] * range_m,
            lidar_pos[1] + ray_dir[1] * range_m,
            lidar_pos[2] + ray_dir[2] * range_m,
        ]

        result = p.rayTest(ray_start, ray_end)
        hit_body = result[0][0]
        hit_frac = result[0][2]

        # Ignore self hits
        if hit_body == self.robot_id:
            hit_body = -1

        # Ground or nothing = gap
        is_gap = hit_body == -1 or hit_body == 0

        # Debug line
        color = [1, 0, 0] if is_gap else [0, 1, 0]
        p.addUserDebugLine(ray_start, ray_end, color, lifeTime=0.05)

        if is_gap:
            return True, range_m
        else:
            return False, hit_frac * range_m

    def nearest_panel(self):

        if self.robot_id is None:
            return (0, 0)
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        best, best_d = (0, 0), 1e9
        for (r, c), pp in self.panel_positions.items():
            d = (pos[0] - pp[0]) ** 2 + (pos[1] - pp[1]) ** 2
            if d < best_d:
                best_d, best = d, (r, c)
        return best

    def step(self):
        p.stepSimulation()

    def disconnect(self):
        p.disconnect()
