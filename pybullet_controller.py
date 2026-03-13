"""
pybullet_controller.py
Manual keyboard controller for the solar panel cleaning robot.
No autonomous code.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
WHEEL VELOCITY — WHY THE SIGNS ARE WHAT THEY ARE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
All wheel joints: rpy="1.5708 0 0", axis xyz="0 0 1"
Rx(90°) rotates local Z → world −Y.
PyBullet positive velocity = CCW about −Y = CW about +Y.
CW about +Y → wheel bottom moves in +X → robot moves FORWARD.

  FORWARD  : set_wheels(+v, +v)   both sides positive
  BACKWARD : set_wheels(−v, −v)   both sides negative
  ROT LEFT : set_wheels(−v, +v)   left back, right fwd
  ROT RIGHT: set_wheels(+v, −v)   left fwd, right back

No negation is applied inside set_wheels().

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
KEY BINDINGS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Movement (hold key):
  ↑  /  W      Forward
  ↓  /  S      Backward
  ←  /  A      Rotate left
  →  /  D      Rotate right

Brush:
  B            Toggle brush spin ON / OFF

Cleaning:
  C            Confirm current panel cleaned.
               Tip: spin brush (B), sweep the panel (↑↓),
               then press C to remove dirt visuals.

Bridge:
  1            Front +10 cm  (max 32 cm)
  2            Back -10 cm  (max 32 cm)
  0            Retract bridge to middle

Body lift (for gap crossing):
  L            Lift body +5 mm  (max 100 mm)
  K            Lower body −5 mm

Info:
  P            Print robot position + panel status
  Q / Esc      Quit
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import pybullet as p
import numpy as np
import time


WHEEL_SPEED = 16.0  # rad/s
WHEEL_FORCE = 800.0  # N
BRUSH_SPEED = 12.0  # rad/s
BRUSH_FORCE = 5.0
BRIDGE_FORCE = 200.0
BRIDGE_VEL = 0.20  # m/s
SUCTION_FORCE = 120.0
SUCTION_VEL = 0.08
LIFT_FORCE = 300.0
LIFT_VEL = 0.05
LIFT_STEP = 0.1  # m per key press


class ManualController:

    def __init__(self, robot_id, env):
        self.robot_id = robot_id
        self.env = env
        self.joints = env.joint_indices

        self.brush_on = False
        self.bridge = 0.0
        self.front_legs_down = False
        self.rear_legs_down = False
        self.lift_pos = 0.0  # metres
        self.suction_at_base_on = True
        self.suction_at_pads_on = False

        print(__doc__)
        print(f"  Panel tilt : {env.panel_tilt_deg:.1f}°")
        print()

    def _set_wheels(self, left: float, right: float):
        """
        Apply velocity to all four wheels.
        left  → FL + RL joints
        right → FR + RR joints
        Positive = robot moves FORWARD (+X). No sign inversion.
        """
        pairs = [
            ("wheel_fl_joint", left),
            ("wheel_rl_joint", left),
            ("wheel_fr_joint", right),
            ("wheel_rr_joint", right),
        ]
        for jname, vel in pairs:
            idx = self.joints.get(jname, -1)
            if idx >= 0:
                p.setJointMotorControl2(
                    self.robot_id,
                    idx,
                    p.VELOCITY_CONTROL,
                    targetVelocity=float(vel),
                    force=WHEEL_FORCE,
                )

    def _stop_wheels(self):
        self._set_wheels(0.0, 0.0)

    def _set_brush(self, on: bool):
        idx = self.joints.get("brush_joint", -1)
        if idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.VELOCITY_CONTROL,
                targetVelocity=BRUSH_SPEED if on else 0.0,
                force=BRUSH_FORCE,
            )

    def _set_bridge(self, pos: float):
        idx = self.joints.get("bridge_mount", -1)
        if idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=float(np.clip(pos, -0.95, 0.95)),
                force=BRIDGE_FORCE,
                maxVelocity=BRIDGE_VEL,
            )

    def _set_suction_pair(self, side: str, pos: float):
        for which in ("left", "right"):
            jname = f"{side}_suction_{which}_joint"
            idx = self.joints.get(jname, -1)
            if idx >= 0:
                p.setJointMotorControl2(
                    self.robot_id,
                    idx,
                    p.POSITION_CONTROL,
                    targetPosition=float(np.clip(pos, 0.0, 0.050)),
                    force=SUCTION_FORCE,
                    maxVelocity=SUCTION_VEL,
                )

    def apply_suction_on_base(self, force_n=120.0):
        """
        Push robot into panel surface by applying an external force
        along -panel_normal.  Call once per simulation step.
        Only active when robot is close to the panel surface.
        Force is reduced to 0 automatically if robot lifts off.
        """
        if self.robot_id is None:
            return

        pos, _ = p.getBasePositionAndOrientation(self.robot_id)

        # Ray from wheel level toward panel surface
        d = [
            -self.env.panel_normal[0],
            -self.env.panel_normal[1],
            -self.env.panel_normal[2],
        ]  # into panel
        # start 0.08m below robot base (near wheel bottom)
        rf = [pos[0] + d[0] * 0.08, pos[1] + d[1] * 0.08, pos[2] + d[2] * 0.08]
        # end 0.06m further down
        rt = [rf[0] + d[0] * 0.085, rf[1] + d[1] * 0.085, rf[2] + d[2] * 0.085]

        res = p.rayTest(rf, rt)
        if not res:
            return
        hit_body, _, hit_frac, _, _ = res[0]
        # Ignore self-hits and misses
        if hit_body == self.robot_id or hit_body == -1:
            return

        # Scale: full force when touching, zero at 0.02m gap
        gap = hit_frac * 0.085
        max_gap = 0.085
        if gap > max_gap:
            return
        scale = 1.0 - gap / max_gap
        # fvec = [d[0] * force_n * scale, d[1] * force_n * scale, d[2] * force_n * scale]
        fvec = [0, 0, -force_n * scale]
        p.applyExternalForce(self.robot_id, -1, fvec, [0,0,0], p.LINK_FRAME)
        print("Force at base")

    def apply_suction_on_legs(self, force_n=30.0):
        if self.robot_id is None:
            return

        # Ray from wheel level toward panel surface
        d = [
            -self.env.panel_normal[0],
            -self.env.panel_normal[1],
            -self.env.panel_normal[2],
        ]  # into panel

        for cup_link in self.env.cup_links:
            state = p.getLinkState(self.robot_id, cup_link)
            cup_pos = state[0]

            very_small = 0.005
            # start 0.08m below robot base (near wheel bottom)
            rf = [cup_pos[0] + d[0] * very_small, cup_pos[1] + d[1] * very_small, cup_pos[2] + d[2] * very_small]
            # end 0.06m further down
            rt = [rf[0] + d[0] * 0.03, rf[1] + d[1] * 0.03, rf[2] + d[2] * 0.03]

            res = p.rayTest(rf, rt)

            if not res:
                return
            print("Hit something", res[0][0], [item['id'] for item in self.env.panel_ids])

            hit_body, _, hit_frac, _, _ = res[0]

            if hit_body in [item['id'] for item in self.env.panel_ids]:
                gap = hit_frac * 0.03
                max_gap = 0.020
                if gap > max_gap:
                    return
                scale = 1.0 - gap / max_gap
                # fvec = [d[0] * force_n * scale, d[1] * force_n * scale, d[2] * force_n * scale]
                # p.applyExternalForce(self.robot_id, cup_link, fvec, cup_pos, p.WORLD_FRAME)
                fvec = [0, 0, -force_n * scale]
                p.applyExternalForce(self.robot_id, cup_link, fvec, [0,0,0], p.LINK_FRAME)
                print("Applying suction at pad: ", cup_link, "at panel: ", hit_body)
            



    def _set_lift(self, pos: float):
        idx = self.joints.get("lift_column_joint", -1)
        if idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=float(np.clip(pos, -0.1, 0)),
                force=LIFT_FORCE,
                maxVelocity=LIFT_VEL,
            )

    def _print_status(self):
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        row, col = self.env.nearest_panel()
        cleaned = len(self.env.cleaned_set)
        total = len(self.env.panel_ids)
        print("\n  ── Status ─────────────────────────────")
        print(f"  Robot pos   : ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        print(
            f"  Nearest panel: ({row},{col})  "
            f"{'✓ cleaned' if (row,col) in self.env.cleaned_set else 'dirty'}"
        )
        print(f"  Cleaned     : {cleaned}/{total}")
        print(f"  Brush       : {'ON' if self.brush_on else 'off'}")
        print(f"  Body lift   : {self.lift_pos*1000:.0f} mm")
        print(
            f"  Front bridge: {self.front_bridge[0]*100:.0f} / "
            f"{self.front_bridge[1]*100:.0f} cm"
        )
        print(
            f"  Rear  bridge: {self.rear_bridge[0]*100:.0f} / "
            f"{self.rear_bridge[1]*100:.0f} cm"
        )
        print(f"  Front legs  : {'DOWN' if self.front_legs_down else 'up'}")
        print(f"  Rear  legs  : {'DOWN' if self.rear_legs_down else 'up'}")
        print("  ───────────────────────────────────────\n")

    # ── key handler ─────────────────────────────────────────
    def _handle_key(self, key: int, state) -> bool:
        v = WHEEL_SPEED

        if key in (p.B3G_UP_ARROW, ord("w"), ord("W")):
            self._set_wheels(v, v)

        elif key in (p.B3G_DOWN_ARROW, ord("s"), ord("S")):
            self._set_wheels(-v, -v)

        elif key in (p.B3G_LEFT_ARROW, ord("a"), ord("A")):
            self._set_wheels(-v, v)

        elif key in (p.B3G_RIGHT_ARROW, ord("d"), ord("D")):
            self._set_wheels(v, -v)

        elif key in (ord("b"), ord("B")) and p.KEY_WAS_TRIGGERED:
            self.brush_on = not self.brush_on
            self._set_brush(self.brush_on)
            if self.brush_on:
                print("  Brush ON  - drive over panel to sweep, then press C")
            else:
                print("  Brush OFF")

        elif key in (ord("c"), ord("C")) and p.KEY_WAS_TRIGGERED:
            row, col = self.env.nearest_panel()
            if (row, col) in self.env.cleaned_set:
                print(f"  Panel ({row},{col}) already clean")
            else:
                if not self.brush_on:
                    self._set_brush(True)
                    for _ in range(int(0.5 * 240)):
                        self.env.step()
                        time.sleep(1.0 / 240.0)
                    self._set_brush(False)
                self.env.clean_panel(row, col)

        elif key == ord("1") and p.KEY_WAS_TRIGGERED:
            self.bridge = min(self.bridge + 0.10, 0.32)
            self._set_bridge(self.bridge)
            print(f"  Front bridge seg1 - {self.bridge*100:.0f} cm")

        elif key == ord("2") and p.KEY_WAS_TRIGGERED:
            self.bridge = max(self.bridge - 0.10, -0.32)
            self._set_bridge(self.bridge)
            print(f"  Front bridge seg1 - {self.bridge*100:.0f} cm")

        elif key == ord("0") and state & p.KEY_WAS_TRIGGERED:
            self.bridge = 0
            self._set_bridge(self.bridge)
            print(f"  Front bridge seg1 - {self.bridge*100:.0f} cm")

        elif key == ord("3") and state & p.KEY_WAS_TRIGGERED:
            self.suction_at_base_on = not self.suction_at_base_on
            print("Suction at base = ", self.suction_at_base_on)

        elif key == ord("4") and state & p.KEY_WAS_TRIGGERED:
            self.suction_at_pads_on = not self.suction_at_pads_on
            print("Suction at pads = ", self.suction_at_pads_on)

        elif key in (ord("l"), ord("L")) and state & p.KEY_WAS_TRIGGERED:
            self.lift_pos = max(self.lift_pos - LIFT_STEP, -.1)
            self._set_lift(-.1)
            print(f"  Body lift ▲ {self.lift_pos*1000:.0f} mm")

        elif key in (ord("k"), ord("K")) and state & p.KEY_WAS_TRIGGERED:
            self.lift_pos = min(self.lift_pos + LIFT_STEP, 0)
            self._set_lift(self.lift_pos)
            print(f"  Body lift ▼ {self.lift_pos*1000:.0f} mm")

        elif key in (ord("p"), ord("P")) and state & p.KEY_WAS_TRIGGERED:
            self._print_status()

        elif key in (ord("q"), ord("Q"), 27):  # 27 = Esc
            print("  Quitting …")
            return False

        return True

    def run(self):
        print("Manual control active.\n")

        MOVE_KEYS = {
            p.B3G_UP_ARROW,
            p.B3G_DOWN_ARROW,
            p.B3G_LEFT_ARROW,
            p.B3G_RIGHT_ARROW,
            ord("w"),
            ord("W"),
            ord("s"),
            ord("S"),
            ord("a"),
            ord("A"),
            ord("d"),
            ord("D"),
        }

        try:
            while True:
                keys = p.getKeyboardEvents()

                any_move = any(
                    (k in MOVE_KEYS) and (state & p.KEY_IS_DOWN)
                    for k, state in keys.items()
                )
                if not any_move:
                    self._stop_wheels()

                for key, state in keys.items():
                    if state & p.KEY_IS_DOWN:
                        if not self._handle_key(key, state):
                            self._stop_wheels()
                            return

                self.env.step()

                if self.suction_at_base_on:
                    self.apply_suction_on_base(force_n=320.0)

                if self.suction_at_pads_on:
                    self.apply_suction_on_legs(force_n=100)

                is_gap, dist = self.env.detect_gap(range_m=0.2)

                if is_gap:
                    print(
                        f"⚠️[LIDAR REPORT] GAP DETECTED ahead! No panel within {dist:.2f}m."
                    )
                else:
                    # This prints the current clearance distance to the panel
                    print(f"[LIDAR STATUS] Surface OK. Distance to panel: {dist:.3f}m")

                time.sleep(1.0 / 240.0)

        except KeyboardInterrupt:
            print("\n  Stopped.")
            self._stop_wheels()
