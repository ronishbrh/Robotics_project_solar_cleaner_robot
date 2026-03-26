"""
pybullet_controller_hobot.py
Manual keyboard controller for the solar panel cleaning robot.
HOBOT-style pivot rotation (differential drive), no skid-turning forward/backward.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
WHEEL VELOCITY — WHY THE SIGNS ARE WHAT THEY ARE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
All wheel joints: rpy="1.5708 0 0", axis xyz="0 0 1"
Rx(90°) rotates local Z → world −Y.
PyBullet positive velocity = CCW about −Y = CW about +Y.
CW about +Y → wheel bottom moves in +X → robot moves FORWARD.

  FORWARD  : set_wheels(+v, +v)   both sides positive
  BACKWARD : set_wheels(−v, −v)   both sides negative
  ROT LEFT : set_wheels(−v, +v)   pivot left in place
  ROT RIGHT: set_wheels(+v, −v)   pivot right in place

No negation is applied inside set_wheels().

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
KEY BINDINGS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Movement (hold key):
  ↑  /  W      Forward
  ↓  /  S      Backward
  ←  /  A      Pivot left
  →  /  D      Pivot right

Brush:
  B            Toggle brush spin ON / OFF

Cleaning:
  C            Confirm current panel cleaned.

Bridge:
  1            Front +10 cm  (max 32 cm)
  2            Back -10 cm  (max 32 cm)
  0            Retract bridge to middle

Body lift:
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
import math

WHEEL_SPEED = 16.0  # rad/s
WHEEL_FORCE = 800.0  # N
BRUSH_SPEED = 12.0  # rad/s
BRUSH_FORCE = 5.0
BRIDGE_FORCE = 200.0
BRIDGE_VEL = 0.20  # m/s
SUCTION_FORCE = 120.0
SUCTION_VEL = 0.08
LIFT_FORCE = 600.0
LIFT_VEL = 0.05
LIFT_STEP = 0.1  # m per key press


class ManualController:

    def __init__(self, robot_id, env):
        self.robot_id = robot_id
        self.env = env
        self.joints = env.joint_indices

        self.brush_on = False
        self.bridge = 0.0
        self.lift_pos = 0.0
        self.suction_at_base_on = True
        self.suction_at_pads_on = False

        print(__doc__)
        print(f"  Panel tilt : {env.panel_tilt_deg:.1f}°")
        print()

    # ── Wheel control ─────────────────────────────────────────
    def _set_wheels(self, left: float, right: float):
        # tilt_rad = math.radians(self.env.panel_tilt_deg)

        # left_vel_adj = left * math.cos(tilt_rad)
        # right_vel_adj = right * math.cos(tilt_rad)

        tilt_rad = math.radians(self.env.panel_tilt_deg)

        is_pivot = (left * right) < 0

        if is_pivot:

            left_vel_adj = left
            right_vel_adj = right
        else:
            # Only scale forward/backward
            scale = math.cos(tilt_rad)
            left_vel_adj = left * scale
            right_vel_adj = right * scale
        pairs = [
            ("wheel_fl_joint", left_vel_adj),
            ("wheel_rl_joint", left_vel_adj),
            ("wheel_fr_joint", right_vel_adj),
            ("wheel_rr_joint", right_vel_adj),
        ]
        for jname, vel in pairs:
            idx = self.joints.get(jname, -1)
            if idx >= 0:
                p.setJointMotorControl2(
                    self.robot_id,
                    idx,
                    p.VELOCITY_CONTROL,
                    targetVelocity=float(vel),
                    force=2500,
                )

    def _stop_wheels(self):
        self._set_wheels(0.0, 0.0)

    # ── Brush ───────────────────────────────────────────────
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

    # ── Bridge ──────────────────────────────────────────────
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

    # ── Suction ─────────────────────────────────────────────
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
        if self.robot_id is None:
            return
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        nx, ny, nz = self.env.panel_normal
        fvec = [-nx * force_n, -ny * force_n, -nz * force_n]
        p.applyExternalForce(self.robot_id, -1, fvec, [0, 0, 0], p.LINK_FRAME)

    def apply_suction_on_legs(self, force_n=1300.0):
        if self.robot_id is None:
            return
        d = [0, 0, -1]
        for cup_link in self.env.cup_links:
            nx, ny, nz = self.env.panel_normal
            fvec = [-nx * force_n, -ny * force_n, -nz * force_n]
            p.applyExternalForce(self.robot_id, cup_link, fvec, [0, 0, 0], p.LINK_FRAME)

    # ── Lift ────────────────────────────────────────────────
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

    # ── Status ──────────────────────────────────────────────
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
        print("  ───────────────────────────────────────\n")

    # ── Key handling ─────────────────────────────────────────
    def _handle_key(self, key: int, state):
        v = WHEEL_SPEED

        # ─ Pivot / forward / backward
        if key in (p.B3G_UP_ARROW, ord("w"), ord("W")) and state & p.KEY_IS_DOWN:
            self._set_wheels(-v, -v)
        elif key in (p.B3G_DOWN_ARROW, ord("s"), ord("S")) and state & p.KEY_IS_DOWN:
            self._set_wheels(v, v)
        elif key in (p.B3G_LEFT_ARROW, ord("a"), ord("A")) and state & p.KEY_IS_DOWN:
            self._set_wheels(v, -v)  # Pivot left in place
        elif key in (p.B3G_RIGHT_ARROW, ord("d"), ord("D")) and state & p.KEY_IS_DOWN:
            self._set_wheels(-v, v)  # Pivot right in place

        # ─ Brush
        elif key in (ord("b"), ord("B")) and state & p.KEY_WAS_TRIGGERED:
            self.brush_on = not self.brush_on
            self._set_brush(self.brush_on)
            print("Brush ON" if self.brush_on else "Brush OFF")

        # ─ Cleaning
        elif key in (ord("c"), ord("C")) and state & p.KEY_WAS_TRIGGERED:
            row, col = self.env.nearest_panel()
            if (row, col) not in self.env.cleaned_set:
                if not self.brush_on:
                    self._set_brush(True)
                    for _ in range(int(0.5 * 240)):
                        self.env.step()
                        time.sleep(1.0 / 240.0)
                    self._set_brush(False)
                self.env.clean_panel(row, col)

        # ─ Bridge
        elif key == ord("1") and state & p.KEY_WAS_TRIGGERED:
            self.bridge = min(self.bridge + 0.10, 0.32)
            self._set_bridge(self.bridge)
            print(f"Front bridge + {self.bridge*100:.0f} cm")
        elif key == ord("2") and state & p.KEY_WAS_TRIGGERED:
            self.bridge = max(self.bridge - 0.10, -0.32)
            self._set_bridge(self.bridge)
            print(f"Front bridge - {self.bridge*100:.0f} cm")
        elif key == ord("0") and state & p.KEY_WAS_TRIGGERED:
            self.bridge = 0
            self._set_bridge(self.bridge)
            print("Front bridge reset")

        # ─ Suction
        elif key == ord("3") and state & p.KEY_WAS_TRIGGERED:
            self.suction_at_base_on = not self.suction_at_base_on
            print("Suction at base =", self.suction_at_base_on)
        elif key == ord("4") and state & p.KEY_WAS_TRIGGERED:
            self.suction_at_pads_on = not self.suction_at_pads_on
            print("Suction at pads =", self.suction_at_pads_on)

        # ─ Lift
        elif key in (ord("l"), ord("L")) and state & p.KEY_WAS_TRIGGERED:
            self.suction_at_base_on = False
            self.lift_pos = max(self.lift_pos - LIFT_STEP, -0.1)
            self._set_lift(self.lift_pos)
            print(f"Body lift ▲ {self.lift_pos*1000:.0f} mm")
        elif key in (ord("k"), ord("K")) and state & p.KEY_WAS_TRIGGERED:
            self.lift_pos = min(self.lift_pos + LIFT_STEP, 0)
            self._set_lift(self.lift_pos)
            print(f"Body lift ▼ {self.lift_pos*1000:.0f} mm")
            # if self.lift_pos == 0:
            #     self.suction_at_base_on = True

        # ─ Status / Quit
        elif key in (ord("p"), ord("P")) and state & p.KEY_WAS_TRIGGERED:
            self._print_status()
        elif key in (ord("q"), ord("Q"), 27):  # Esc
            print("Quitting …")
            return False

        return True

    # ── Main loop ─────────────────────────────────────────
    def run(self):
        print("Manual control active.\n")
        try:
            while True:
                keys = p.getKeyboardEvents()

                any_move = any(
                    (
                        k
                        in (
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
                        )
                        and state & p.KEY_IS_DOWN
                    )
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
                    self.apply_suction_on_base(force_n=600.0)
                if self.suction_at_pads_on:
                    tilt = math.radians(self.env.panel_tilt_deg)
                    self.apply_suction_on_legs(800 + 2000 * math.sin(tilt))

                is_gap, dist = self.env.detect_gap(range_m=0.2)
                if is_gap:
                    print(
                        f"⚠️[LIDAR REPORT] GAP DETECTED ahead! No panel within {dist:.2f}m."
                    )
                else:
                    print(f"[LIDAR STATUS] Surface OK. Distance to panel: {dist:.3f}m")

                time.sleep(1.0 / 240.0)

        except KeyboardInterrupt:
            print("\nStopped.")
            self._stop_wheels()
