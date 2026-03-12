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

Front bridge (press repeatedly to extend 10 cm at a time):
  1            Front seg1  +10 cm  (max 40 cm)
  2            Front seg2  +10 cm  (max 30 cm)
  0            Retract front bridge fully

Rear bridge:
  3            Rear seg1  +10 cm  (max 40 cm)
  4            Rear seg2  +10 cm  (max 30 cm)
  9            Retract rear bridge fully

Suction legs (bridge tip pads):
  F            Toggle FRONT tip suction pads DOWN / UP
  R            Toggle REAR  tip suction pads DOWN / UP

Body lift (for gap crossing):
  L            Lift body +5 mm  (max 50 mm)
  K            Lower body −5 mm

Info:
  P            Print robot position + panel status
  Q / Esc      Quit
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import pybullet as p
import numpy as np
import time


WHEEL_SPEED = 5.0  # rad/s
WHEEL_FORCE = 300.0  # N
BRUSH_SPEED = 12.0  # rad/s
BRUSH_FORCE = 5.0
BRIDGE_FORCE = 200.0
BRIDGE_VEL = 0.20  # m/s
SUCTION_FORCE = 120.0
SUCTION_VEL = 0.08
LIFT_FORCE = 300.0
LIFT_VEL = 0.05
LIFT_STEP = 0.005  # m per key press


class ManualController:

    def __init__(self, robot_id, env):
        self.robot_id = robot_id
        self.env = env
        self.joints = env.joint_indices

        self.brush_on = False
        self.front_bridge = [0.0, 0.0]
        self.rear_bridge = [0.0, 0.0]
        self.front_legs_down = False
        self.rear_legs_down = False
        self.lift_pos = 0.0  # metres

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

    def _set_bridge(self, side: str, seg: int, pos: float):
        """side = 'front' | 'rear',  seg = 1 | 2"""
        jname = f"{side}_bridge_extend{seg}"
        upper = 0.40 if seg == 1 else 0.30
        idx = self.joints.get(jname, -1)
        if idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=float(np.clip(pos, 0.0, upper)),
                force=BRIDGE_FORCE,
                maxVelocity=BRIDGE_VEL,
            )
        else:
            print(f"  [WARN] joint not found: {jname}")

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

    def _set_lift(self, pos: float):
        idx = self.joints.get("body_lift_joint", -1)
        if idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                idx,
                p.POSITION_CONTROL,
                targetPosition=float(np.clip(pos, 0.0, 0.050)),
                force=LIFT_FORCE,
                maxVelocity=LIFT_VEL,
            )

    def _print_status(self):
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        row, col = self.env.nearest_panel()
        cleaned = len(self.env.cleaned_set)
        total = len(self.env.panel_ids)
        print(f"\n  ── Status ─────────────────────────────")
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
        print(f"  ───────────────────────────────────────\n")

    # ── key handler ─────────────────────────────────────────
    def _handle_key(self, key: int) -> bool:
        v = WHEEL_SPEED

        if key in (p.B3G_UP_ARROW, ord("w"), ord("W")):
            self._set_wheels(v, v)

        elif key in (p.B3G_DOWN_ARROW, ord("s"), ord("S")):
            self._set_wheels(-v, -v)

        elif key in (p.B3G_LEFT_ARROW, ord("a"), ord("A")):
            self._set_wheels(-v, v)

        elif key in (p.B3G_RIGHT_ARROW, ord("d"), ord("D")):
            self._set_wheels(v, -v)

        elif key in (ord("b"), ord("B")):
            self.brush_on = not self.brush_on
            self._set_brush(self.brush_on)
            if self.brush_on:
                print("  Brush ON  - drive over panel to sweep, then press C")
            else:
                print("  Brush OFF")

        elif key in (ord("c"), ord("C")):
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

        elif key == ord("1"):
            self.front_bridge[0] = min(self.front_bridge[0] + 0.10, 0.40)
            self._set_bridge("front", 1, self.front_bridge[0])
            print(f"  Front bridge seg1 - {self.front_bridge[0]*100:.0f} cm")

        elif key == ord("2"):
            self.front_bridge[1] = min(self.front_bridge[1] + 0.10, 0.30)
            self._set_bridge("front", 2, self.front_bridge[1])
            print(f"  Front bridge seg2 - {self.front_bridge[1]*100:.0f} cm")

        elif key == ord("0"):
            self.front_bridge = [0.0, 0.0]
            self._set_bridge("front", 2, 0.0)
            self._set_bridge("front", 1, 0.0)
            print("  Front bridge retracted")

        elif key == ord("3"):
            self.rear_bridge[0] = min(self.rear_bridge[0] + 0.10, 0.40)
            self._set_bridge("rear", 1, self.rear_bridge[0])
            print(f"  Rear bridge seg1 - {self.rear_bridge[0]*100:.0f} cm")

        elif key == ord("4"):
            self.rear_bridge[1] = min(self.rear_bridge[1] + 0.10, 0.30)
            self._set_bridge("rear", 2, self.rear_bridge[1])
            print(f"  Rear bridge seg2 - {self.rear_bridge[1]*100:.0f} cm")

        elif key == ord("9"):
            self.rear_bridge = [0.0, 0.0]
            self._set_bridge("rear", 2, 0.0)
            self._set_bridge("rear", 1, 0.0)
            print("  Rear bridge retracted")

        elif key in (ord("f"), ord("F")):
            self.front_legs_down = not self.front_legs_down
            self._set_suction_pair("front", 0.045 if self.front_legs_down else 0.0)
            print(f"  Front legs {'▼ DOWN' if self.front_legs_down else '▲ UP'}")

        elif key in (ord("r"), ord("R")):
            self.rear_legs_down = not self.rear_legs_down
            self._set_suction_pair("rear", 0.045 if self.rear_legs_down else 0.0)
            print(f"  Rear legs {'▼ DOWN' if self.rear_legs_down else '▲ UP'}")

        elif key in (ord("l"), ord("L")):
            self.lift_pos = min(self.lift_pos + LIFT_STEP, 0.050)
            self._set_lift(self.lift_pos)
            print(f"  Body lift ▲ {self.lift_pos*1000:.0f} mm")

        elif key in (ord("k"), ord("K")):
            self.lift_pos = max(self.lift_pos - LIFT_STEP, 0.0)
            self._set_lift(self.lift_pos)
            print(f"  Body lift ▼ {self.lift_pos*1000:.0f} mm")

        elif key in (ord("p"), ord("P")):
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
                        if not self._handle_key(key):
                            self._stop_wheels()
                            return

                # Step physics + suction for inclined panels
                self.env.step()
                if self.env.panel_tilt_deg > 0:
                    self.env.apply_suction(force_n=120.0)
                time.sleep(1.0 / 240.0)

        except KeyboardInterrupt:
            print("\n  Stopped.")
            self._stop_wheels()
