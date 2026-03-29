"""
autonomous_executor.py
Sensor-based autonomous controller

Wheel convention (confirmed):
  FORWARD  : set_wheels(-v, -v)
  BACKWARD : set_wheels(+v, +v)
  ROT LEFT : set_wheels(+v, -v)
  ROT RIGHT: set_wheels(-v, +v)

Gap crossing sequence (from key observation):
  1  bridge forward  (+0.10)
  4  suction pads ON
  L  lift UP
  3  suction base ON
  drive forward until landed on next panel
  2  bridge back
  3  suction base OFF
  4  suction pads OFF
  K  lift DOWN
  0  bridge center
"""

import math
import time
from pybullet_controller import ManualController

TURN_STEPS = 910
SIM_DT = 1.0 / 480.0
SLOW = 2.0

FWD = 8.0  # forward
BACK = -8.0  # backward
ROT = 8.0  # rotation magnitude
LIFT_STEP = 0.1  # must match ManualController.LIFT_STEP


# ─────────────────────────────────────────────
# ROBOT WRAPPER
# ─────────────────────────────────────────────
class AutoRobot:

    def __init__(self, ctrl):
        self.ctrl = ctrl
        self.env = ctrl.env
        self.alternating_in_panel = "right"
        self.alternating_across_panels = "left"
        self.PASSES = int(self.env.PANEL_LENGTH // 0.35)
        print(f"Total Passess: {self.PASSES}")

    def tick(self):
        self.env.step()
        if self.ctrl.suction_at_base_on:
            self.ctrl.apply_suction_on_base(6500)
        if self.ctrl.suction_at_pads_on:
            self.ctrl.apply_suction_on_legs(4000)
        # time.sleep(SIM_DT/2)

    def stop(self):
        self.ctrl._stop_wheels()

    def set_wheels(self, l, r):
        self.ctrl._set_wheels(l, r)

    def wait(self, steps):
        for _ in range(steps):
            self.tick()

    # ── ACTUATORS (named after key presses) ───

    def bridge_forward(self):  # key 1
        print("  [1] BRIDGE FORWARD")
        self.ctrl.bridge = min(self.ctrl.bridge + 0.10, 0.3)
        self.ctrl._set_bridge(self.ctrl.bridge)
        self.wait(40)

    def bridge_backward(self):  # key 2
        print("  [2] BRIDGE BACKWARD")
        self.ctrl.bridge = max(self.ctrl.bridge - 0.10, -0.3)
        self.ctrl._set_bridge(self.ctrl.bridge)
        self.wait(40)

    def bridge_center(self):  # key 0
        print("  [0] BRIDGE CENTER")
        self.ctrl.bridge = 0
        self.ctrl._set_bridge(self.ctrl.bridge)
        self.wait(40)

    def suction_base_toggle(self):  # key 3 — toggles
        self.ctrl.suction_at_base_on = not self.ctrl.suction_at_base_on
        print(f"  [3] Suction at base = {self.ctrl.suction_at_base_on}")

    def suction_pads_toggle(self):  # key 4 — toggles
        self.ctrl.suction_at_pads_on = not self.ctrl.suction_at_pads_on
        print(f"  [4] Suction at pads = {self.ctrl.suction_at_pads_on}")

    def lift_up(self):  # key L
        print("  [L] LIFT UP")
        self.ctrl.lift_pos = max(self.ctrl.lift_pos - LIFT_STEP, -0.1)
        self.ctrl._set_lift(-0.1)
        self.wait(120)

    def lift_down(self):  # key K
        print("  [K] LIFT DOWN")
        self.ctrl.lift_pos = min(self.ctrl.lift_pos + LIFT_STEP, 0)
        self.ctrl._set_lift(self.ctrl.lift_pos)
        self.wait(120)


# ─────────────────────────────────────────────
# PRIMITIVES
# ─────────────────────────────────────────────
def safe_forward(robot, max_steps=2000):
    """
    Drive forward until the lidar goes red (no panel directly below = gap edge).
    Stops the robot right at the gap edge and returns 'gap'.
    """
    print("  >> safe_forward")
    for _ in range(max_steps):
        is_gap, _ = robot.env.detect_gap(range_m=0.3)
        if is_gap:
            robot.stop()
            print("  ⚠ LIDAR RED — at gap edge, stopped")
            return "gap"
        robot.set_wheels(FWD, FWD)
        robot.tick()
    robot.stop()
    return "done"


def safe_backward(robot, steps=100):
    print("  >> safe_backward")
    for _ in range(steps):
        robot.set_wheels(BACK, BACK)
        robot.tick()
    robot.stop()


# def rotate_left(robot, steps=120):
#     print("  >> rotate_left")
#     for _ in range(steps):
#         robot.set_wheels(-ROT, ROT)
#         robot.tick()
#     robot.stop()


# def rotate_right(robot, steps=120):
#     print("  >> rotate_right")
#     for _ in range(steps):
#         robot.set_wheels(ROT, -ROT)
#         robot.tick()
#     robot.stop()

def rotate_left_exact(robot, degrees=90, speed=ROT):
    prev_yaw = robot.env.get_robot_yaw()
    total_rotation = 0.0
    target = math.radians(degrees)

    print(f"  >> rotate_left_exact {degrees}°")

    while total_rotation < target:
        robot.set_wheels(-speed, speed)
        robot.tick()

        current_yaw = robot.env.get_robot_yaw()

        delta = ((current_yaw - prev_yaw + math.pi) % (2 * math.pi)) - math.pi

        total_rotation += abs(delta)
        prev_yaw = current_yaw

    robot.stop()


def rotate_right_exact(robot, degrees=90, speed=ROT):
    prev_yaw = robot.env.get_robot_yaw()
    total_rotation = 0.0
    target = math.radians(degrees)

    print(f"  >> rotate_right_exact {degrees}°")

    while total_rotation < target:
        robot.set_wheels(speed, -speed)
        robot.tick()

        current_yaw = robot.env.get_robot_yaw()

        delta = ((prev_yaw - current_yaw + math.pi) % (2 * math.pi)) - math.pi

        total_rotation += abs(delta)
        prev_yaw = current_yaw

    robot.stop()

# ─────────────────────────────────────────────
# GAP CROSSING
# Robot has stopped at gap edge (lidar red).
# 1. Back up slightly so rear wheels are safe
# 2. Sequence: 1 → 4 → 3 → L → drive forward until landed → 2 → 3 → K → 0
# ─────────────────────────────────────────────
def cross_gap(robot):
    print("\n=== CROSSING GAP ===")

    # Back up slightly so robot isn't teetering on edge
    print("  >> backing up from edge...")
    # safe_backward(robot, steps=40)

    robot.bridge_forward()  # 1 — bridge extends forward
    robot.bridge_forward()
    robot.bridge_forward()
    robot.suction_pads_toggle()  # 4 — pads ON
    robot.lift_up()
    if robot.ctrl.suction_at_base_on:
        robot.suction_base_toggle()
    robot.wait(220)

    # Drive forward until lidar sees next panel (not ground plane)
    print("  >> driving across gap...")
    # for _ in range(1000):
    #    robot.set_wheels(FWD, FWD)
    #    robot.tick()
    #    is_gap, _ = robot.env.detect_gap(range_m=0.35)
    #    if not is_gap:
    #        print("  ✅ LIDAR GREEN — landed on next panel")
    #        for _ in range(80):
    #            robot.set_wheels(FWD, FWD)
    #            robot.tick()
    #        break
    # robot.stop()

    robot.bridge_backward()
    robot.bridge_backward()
    robot.bridge_backward()
    robot.bridge_backward()
    robot.bridge_backward()
    robot.bridge_backward()

    # 3 — base suction OFF
    if not robot.ctrl.suction_at_base_on:
        robot.suction_base_toggle()

    robot.lift_down()
    print("lift down sakyo")
    if robot.ctrl.suction_at_pads_on:
        print("Suction pads on cha")
        robot.suction_pads_toggle()
    robot.bridge_center()  # 0 — centre bridge
    print("bridge center bhayo")

    # Re-enable base suction for normal driving on new panel
    if not robot.ctrl.suction_at_base_on:
        robot.suction_base_toggle()
    print("  [3] Suction at base = ON (restored)")

    print("=== GAP CROSSED ===\n")


# ─────────────────────────────────────────────
# CLEAN ONE PANEL — simple sweep
# ─────────────────────────────────────────────


def clean_panel(robot, last_panel_in_row=False):
    print("  >> cleaning panel")

    for pass_n in range(robot.PASSES):
        print(f"  pass {pass_n + 1}/{robot.PASSES}")
        result = safe_forward(robot)

        if result == "gap":
            print("Gap detected: Edge reached")

            # Already stopped at gap edge — cross it
            # cross_gap(robot)
            # print("cross gap finish")

        safe_backward(robot, steps=60)

        if pass_n + 1 == robot.PASSES and last_panel_in_row:
            break

        if robot.alternating_in_panel == "left":
            # rotate_left(robot, steps=TURN_STEPS)
            rotate_left_exact(robot)
            if pass_n + 1 == robot.PASSES:
                break
            result = safe_forward(robot, max_steps=358 + int((90) * math.sin(math.radians(robot.env.panel_tilt_deg))) * (1 if robot.alternating_across_panels == "left" else -1) + (90 if pass_n + 2 == robot.PASSES else 0))
            if result == "gap":
                safe_backward(robot, steps=55 + int((22 + (30 if pass_n + 2 == robot.PASSES else 0)) * math.sin(math.radians(robot.env.panel_tilt_deg))) * (-1 if robot.alternating_across_panels == "left" else 1))
            # rotate_left(robot, steps=TURN_STEPS)
            rotate_left_exact(robot)
            robot.alternating_in_panel = "right"
        elif robot.alternating_in_panel == "right":
            # rotate_right(robot, steps=TURN_STEPS)
            rotate_right_exact(robot)
            if pass_n + 1 == robot.PASSES:
                break
            result = safe_forward(robot, max_steps=358 + int((90) * math.sin(math.radians(robot.env.panel_tilt_deg))) * (1 if robot.alternating_across_panels == "left" else -1) + (90 if pass_n + 2 == robot.PASSES else 0))
            if result == "gap":
                safe_backward(robot, steps=55 + int((22 + (30 if pass_n + 2 == robot.PASSES else 0)) * math.sin(math.radians(robot.env.panel_tilt_deg))) * (-1 if robot.alternating_across_panels == "left" else 1))
            # rotate_right(robot, steps=TURN_STEPS)
            rotate_right_exact(robot)
            robot.alternating_in_panel = "left"
        else:
            print("Programmer error")

    print("  >> panel done")


# ─────────────────────────────────────────────
# SNAKE PATH GENERATOR
# col 0: rows 0→2, col 1: rows 2→0, col 2: rows 0→2 ...
# ─────────────────────────────────────────────
def generate_path(rows=3, cols=4):
    path = []
    for c in range(cols):

        col_rows = range(rows) if c % 2 == 0 else reversed(range(rows))

        for r in col_rows:
            path.append((r, c))
    return path


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
def run_autonomous(env):
    ctrl = ManualController(env.robot_id, env)
    robot = AutoRobot(ctrl)

    print("\n===== AUTONOMOUS START =====\n")

    # suction_at_base_on is True by default in ManualController
    # just settle with suction active
    print("  >> settling (suction active)...")
    robot.wait(240)

    rows = 1
    cols = 3
    last_row = rows - 1
    path = generate_path(rows, cols)
    print(f"Path: {path}")
    if not robot.ctrl.suction_at_base_on:
        robot.suction_base_toggle()

    for r, c in path:
        print(f"\n>>> PANEL ({r},{c})")
        print(f"\n>>> Last Row {last_row}")

        clean_panel(robot, last_panel_in_row=r == last_row)
        env.clean_panel(r, c)

        if r == last_row and c+1 == cols:
            break

        if r == last_row:
            if robot.alternating_in_panel == robot.alternating_across_panels:
                if robot.alternating_across_panels == "left":
                    rotate_left_exact(robot, degrees=90)
                    safe_forward(robot, int(70* math.sin(math.radians(robot.env.panel_tilt_deg))) * (1 if robot.alternating_across_panels == "left" else -1))
                    rotate_left_exact(robot, degrees=90)
                    robot.alternating_across_panels = "right"
                elif robot.alternating_across_panels == "right":
                    rotate_left_exact(robot, degrees=90)
                    safe_forward(robot, int(70* math.sin(math.radians(robot.env.panel_tilt_deg))) * (1 if robot.alternating_across_panels == "right" else -1))
                    rotate_left_exact(robot, degrees=90)
                    robot.alternating_across_panels = "left"
            else:
                if robot.alternating_in_panel == "left":
                    robot.alternating_in_panel = "right"
                    robot.alternating_across_panels = "left"
                elif robot.alternating_in_panel == "right":
                    robot.alternating_in_panel = "left"
                    robot.alternating_across_panels = "right"


        safe_forward(robot)
        cross_gap(robot)

        if r != last_row:
            if robot.alternating_in_panel == "left":
                rotate_left_exact(robot)
                # rotate_left(robot, steps=TURN_STEPS)
                robot.alternating_in_panel = "right"
            elif robot.alternating_in_panel == "right":
                rotate_right_exact(robot)
                # rotate_right(robot, steps=TURN_STEPS)
                robot.alternating_in_panel = "left"

        if r == last_row:
            last_row = 0 if last_row == rows - 1 else rows - 1

    print("\n===== ALL PANELS CLEANED =====\n")

    while True:
        robot.tick()


# ─────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────
if __name__ == "__main__":
    from pybullet_environment import SolarPanelEnvironment

    env = SolarPanelEnvironment(
        gui=True, urdf_path="solar_robot_pybullet.urdf", panel_tilt_deg=50
    )
    env.add_dirt_patches()
    run_autonomous(env)
