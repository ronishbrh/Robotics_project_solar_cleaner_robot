import sys
import os
import time

from pybullet_environment import SolarPanelEnvironment
from pybullet_controller import RobotController, ManualController


def _banner(title):
    print("\n" + "=" * 62)
    print(f"  {title}")
    print("=" * 62)


def _get_urdf():
    default = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "solar_robot_pybullet.urdf"
    )
    raw = input(f"\n  URDF path  [{default}]: ").strip()
    path = raw if raw else default
    if not os.path.exists(path):
        print(f"\n❌  URDF not found: {path}")
        sys.exit(1)
    return path


def _get_tilt():
    raw = input(
        "\n  Panel tilt angle in degrees  [0 = flat, 25 = typical install]: "
    ).strip()
    try:
        t = float(raw) if raw else 0.0
        t = max(0.0, min(t, 50.0))
        return t
    except ValueError:
        print("  Invalid input – using 0°")
        return 0.0


def _get_vacuum():
    raw = input(
        "\n  Vacuum suction force in Newtons  [180 = default, 0 = disabled]: "
    ).strip()
    try:
        return max(0.0, float(raw)) if raw else 180.0
    except ValueError:
        return 180.0


def run_autonomous(urdf_path, tilt, vacuum):
    _banner(f"AUTONOMOUS CLEANING  │  tilt={tilt:.1f}°  suction={vacuum:.0f}N")
    env = SolarPanelEnvironment(
        gui=True, urdf_path=urdf_path, panel_tilt_deg=tilt, vacuum_force_N=vacuum
    )
    env.add_dirt_patches()
    if env.robot_id is None:
        print("Robot not loaded.")
        env.disconnect()
        return

    ctrl = RobotController(env.robot_id, env)
    print("\nStarting in 3 s …  (Ctrl-C to abort)")
    time.sleep(3)

    try:
        ctrl.run_autonomous()
    except KeyboardInterrupt:
        print("\nAborted.")

    print("\nMission done. Close window or Ctrl-C.")
    try:
        while True:
            env.step_simulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    env.disconnect()


def run_manual(urdf_path, tilt, vacuum):
    _banner(f"MANUAL CONTROL  │  tilt={tilt:.1f}°  suction={vacuum:.0f}N")
    env = SolarPanelEnvironment(
        gui=True, urdf_path=urdf_path, panel_tilt_deg=tilt, vacuum_force_N=vacuum
    )
    env.add_dirt_patches()
    if env.robot_id is None:
        print("Robot not loaded.")
        env.disconnect()
        return

    ctrl = ManualController(env.robot_id, env)
    try:
        ctrl.run()
    except KeyboardInterrupt:
        print("\nManual control ended.")
    env.disconnect()


def run_env_only(tilt):
    _banner(f"ENVIRONMENT VIEWER  │  tilt={tilt:.1f}°  (no robot)")
    env = SolarPanelEnvironment(gui=True, panel_tilt_deg=tilt)
    env.add_dirt_patches()
    print("Running. Ctrl-C to exit.")
    try:
        while True:
            env.step_simulation()
            time.sleep(1.0 / 240.0)
    except KeyboardInterrupt:
        pass
    env.disconnect()


def main():
    # Allow quick CLI launch: script.py <mode> <tilt>
    cli_mode = sys.argv[1] if len(sys.argv) > 1 else None
    cli_tilt = float(sys.argv[2]) if len(sys.argv) > 2 else None
    cli_vac = float(sys.argv[3]) if len(sys.argv) > 3 else None

    _banner("SOLAR PANEL CLEANING ROBOT  –  PyBullet Simulation")
    print(
        """
  1  Autonomous cleaning  (full snake-path mission)
  2  Manual control       (keyboard-driven + J for auto-cross)
  3  Environment only     (panel array viewer, no robot)
  q  Quit
"""
    )
    choice = cli_mode if cli_mode else input("  Choice: ").strip().lower()

    if choice == "q":
        print("Bye!")
        return
    elif choice in ("1", "2"):
        tilt = cli_tilt if cli_tilt is not None else _get_tilt()
        vacuum = cli_vac if cli_vac is not None else _get_vacuum()
        urdf = _get_urdf()
        if choice == "1":
            run_autonomous(urdf, tilt, vacuum)
        elif choice == "2":
            run_manual(urdf, tilt, vacuum)
    elif choice == "3":
        tilt = cli_tilt if cli_tilt is not None else _get_tilt()
        run_env_only(tilt)
    else:
        print("Invalid choice.")


if __name__ == "__main__":
    main()
