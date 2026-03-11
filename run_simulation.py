"""
run_simulation.py
Launch the solar panel cleaning robot simulation in manual mode.

Usage:
    python run_simulation.py
    python run_simulation.py 25        # 25 degree panel tilt
"""

import sys
import os
import time

from pybullet_environment import SolarPanelEnvironment
from pybullet_controller import ManualController


def main():
    # tilt angle from command line
    tilt = 0.0
    if len(sys.argv) > 1:
        try:
            tilt = float(sys.argv[1])
            tilt = max(0.0, min(tilt, 45.0))
        except ValueError:
            print("Usage: python run_simulation.py [tilt_degrees]")
            sys.exit(1)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "solar_robot_pybullet.urdf")

    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF not found at {urdf_path}")
        sys.exit(1)

    print("=" * 50)
    print("  Solar Panel Cleaning Robot")
    print(f"  Panel tilt : {tilt:.1f}°")
    print(f"  URDF       : {urdf_path}")
    print("=" * 50)

    env = SolarPanelEnvironment(gui=True, urdf_path=urdf_path, panel_tilt_deg=tilt)

    env.add_dirt_patches()

    if env.robot_id is None:
        print("ERROR: Robot failed to load. Check URDF path.")
        env.disconnect()
        sys.exit(1)

    ctrl = ManualController(env.robot_id, env)
    try:
        ctrl.run()
    except KeyboardInterrupt:
        pass

    env.disconnect()
    print("Simulation ended.")


if __name__ == "__main__":
    main()
