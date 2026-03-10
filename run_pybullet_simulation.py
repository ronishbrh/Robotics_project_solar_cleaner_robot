"""
Main PyBullet Simulation
Solar Panel Cleaning Robot - Complete Demo
"""

import sys
import os
from pybullet_environment import SolarPanelEnvironment
from pybullet_controller import RobotController, ManualController


def run_autonomous_simulation(urdf_path):
    """Run autonomous cleaning simulation"""
    
    print("\n" + "="*60)
    print("AUTONOMOUS SOLAR PANEL CLEANING SIMULATION")
    print("="*60)
    
    # Create environment
    env = SolarPanelEnvironment(gui=True, urdf_path=urdf_path)
    
    # Add dirt patches for visualization
    env.add_dirt_patches()
    
    if env.robot_id is None:
        print("ERROR: Robot not loaded. Check URDF path.")
        return
    
    # Create autonomous controller
    controller = RobotController(env.robot_id, env)
    
    # Run autonomous mission
    print("\nStarting autonomous cleaning in 3 seconds...")
    print("Close window to stop.")
    
    import time
    time.sleep(3)
    
    try:
        controller.run_autonomous(max_steps=50000)
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    
    print("\nSimulation complete. Close window to exit.")
    
    # Keep window open
    try:
        while True:
            env.step_simulation()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        pass
    
    env.disconnect()


def run_manual_simulation(urdf_path):
    """Run manual control simulation"""
    
    print("\n" + "="*60)
    print("MANUAL CONTROL SIMULATION")
    print("="*60)
    
    # Create environment
    env = SolarPanelEnvironment(gui=True, urdf_path=urdf_path)
    
    # Add dirt patches
    env.add_dirt_patches()
    
    if env.robot_id is None:
        print("ERROR: Robot not loaded. Check URDF path.")
        return
    
    # Create manual controller
    controller = ManualController(env.robot_id, env)
    
    # Run manual control
    try:
        controller.run()
    except KeyboardInterrupt:
        print("\nManual control ended")
    
    env.disconnect()


def run_environment_only():
    """Run environment without robot (for testing)"""
    
    print("\n" + "="*60)
    print("ENVIRONMENT TEST - Solar Panel Array Only")
    print("="*60)
    
    # Create environment without robot
    env = SolarPanelEnvironment(gui=True, urdf_path=None)
    
    # Add dirt
    env.add_dirt_patches()
    
    print("\nEnvironment loaded. Close window to exit.")
    
    import time
    try:
        while True:
            env.step_simulation()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        pass
    
    env.disconnect()


def main():
    """Main entry point"""
    
    print("\n" + "="*60)
    print("SOLAR PANEL CLEANING ROBOT - PYBULLET SIMULATION")
    print("="*60)
    print("\nSelect simulation mode:")
    print("  1 - Autonomous cleaning (robot cleans all panels)")
    print("  2 - Manual control (keyboard control)")
    print("  3 - Environment only (no robot, just panels)")
    print("  q - Quit")
    
    choice = input("\nEnter choice (1/2/3/q): ").strip()
    
    if choice == 'q':
        print("Exiting...")
        return
    
    # Get URDF path
    urdf_path = None
    if choice in ['1', '2']:
        print("\nEnter path to robot URDF file:")
        print("(or press Enter to use: ./solar_robot_correct.urdf)")
        
        user_path = input("URDF path: ").strip()
        
        if user_path:
            urdf_path = user_path
        else:
            urdf_path = "./solar_robot_correct.urdf"
        
        if not os.path.exists(urdf_path):
            print(f"\nERROR: URDF file not found: {urdf_path}")
            print("Please provide correct path to solar_robot_correct.urdf")
            return
    
    # Run selected simulation
    if choice == '1':
        run_autonomous_simulation(urdf_path)
    elif choice == '2':
        run_manual_simulation(urdf_path)
    elif choice == '3':
        run_environment_only()
    else:
        print("Invalid choice")


if __name__ == "__main__":
    main()
