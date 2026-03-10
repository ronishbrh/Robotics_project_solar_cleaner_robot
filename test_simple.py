"""
SOLAR PANEL ROBOT TEST SIMULATION
Stable PyBullet test environment for solar cleaning robot
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import os


# ==========================================
# CREATE WORLD
# ==========================================

def create_world():

    physics = p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Improve physics stability
    p.setPhysicsEngineParameter(
        numSolverIterations=150,
        numSubSteps=4
    )

    # Camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=3.0,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[1, 0, 0]
    )

    # Ground plane
    plane = p.loadURDF("plane.urdf")

    # Panel configuration
    panel_size = [1.6, 1.0, 0.05]
    gap = 0.15

    print("\nCreating solar panel grid...")

    panels = []

    for row in range(3):
        for col in range(4):

            x = row * (panel_size[0] + gap)
            y = col * (panel_size[1] + gap) - 1.5
            z = 0.025

            collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[s / 2 for s in panel_size]
            )

            visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[s / 2 for s in panel_size],
                rgbaColor=[0.1, 0.1, 0.4, 1]
            )

            panel = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=[x, y, z]
            )

            # Add friction to panel
            p.changeDynamics(panel, -1, lateralFriction=1.5)

            panels.append(panel)

            print(f"Panel ({row},{col}) at {x:.2f}, {y:.2f}")

    print(f"\nTotal panels created: {len(panels)}")

    return panels


# ==========================================
# LOAD ROBOT
# ==========================================

def load_robot(urdf_path):

    if not os.path.exists(urdf_path):
        print("URDF file not found!")
        return None, -1, -1

    print("\nLoading robot...")

    start_position = [0.0, -1.5, 0.20]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    robot = p.loadURDF(
        urdf_path,
        basePosition=start_position,
        baseOrientation=start_orientation,
        useFixedBase=False
    )

    print("Robot loaded successfully!")

    # Apply friction to all links
    for i in range(p.getNumJoints(robot)):
        p.changeDynamics(robot, i, lateralFriction=1.8)

    p.changeDynamics(robot, -1, lateralFriction=1.8)

    # Let robot settle
    print("Letting robot settle...")

    for i in range(300):
        p.stepSimulation()
        time.sleep(1 / 240)

    pos, _ = p.getBasePositionAndOrientation(robot)

    print(f"Robot settled at height: {pos[2]:.3f}")

    # Detect wheels
    left_wheel = -1
    right_wheel = -1

    print("\nRobot joints:")

    for i in range(p.getNumJoints(robot)):

        info = p.getJointInfo(robot, i)
        name = info[1].decode("utf-8")

        print(i, name)

        if "wheel_left" in name.lower():
            left_wheel = i

        if "wheel_right" in name.lower():
            right_wheel = i

    print("\nWheel joints detected:")
    print("Left:", left_wheel)
    print("Right:", right_wheel)

    # Extra wheel traction
    if left_wheel >= 0:
        p.changeDynamics(robot, left_wheel, lateralFriction=3.0)

    if right_wheel >= 0:
        p.changeDynamics(robot, right_wheel, lateralFriction=3.0)

    return robot, left_wheel, right_wheel


# ==========================================
# DRIVE FUNCTION
# ==========================================

def drive(robot, left_wheel, right_wheel, speed, force):

    p.setJointMotorControl2(
        robot,
        left_wheel,
        p.VELOCITY_CONTROL,
        targetVelocity=speed,
        force=force
    )

    p.setJointMotorControl2(
        robot,
        right_wheel,
        p.VELOCITY_CONTROL,
        targetVelocity=speed,
        force=force
    )


# ==========================================
# TEST MOVEMENT
# ==========================================

def test_motion(robot, left_wheel, right_wheel):

    if robot is None or left_wheel < 0 or right_wheel < 0:
        print("Robot or wheels not detected!")
        return

    print("\nTesting robot movement...\n")

    start_pos, _ = p.getBasePositionAndOrientation(robot)

    wheel_speed = 8.0
    motor_force = 200

    steps = 720

    for i in range(steps):

        drive(robot, left_wheel, right_wheel, wheel_speed, motor_force)

        p.stepSimulation()
        time.sleep(1 / 240)

        if i % 240 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot)
            print("Position:", pos)

    # Stop robot
    drive(robot, left_wheel, right_wheel, 0, 0)

    for _ in range(240):
        p.stepSimulation()
        time.sleep(1 / 240)

    end_pos, _ = p.getBasePositionAndOrientation(robot)

    distance = np.sqrt(
        (end_pos[0] - start_pos[0]) ** 2 +
        (end_pos[1] - start_pos[1]) ** 2
    )

    print("\nDistance moved:", round(distance, 3), "meters")

    if distance > 0.1:
        print("SUCCESS: Robot moved correctly!")
    else:
        print("WARNING: Robot barely moved.")


# ==========================================
# MAIN
# ==========================================

def main():

    print("\n===================================")
    print("SOLAR CLEANING ROBOT TEST")
    print("===================================")

    urdf_path = "./solar_robot_pybullet.urdf"

    panels = create_world()

    robot, left_wheel, right_wheel = load_robot(urdf_path)

    time.sleep(2)

    test_motion(robot, left_wheel, right_wheel)

    print("\nSimulation running. Press CTRL+C to exit.")

    try:

        while True:
            p.stepSimulation()
            time.sleep(1 / 240)

    except KeyboardInterrupt:
        pass

    p.disconnect()


# ==========================================

if __name__ == "__main__":
    main()