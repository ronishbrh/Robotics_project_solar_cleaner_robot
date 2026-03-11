"""
PyBullet Simulation Environment
Solar Panel Cleaning Robot with Array
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import os


class SolarPanelEnvironment:

    def __init__(self, gui=True, urdf_path=None):
        if gui:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(0, 0, -9.81)

        self.plane_id = p.loadURDF("plane.urdf")

        self.setup_camera()

        self.panel_ids = []
        self.create_solar_panel_array()

        self.robot_id = None
        self.urdf_path = urdf_path
        if urdf_path and os.path.exists(urdf_path):
            self.load_robot(urdf_path)

        # 3 rows x 4 columns
        self.panel_positions = self.get_panel_positions()

        p.setTimeStep(1./240.)
        p.setRealTimeSimulation(0)

        print("PyBullet Environment Initialized")
        print(f"Solar Panels: {len(self.panel_ids)}")

    def setup_camera(self):
        """Setup camera view"""
        p.resetDebugVisualizerCamera(
            cameraDistance=5.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[2, 0, 1]
        )

    def create_solar_panel_array(self):

        panel_length = 1.6
        panel_width = 1.0
        panel_thickness = 0.05

        gap = 0.15

        inclination = 0.0  # Changed from 0.436332 to 0.0

        row_spacing = panel_length + gap
        col_spacing = panel_width + gap

        panel_count = 0
        for row in range(3):
            for col in range(4):
                # Calculate position
                x = row * row_spacing
                y = col * col_spacing - 1.5  # Center around 0
                z = panel_thickness / 2 + 0.01  # Just above ground

                panel_id = self.create_panel(
                    position=[x, y, z],
                    length=panel_length,
                    width=panel_width,
                    thickness=panel_thickness,
                    inclination=inclination
                )

                self.panel_ids.append({
                    'id': panel_id,
                    'row': row,
                    'col': col,
                    'position': [x, y, z],
                    'cleaned': False
                })

                panel_count += 1

        print(f"Created {panel_count} solar panels in 3x4 array (FLAT)")

    def create_panel(self, position, length, width, thickness, inclination):
        """Create a single solar panel"""

        # Create collision shape
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[length/2, width/2, thickness/2]
        )

        # Create visual shape dark blue panel
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[length/2, width/2, thickness/2],
            rgbaColor=[0.1, 0.1, 0.4, 1.0],
            specularColor=[0.8, 0.8, 0.9]
        )

        # Calculate orientation (tilted 25 degrees)
        orientation = p.getQuaternionFromEuler([0, inclination, 0])

        # Create multi-body
        panel_id = p.createMultiBody(
            baseMass=0,  # Static
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
            baseOrientation=orientation
        )

        # Set friction (smooth glass surface)
        p.changeDynamics(
            panel_id, -1,
            lateralFriction=0.3,
            spinningFriction=0.1,
            rollingFriction=0.01
        )

        return panel_id

    def load_robot(self, urdf_path):

        # Starting position (on first panel - FLAT orientation)
        # First panel is at (0, -1.5, 0.03)
        # Robot needs to be LOW so wheels touch ground
        start_pos = [0.0, -1.5, 0.08]  

        # Face robot along positive X axis (forward = +X direction)
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation

        try:
            self.robot_id = p.loadURDF(
                urdf_path,
                basePosition=start_pos,
                baseOrientation=start_orientation,
                useFixedBase=False,
                flags=p.URDF_USE_INERTIA_FROM_FILE
            )

            print(f"Robot loaded: ID = {self.robot_id}")
            print(f"Robot spawn position: {start_pos}")
            print(f"Robot orientation: Facing +X (forward)")

            # Let robot settle onto panel (important!)
            for _ in range(500):  # 500 steps = 2+ seconds settling
                p.stepSimulation()

            # Get joint info
            self.joint_indices = {}
            num_joints = p.getNumJoints(self.robot_id)

            print(f"\nRobot has {num_joints} joints:")
            for i in range(num_joints):
                joint_info = p.getJointInfo(self.robot_id, i)
                joint_name = joint_info[1].decode('utf-8')
                joint_type = joint_info[2]

                self.joint_indices[joint_name] = i

                print(f"  [{i}] {joint_name} (Type: {joint_type})")

            # Set friction for grip 
            self.set_suction_friction()

            # Enable wheel motors
            self.enable_wheel_motors()

            # Initialize bridges to retracted position
            self.initialize_bridges()

            final_pos, final_orn = p.getBasePositionAndOrientation(self.robot_id)
            euler = p.getEulerFromQuaternion(final_orn)
            print(f"\nRobot settled at: {final_pos}")
            print(f"Robot orientation (RPY): {euler}")

            contacts = p.getContactPoints(self.robot_id)
            print(f"Contact points: {len(contacts)}")

        except Exception as e:
            print(f"Error loading robot: {e}")
            self.robot_id = None

    def set_suction_friction(self):

        if self.robot_id is None:
            return

        # Find suction pad link
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')

            if 'suction' in link_name.lower():
                # High friction but not too high (was 5.0, now 2.0)
                p.changeDynamics(
                    self.robot_id, i,
                    lateralFriction=2.0,
                    spinningFriction=1.0,
                    rollingFriction=0.5,
                    contactStiffness=50000,
                    contactDamping=500
                )
                print(f"Set high friction on: {link_name}")

            # Low friction for bridge support pads (smooth sliding)
            if 'bridge_foot' in link_name.lower():
                p.changeDynamics(
                    self.robot_id, i,
                    lateralFriction=0.3,
                    spinningFriction=0.1,
                    rollingFriction=0.1,
                    contactStiffness=10000,
                    contactDamping=100
                )
                print(f"Set low friction (sliding pad) on: {link_name}")

        # Also set on base link (main suction pad)
        p.changeDynamics(
            self.robot_id, -1,
            lateralFriction=2.0,
            spinningFriction=1.0,
            rollingFriction=0.5,
            contactStiffness=50000,
            contactDamping=500
        )

    def enable_wheel_motors(self):

        if self.robot_id is None:
            return

        # Find and configure all wheel joints
        wheel_count = 0
        for joint_name, joint_idx in self.joint_indices.items():
            if 'wheel' in joint_name.lower():
                # Set zero damping and friction for free rotation
                p.changeDynamics(
                    self.robot_id, joint_idx,
                    linearDamping=0.0,
                    angularDamping=0.0,
                    jointDamping=0.0,
                    lateralFriction=2.5
                )
                # Enable motor control
                p.setJointMotorControl2(
                    self.robot_id, joint_idx,
                    p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=0  # Start with no force
                )
                wheel_count += 1
                print(f"Wheel motor enabled: {joint_name}")

        print(f"Total wheels configured: {wheel_count}")

    def initialize_bridges(self):

        if self.robot_id is None:
            return

        print("\nInitializing bridges to retracted position...")

        # Retract FRONT bridge
        front_joints = ['bridge_deploy_joint_1', 'bridge_deploy_joint_2']
        for joint_name in front_joints:
            joint_idx = self.joint_indices.get(joint_name, -1)
            if joint_idx >= 0:
                p.resetJointState(self.robot_id, joint_idx, 0.0)
                p.setJointMotorControl2(
                    self.robot_id, joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=0.0,
                    force=50.0
                )
                print(f"  Front bridge {joint_name} retracted")

        # Retract REAR bridge
        rear_joints = ['rear_bridge_deploy_joint_1', 'rear_bridge_deploy_joint_2']
        for joint_name in rear_joints:
            joint_idx = self.joint_indices.get(joint_name, -1)
            if joint_idx >= 0:
                p.resetJointState(self.robot_id, joint_idx, 0.0)
                p.setJointMotorControl2(
                    self.robot_id, joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=0.0,
                    force=50.0
                )
                print(f"  Rear bridge {joint_name} retracted")

        # Let physics settle
        for _ in range(100):
            p.stepSimulation()

        print("Both bridges initialized to retracted position")

    def get_panel_positions(self):
        positions = {}
        for panel in self.panel_ids:
            key = (panel['row'], panel['col'])
            positions[key] = panel['position']
        return positions

    def get_joint_index(self, joint_name):
        """Get joint index by name"""
        return self.joint_indices.get(joint_name, -1)

    def step_simulation(self):
        p.stepSimulation()

    def disconnect(self):
        p.disconnect()

    def get_robot_state(self):
        """Get current robot state"""
        if self.robot_id is None:
            return None

        # Base position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)

        # Joint states
        joint_states = {}
        for name, idx in self.joint_indices.items():
            state = p.getJointState(self.robot_id, idx)
            joint_states[name] = {
                'position': state[0],
                'velocity': state[1]
            }

        return {
            'position': pos,
            'orientation': orn,
            'joints': joint_states
        }

    def add_dirt_patches(self):
        """Add visual dirt patches on some panels"""

        dirt_panels = [
            (0, 0), (0, 1),  # Row 1
            (1, 1), (1, 2),  # Row 2
            (2, 0), (2, 3)   # Row 3
        ]

        for row, col in dirt_panels:
            panel_pos = self.panel_positions[(row, col)]

            # small brown sphere as dirt
            dirt_visual = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.1,
                rgbaColor=[0.6, 0.4, 0.3, 0.9]
            )

            # Offset slightly above panel
            dirt_pos = [
                panel_pos[0] - 0.2,
                panel_pos[1] + 0.1,
                panel_pos[2] + 0.05
            ]

            dirt_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=dirt_visual,
                basePosition=dirt_pos
            )

            print(f"Added dirt on panel ({row}, {col})")


def test_environment():
    print("Testing PyBullet Environment...")

    env = SolarPanelEnvironment(gui=True)

    env.add_dirt_patches()

    print("\nSimulation running. Close window to exit.")

    try:
        while True:
            env.step_simulation()
            time.sleep(1./240.)

    except KeyboardInterrupt:
        print("\nShutting down...")

    env.disconnect()


if __name__ == "__main__":
    test_environment()
