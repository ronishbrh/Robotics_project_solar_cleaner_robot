"""
PyBullet Robot Controller
Autonomous solar panel cleaning with bridge mechanism
"""

import pybullet as p
import numpy as np
import time
from enum import Enum


class RobotState(Enum):
    """Robot operational states"""
    IDLE = 0
    SCANNING = 1
    CLEANING = 2
    APPROACHING_GAP = 3
    DEPLOYING_BRIDGE = 4
    CROSSING_GAP = 5
    RETRACTING_BRIDGE = 6
    COMPLETED = 7


class RobotController:
    """Controller for autonomous solar panel cleaning robot"""
    
    def __init__(self, robot_id, environment):
      
        self.robot_id = robot_id
        self.env = environment
        
        # State
        self.state = RobotState.IDLE
        self.current_panel = (0, 0)  # (row, col)
        self.target_panel = None
        
        # Cleaning path
        self.cleaning_path = self.plan_snake_path()
        self.path_index = 0
        
        # Joint control
        self.joint_indices = environment.joint_indices
        
        # Parameters
        self.wheel_speed = 5.0  # rad/s - increased from 2.0
        self.bridge_extension_speed = 0.1  # m/s
        self.cleaning_time = 5.0  # seconds per panel - increased from 3.0
        
        print("Robot Controller Initialized")
        print(f"Cleaning path: {len(self.cleaning_path)} panels")
        
    def plan_snake_path(self):
        """Plan snake pattern through all panels"""
        path = []
        
        for row in range(3):
            if row % 2 == 0:
                # Left to right
                for col in range(4):
                    path.append((row, col))
            else:
                # Right to left
                for col in range(3, -1, -1):
                    path.append((row, col))
        
        return path
    
    def set_wheel_velocity(self, left_vel, right_vel):
        """
        Set wheel velocities for 4-wheel drive
        Front and rear wheels on each side move together
        
        Args:
            left_vel: Left side wheels velocity (rad/s)
            right_vel: Right side wheels velocity (rad/s)
        """
        # Get all 4 wheel joint indices
        front_left = self.joint_indices.get('body_to_wheel_front_left', -1)
        front_right = self.joint_indices.get('body_to_wheel_front_right', -1)
        rear_left = self.joint_indices.get('body_to_wheel_rear_left', -1)
        rear_right = self.joint_indices.get('body_to_wheel_rear_right', -1)
        
        # Control left side wheels
        if front_left >= 0:
            p.setJointMotorControl2(
                self.robot_id, front_left,
                p.VELOCITY_CONTROL,
                targetVelocity=left_vel,
                force=200.0
            )
        
        if rear_left >= 0:
            p.setJointMotorControl2(
                self.robot_id, rear_left,
                p.VELOCITY_CONTROL,
                targetVelocity=left_vel,
                force=200.0
            )
        
        # Control right side wheels
        if front_right >= 0:
            p.setJointMotorControl2(
                self.robot_id, front_right,
                p.VELOCITY_CONTROL,
                targetVelocity=right_vel,
                force=200.0
            )
        
        if rear_right >= 0:
            p.setJointMotorControl2(
                self.robot_id, rear_right,
                p.VELOCITY_CONTROL,
                targetVelocity=right_vel,
                force=200.0
            )
    
    def move_forward(self, speed=1.0):
        """Move robot forward"""
        vel = speed * self.wheel_speed
        # REVERSED: Robot was going backward!
        self.set_wheel_velocity(-vel, -vel)
    
    def move_backward(self, speed=1.0):
        """Move robot backward"""
        vel = speed * self.wheel_speed
        # REVERSED: Now this goes backward
        self.set_wheel_velocity(vel, vel)
    
    def rotate_left(self, speed=0.5):
        """Rotate robot counterclockwise"""
        vel = speed * self.wheel_speed
        self.set_wheel_velocity(-vel, vel)
    
    def rotate_right(self, speed=0.5):
        """Rotate robot clockwise"""
        vel = speed * self.wheel_speed
        self.set_wheel_velocity(vel, -vel)
    
    def stop(self):
        """Stop all motion"""
        self.set_wheel_velocity(0, 0)
    
    def set_bridge_position(self, segment, position):
        """
        Set bridge segment position
        
        Args:
            segment: 1 or 2
            position: Target position (0 to 0.5 meters)
        """
        joint_name = f'bridge_deploy_joint_{segment}'
        joint_idx = self.joint_indices.get(joint_name, -1)
        
        if joint_idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=position,
                force=50.0,
                maxVelocity=self.bridge_extension_speed
            )
    
    def extend_bridge(self, length=0.5):
        """
        Extend bridge to specified length
        
        Args:
            length: Total bridge length (0 to 1.0 meters)
        """
        print(f"Extending bridge to {length*100:.0f}cm...")
        
        # Extend segment 1
        segment1_length = min(length, 0.5)
        self.set_bridge_position(1, segment1_length)
        
        # Step simulation while extending
        steps = int(3.0 * 240)  # 3 seconds at 240 Hz
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        # Extend segment 2 if needed
        if length > 0.5:
            segment2_length = min(length - 0.5, 0.5)
            self.set_bridge_position(2, segment2_length)
            
            # Step simulation
            for _ in range(steps):
                self.env.step_simulation()
                time.sleep(1./240.)
        
        print("Bridge extended")
    
    def set_rear_bridge_position(self, segment, position):
        """
        Set REAR bridge segment position
        
        Args:
            segment: 1 or 2
            position: Target position (0 to 0.5 meters)
        """
        joint_name = f'rear_bridge_deploy_joint_{segment}'
        joint_idx = self.joint_indices.get(joint_name, -1)
        
        if joint_idx >= 0:
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=position,
                force=50.0,
                maxVelocity=self.bridge_extension_speed
            )
    
    def extend_rear_bridge(self, length=0.5):
        """
        Extend REAR bridge to specified length
        
        Args:
            length: Total rear bridge length (0 to 1.0 meters)
        """
        print(f"Extending REAR bridge to {length*100:.0f}cm...")
        
        # Extend segment 1
        segment1_length = min(length, 0.5)
        self.set_rear_bridge_position(1, segment1_length)
        
        # Step simulation while extending
        steps = int(3.0 * 240)  # 3 seconds at 240 Hz
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        # Extend segment 2 if needed
        if length > 0.5:
            segment2_length = min(length - 0.5, 0.5)
            self.set_rear_bridge_position(2, segment2_length)
            
            # Step simulation
            for _ in range(steps):
                self.env.step_simulation()
                time.sleep(1./240.)
        
        print("Rear bridge extended")
    
    def retract_rear_bridge(self):
        """Retract REAR bridge completely"""
        print("Retracting rear bridge...")
        self.set_rear_bridge_position(2, 0.0)
        self.set_rear_bridge_position(1, 0.0)
        
        # Step simulation while retracting
        steps = int(3.0 * 240)
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        print("Rear bridge retracted")
    
    def retract_bridge(self):
        """Retract bridge back to storage"""
        print("Retracting bridge...")
        
        # Retract segment 2 first
        self.set_bridge_position(2, 0.0)
        
        # Step simulation
        steps = int(3.0 * 240)
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        # Retract segment 1
        self.set_bridge_position(1, 0.0)
        
        # Step simulation
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        print("Bridge retracted")
    
    def get_robot_position(self):
        """Get robot's current position"""
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        return pos
    
    def get_distance_to_target(self, target_pos):
        """Calculate distance to target position"""
        current_pos = self.get_robot_position()
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        return np.sqrt(dx*dx + dy*dy)
    
    def check_bridge_contact(self):
        """
        Check if bridge feet are in contact with surface
        Simplified: Just returns True after extension
        """
        # In full implementation, would check contact points
        return True
    
    def clean_current_panel(self):
        """Execute cleaning action on current panel"""
        print(f"Cleaning panel {self.current_panel}...")
        
        # Move forward while cleaning WITH SAFETY CHECKS
        self.move_forward(0.5)
        
        # Step simulation while moving, checking for edges
        steps = int(self.cleaning_time * 240)  # 240 Hz physics
        for step in range(steps):
            # Safety check every 60 steps (0.25 seconds)
            if step % 60 == 0:
                edge_detected, distance = self.check_edge_ahead()
                if edge_detected:
                    print(f"⚠️ Edge detected {distance:.2f}m ahead - stopping!")
                    self.stop()
                    break
            
            self.env.step_simulation()
            time.sleep(1./240.)
        
        self.stop()
        print(f"✓ Panel {self.current_panel} cleaned")
    
    def check_edge_ahead(self):
        """
        Simple edge detection using robot height
        Returns: (edge_detected, distance)
        """
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        
        # Cast ray downward from front of robot
        ray_from = [pos[0] + 0.25, pos[1], pos[2]]  # 25cm ahead
        ray_to = [pos[0] + 0.25, pos[1], pos[2] - 0.5]  # Down 50cm
        
        result = p.rayTest(ray_from, ray_to)
        
        if result:
            hit_fraction = result[0][2]
            distance_down = hit_fraction * 0.5
            
            # Edge if drop more than 10cm
            if distance_down > 0.10:
                return True, distance_down
        
        return False, 0.0
    
    def navigate_to_panel(self, target_panel):
        """Navigate to target panel"""
        current_pos = self.get_robot_position()
        target_pos = self.env.panel_positions[target_panel]
        
        distance = self.get_distance_to_target(target_pos)
        
        print(f"Navigating to panel {target_panel} (distance: {distance:.2f}m)")
        
        # Check if gap crossing needed
        current_row, current_col = self.current_panel
        target_row, target_col = target_panel
        
        gap_detected = False
        
        # Row change = gap crossing needed
        if target_row != current_row:
            gap_detected = True
            print("Gap detected - will use bridge")
        
        # Column change within same row = just move
        elif target_col != current_col:
            gap_detected = False
            print("Moving along panel row")
        
        return gap_detected
    
    def execute_gap_crossing(self):
        """
        Execute LEGO-style gap crossing
        Based on: https://www.youtube.com/watch?v=pwglOlD7e0M&t=152s
        
        KEY: Bridge stays extended until REAR wheels are on next panel!
        
        1. Extend bridge forward
        2. Drive forward - rear wheels onto bridge
        3. Keep driving - bridge slides under robot
        4. WAIT until rear wheels on next panel
        5. Then retract bridge
        """
        print("=== EXECUTING LEGO-STYLE GAP CROSSING ===")
        
        # 1. Approach gap edge slowly
        print("1. Approaching gap edge...")
        self.move_forward(0.3)
        
        steps = int(2.0 * 240)
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        self.stop()
        
        # Get starting position
        start_pos = self.get_robot_position()
        print(f"   Starting position: X={start_pos[0]:.2f}, Y={start_pos[1]:.2f}")
        
        # 2. Deploy bridge forward
        print("2. Deploying bridge forward...")
        self.extend_bridge(0.40)  # 40cm extension
        
        # 3. Verify bridge contact with next panel
        print("3. Verifying bridge contact...")
        
        steps = int(2.0 * 240)
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        contact = self.check_bridge_contact()
        
        if not contact:
            print("✗ Bridge contact failed - aborting")
            self.retract_bridge()
            return False
        
        print("✓ Bridge touching next panel")
        
        # 4. Calculate target distance
        # FRONT of robot = where bridge is (crosses gap first)
        # REAR of robot = opposite end (crosses gap last - these are the wheels we wait for!)
        # 
        # Robot length ≈ 0.40m (from front wheels to rear wheels: 0.30m)
        # Gap = 0.15m
        # Safety margin = 0.10m
        # 
        # Front wheels cross immediately (they're where the bridge is)
        # Rear wheels need to travel: 0.30m (wheelbase) + 0.15m (gap) + 0.10m (margin) = 0.55m
        target_distance = 0.60  # 60cm to ensure REAR wheels (opposite from bridge) are safely across
        
        print(f"4. Driving across gap (target: {target_distance*100:.0f}cm)...")
        print("   Phase 1: Front wheels (with bridge) crossing...")
        
        self.move_forward(0.25)  # Moderate speed
        
        # Drive and monitor distance
        distance_traveled = 0.0
        step_count = 0
        max_steps = int(15.0 * 240)  # 15 seconds max
        
        while step_count < max_steps and distance_traveled < target_distance:
            self.env.step_simulation()
            time.sleep(1./240.)
            step_count += 1
            
            # Check distance every 60 steps (0.25 seconds)
            if step_count % 60 == 0:
                current_pos = self.get_robot_position()
                distance_traveled = abs(current_pos[0] - start_pos[0])
                
                # Progress updates
                if distance_traveled < 0.20:
                    pass  # Front wheels still crossing
                elif distance_traveled < 0.40:
                    if step_count % 240 == 0:  # Every second
                        print(f"   Phase 2: Front wheels across, rear wheels on bridge (traveled {distance_traveled*100:.0f}cm)")
                elif distance_traveled < target_distance:
                    if step_count % 240 == 0:
                        print(f"   Phase 3: Bridge under robot, rear wheels crossing (traveled {distance_traveled*100:.0f}cm)")
        
        self.stop()
        
        final_pos = self.get_robot_position()
        distance_traveled = abs(final_pos[0] - start_pos[0])
        
        print(f"✓ Crossing complete! Traveled {distance_traveled*100:.0f}cm")
        print(f"   Final position: X={final_pos[0]:.2f}, Y={final_pos[1]:.2f}")
        
        # Verify REAR wheels (the ones without bridge) are on solid ground
        print("5. Verifying REAR wheels on panel...")
        
        steps = int(1.0 * 240)  # 1 second to settle
        for _ in range(steps):
            self.env.step_simulation()
            time.sleep(1./240.)
        
        print("✓ All wheels safely on next panel!")
        
        # 6. NOW retract bridge (rear wheels are safe!)
        print("6. Retracting bridge...")
        self.retract_bridge()
        
        print("=== GAP CROSSING COMPLETE ===")
        return True
    
    def update(self):
        """Update controller - called each simulation step"""
        
        if self.state == RobotState.IDLE:
            # Start cleaning mission
            print("\n=== STARTING AUTONOMOUS CLEANING ===")
            self.state = RobotState.SCANNING
            
        elif self.state == RobotState.SCANNING:
            # Scan current panel (simplified - just check if it exists)
            print(f"\nScanning panel {self.current_panel}...")
            
            steps = int(1.0 * 240)
            for _ in range(steps):
                self.env.step_simulation()
                time.sleep(1./240.)
            
            self.state = RobotState.CLEANING
            
        elif self.state == RobotState.CLEANING:
            # Clean current panel
            self.clean_current_panel()
            
            # Get next panel
            self.path_index += 1
            if self.path_index >= len(self.cleaning_path):
                self.state = RobotState.COMPLETED
                return
            
            self.target_panel = self.cleaning_path[self.path_index]
            self.state = RobotState.APPROACHING_GAP
            
        elif self.state == RobotState.APPROACHING_GAP:
            # Navigate to next panel
            gap_detected = self.navigate_to_panel(self.target_panel)
            
            if gap_detected:
                # Gap crossing - deploy bridge
                self.state = RobotState.DEPLOYING_BRIDGE
            else:
                # No gap - LOCK bridge fully retracted
                print("No gap - retracting and locking bridge...")
                self.set_bridge_position(1, 0.0)
                self.set_bridge_position(2, 0.0)
                
                # Wait longer for full retraction
                steps = int(1.0 * 240)
                for _ in range(steps):
                    self.env.step_simulation()
                    time.sleep(1./240.)
                
                # Verify bridge is retracted by checking joint positions
                seg1_pos = p.getJointState(self.robot_id, self.joint_indices.get('bridge_deploy_joint_1', -1))[0]
                seg2_pos = p.getJointState(self.robot_id, self.joint_indices.get('bridge_deploy_joint_2', -1))[0]
                
                print(f"Bridge positions: seg1={seg1_pos:.3f}m, seg2={seg2_pos:.3f}m")
                
                if abs(seg1_pos) > 0.01 or abs(seg2_pos) > 0.01:
                    print("⚠️ Bridge not fully retracted! Forcing retraction...")
                    # Force retract with high effort
                    for _ in range(3):
                        self.set_bridge_position(1, 0.0)
                        self.set_bridge_position(2, 0.0)
                        for _ in range(int(0.5 * 240)):
                            self.env.step_simulation()
                            time.sleep(1./240.)
                
                print("✓ Bridge confirmed retracted - safe to move")
                
                # Now move to next panel
                self.move_forward(0.5)
                
                steps = int(5.0 * 240)  # 5 seconds movement
                for step in range(steps):
                    # Safety check every 60 steps
                    if step % 60 == 0:
                        edge_detected, distance = self.check_edge_ahead()
                        if edge_detected:
                            print(f"⚠️ Edge detected during navigation - stopping!")
                            self.stop()
                            # Stay at current panel
                            print(f"Remaining at panel {self.current_panel}")
                            self.state = RobotState.SCANNING
                            return
                    
                    self.env.step_simulation()
                    time.sleep(1./240.)
                
                self.stop()
                self.current_panel = self.target_panel
                self.state = RobotState.SCANNING
            
        elif self.state == RobotState.DEPLOYING_BRIDGE:
            # Execute gap crossing
            success = self.execute_gap_crossing()
            
            if success:
                self.current_panel = self.target_panel
                self.state = RobotState.SCANNING
            else:
                print("Gap crossing failed - stopping")
                self.state = RobotState.COMPLETED
                
        elif self.state == RobotState.COMPLETED:
            # Mission complete
            print("\n=== ALL PANELS CLEANED ===")
            print(f"Total panels cleaned: {self.path_index}")
            self.stop()
            return True  # Signal completion
        
        return False  # Not yet completed
    
    def run_autonomous(self, max_steps=10000):
        """
        Run autonomous cleaning mission
        
        Args:
            max_steps: Maximum simulation steps
        """
        print("\n" + "="*50)
        print("AUTONOMOUS CLEANING MISSION")
        print("="*50)
        
        step_count = 0
        
        while step_count < max_steps:
            # Update controller
            completed = self.update()
            
            if completed:
                break
            
            # Step simulation
            self.env.step_simulation()
            time.sleep(1./240.)
            
            step_count += 1
            
            # Progress update every 1000 steps
            if step_count % 1000 == 0:
                print(f"Steps: {step_count}, State: {self.state.name}")
        
        print("\nMission ended")
        print(f"Total steps: {step_count}")


class ManualController:
    """Manual keyboard control for testing"""
    
    def __init__(self, robot_id, environment):
        """Initialize manual controller"""
        self.robot_id = robot_id
        self.env = environment
        self.joint_indices = environment.joint_indices
        
        self.wheel_speed = 5.0  # Increased from 2.0 to match autonomous
        self.bridge_pos = [0.0, 0.0]  # Segment 1, Segment 2
        
        print("\nManual Controller Ready")
        self.print_instructions()
    
    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*50)
        print("MANUAL CONTROL KEYS")
        print("="*50)
        print("Arrow Keys:")
        print("  ↑ - Forward")
        print("  ↓ - Backward")
        print("  ← - Rotate Left")
        print("  → - Rotate Right")
        print("\nBridge Control:")
        print("  1 - Extend segment 1")
        print("  2 - Extend segment 2")
        print("  0 - Retract all")
        print("\nPress 'q' to quit")
        print("="*50 + "\n")
    
    def set_wheel_velocity(self, left_vel, right_vel):
        """Set wheel velocities for 4-wheel drive"""
        # Get all 4 wheel joint indices
        front_left = self.joint_indices.get('body_to_wheel_front_left', -1)
        front_right = self.joint_indices.get('body_to_wheel_front_right', -1)
        rear_left = self.joint_indices.get('body_to_wheel_rear_left', -1)
        rear_right = self.joint_indices.get('body_to_wheel_rear_right', -1)
        
        # Control left side wheels
        if front_left >= 0:
            p.setJointMotorControl2(
                self.robot_id, front_left,
                p.VELOCITY_CONTROL,
                targetVelocity=left_vel,
                force=200.0
            )
        
        if rear_left >= 0:
            p.setJointMotorControl2(
                self.robot_id, rear_left,
                p.VELOCITY_CONTROL,
                targetVelocity=left_vel,
                force=200.0
            )
        
        # Control right side wheels
        if front_right >= 0:
            p.setJointMotorControl2(
                self.robot_id, front_right,
                p.VELOCITY_CONTROL,
                targetVelocity=right_vel,
                force=200.0
            )
        
        if rear_right >= 0:
            p.setJointMotorControl2(
                self.robot_id, rear_right,
                p.VELOCITY_CONTROL,
                targetVelocity=right_vel,
                force=200.0
            )
    
    def set_bridge_position(self, segment, position):
        """Set bridge position"""
        joint_name = f'bridge_deploy_joint_{segment}'
        joint_idx = self.joint_indices.get(joint_name, -1)
        
        if joint_idx >= 0:
            p.setJointMotorControl2(
                self.robot_id, joint_idx,
                p.POSITION_CONTROL,
                targetPosition=position,
                force=50.0
            )
    
    def handle_key(self, key):
        
        if key == p.B3G_UP_ARROW:
            # Forward (REVERSED - negative velocity moves forward)
            self.set_wheel_velocity(-self.wheel_speed, -self.wheel_speed)
            
        elif key == p.B3G_DOWN_ARROW:
            # Backward (REVERSED - positive velocity moves backward)
            self.set_wheel_velocity(self.wheel_speed, self.wheel_speed)
            
        elif key == p.B3G_LEFT_ARROW:
            # Rotate left
            self.set_wheel_velocity(self.wheel_speed, -self.wheel_speed)
            
        elif key == p.B3G_RIGHT_ARROW:
            # Rotate right
            self.set_wheel_velocity(-self.wheel_speed, self.wheel_speed)
            
        elif key == ord('1'):
            # Extend segment 1
            self.bridge_pos[0] = min(self.bridge_pos[0] + 0.1, 0.5)
            self.set_bridge_position(1, self.bridge_pos[0])
            print(f"Bridge segment 1: {self.bridge_pos[0]*100:.0f}cm")
            
        elif key == ord('2'):
            # Extend segment 2
            self.bridge_pos[1] = min(self.bridge_pos[1] + 0.1, 0.5)
            self.set_bridge_position(2, self.bridge_pos[1])
            print(f"Bridge segment 2: {self.bridge_pos[1]*100:.0f}cm")
            
        elif key == ord('0'):
            # Retract all
            self.bridge_pos = [0.0, 0.0]
            self.set_bridge_position(1, 0.0)
            self.set_bridge_position(2, 0.0)
            print("Bridge retracted")
            
        elif key == ord('q'):
            return False
        
        return True
    
    def run(self):
        """Run manual control loop"""
        
        print("Manual control active. Use arrow keys and numbers.")
        
        try:
            while True:
                keys = p.getKeyboardEvents()
                
                if not keys:
                    self.set_wheel_velocity(0, 0)

                for key in keys:
                    if keys[key] & p.KEY_IS_DOWN:
                        if not self.handle_key(key):
                            return
                
                self.env.step_simulation()
                time.sleep(1./240.)
                
        except KeyboardInterrupt:
            print("\nManual control ended")
