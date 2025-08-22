import dataclasses
import threading
import time
from typing import Dict, Optional

import numpy as np
from pyquaternion import Quaternion

from gello.robots.robot import Robot


def _aa_from_quat(quat: np.ndarray) -> np.ndarray:
    """Convert a quaternion to an axis-angle representation.

    Args:
        quat (np.ndarray): The quaternion to convert.

    Returns:
        np.ndarray: The axis-angle representation of the quaternion.
    """
    assert quat.shape == (4,), "Input quaternion must be a 4D vector."
    norm = np.linalg.norm(quat)
    assert norm != 0, "Input quaternion must not be a zero vector."
    quat = quat / norm  # Normalize the quaternion

    Q = Quaternion(w=quat[3], x=quat[0], y=quat[1], z=quat[2])
    angle = Q.angle
    axis = Q.axis
    aa = axis * angle
    return aa


def _quat_from_aa(aa: np.ndarray) -> np.ndarray:
    """Convert an axis-angle representation to a quaternion.

    Args:
        aa (np.ndarray): The axis-angle representation to convert.

    Returns:
        np.ndarray: The quaternion representation of the axis-angle.
    """
    assert aa.shape == (3,), "Input axis-angle must be a 3D vector."
    norm = np.linalg.norm(aa)
    assert norm != 0, "Input axis-angle must not be a zero vector."
    axis = aa / norm  # Normalize the axis-angle

    Q = Quaternion(axis=axis, angle=norm)
    quat = np.array([Q.x, Q.y, Q.z, Q.w])
    return quat


@dataclasses.dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    z: float
    gripper: float
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    aa: np.ndarray

    @staticmethod
    def from_robot(
        cartesian: np.ndarray,
        joints: np.ndarray,
        gripper: float,
        aa: np.ndarray,
    ) -> "RobotState":
        return RobotState(
            cartesian[0],
            cartesian[1],
            cartesian[2],
            gripper,
            joints[0],
            joints[1],
            joints[2],
            joints[3],
            joints[4],
            joints[5],
            aa,
        )

    def cartesian_pos(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def quat(self) -> np.ndarray:
        return _quat_from_aa(self.aa)

    def joints(self) -> np.ndarray:
        return np.array([self.j1, self.j2, self.j3, self.j4, self.j5, self.j6])

    def gripper_pos(self) -> float:
        return self.gripper


class Rate:
    def __init__(self, *, duration):
        self.duration = duration
        self.last = time.time()

    def sleep(self, duration=None) -> None:
        duration = self.duration if duration is None else duration
        assert duration >= 0
        now = time.time()
        passed = now - self.last
        remaining = duration - passed
        assert passed >= 0
        if remaining > 0.0001:
            time.sleep(remaining)
        self.last = time.time()


class XArmRobot(Robot):
    GRIPPER_OPEN = 800
    GRIPPER_CLOSE = 0
    #  MAX_DELTA = 0.2
    DEFAULT_MAX_DELTA = 0.05
    DEFAULT_WORKSPACE_LIMITS = {
            'x_min': -308,  # mm
            'x_max': 962,  # mm
            'y_min': -615,  # mm
            'y_max': 610,  # mm
            'z_min': -32,  # mm
            'z_max': 450,  # mm
        }
    INITIAL_POSITION = [377.49, 2.07, -4.87]
    INITIAL_ANGLES =[0.015336, 0.062893, -0.004608, 0.058289, -0.029144, -0.159536, 0.0]
    TOLERANCE = 0.05

    def num_dofs(self) -> int:
        return 7

    def get_joint_state(self) -> np.ndarray:
        state = self.get_state()
        gripper = state.gripper_pos()
        all_dofs = np.concatenate([state.joints(), np.array([gripper])])
        return all_dofs

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        if len(joint_state) == 6:
            self.set_command(joint_state, None)
        elif len(joint_state) == 7:
            self.set_command(joint_state[:6], joint_state[6])
        else:
            raise ValueError(
                f"Invalid joint state: {joint_state}, len={len(joint_state)}"
            )

    def stop(self):
        self.stop_position_monitoring()  # Stop monitoring first
        self.running = False  # This will also stop the emergency stop listener
        if self.robot is not None:
            self.robot.disconnect()

        if self.command_thread is not None:
            self.command_thread.join()
            
        print("Robot stopped and all threads terminated.")

    def __init__(
        self,
        ip: str = "192.168.1.221",
        real: bool = True,
        control_frequency: float = 500.0,
        max_delta: float = DEFAULT_MAX_DELTA,
        model: Optional[str] = None,
    ):
        print(ip)
        self.real = real
        self.max_delta = max_delta
        self.monitoring = False
        self.monitor_thread = None
        self.boundaries = self.DEFAULT_WORKSPACE_LIMITS.copy()
        self.locked_after_emergency = False
        self.initialization_complete = False  # Add flag to track initialization
        self.initialization_event = threading.Event()  # Add event for synchronization

        if real:
            from xarm.wrapper import XArmAPI
            self.robot = XArmAPI(ip, is_radian=True)
            # Verify connection
            if not self._verify_connection():
                raise ConnectionError(f"Failed to connect to robot at {ip}")
        else:
            self.robot = None

        self._control_frequency = control_frequency
        self._clear_error_states()
        self._set_gripper_position(self.GRIPPER_OPEN)
        
        # Setup state variables regardless of initialization
        self.last_state_lock = threading.Lock()
        self.target_command_lock = threading.Lock()
        self.last_state = self._update_last_state()
        self.target_command = {
            "joints": self.last_state.joints(),
            "gripper": 1,
        }
        self.running = True
        self.command_thread = None
        self.joint_history = []
        self.history_size = 3  # Number of past commands to average
        self.recorded_maxmin = {"x_max": 0, "x_min": 0, "y_max": 0, "y_min": 0, "z_max": 0, "z_min": 0}
        
        # Start initialization in a separate thread
        if real:
            init_thread = threading.Thread(target=self._initialize_robot)
            init_thread.daemon = True
            init_thread.start()
            
            # Start emergency stop listener thread
            emergency_thread = threading.Thread(target=self._emergency_stop_listener)
            emergency_thread.daemon = True
            emergency_thread.start()
        else:
            self.initialization_complete = True
            self.initialization_event.set()

    def _initialize_robot(self):
        """Initialize the robot in a separate thread"""
        print("Press 's' then Enter to let robot go home and start the program.")
        print("NOTE: This process must complete before run_env.py can connect to the robot.")
        print("After initialization, you can press 'e' + Enter for emergency stop, or 'q' + Enter to quit.")
        while True:
            user_input = input()
            if user_input.strip().lower() == 's':
                print("Clearing error and resuming...")
                # Clear error and enable motion
                self.robot.clean_error()
                time.sleep(1)
                
                if self.real and self.robot is not None:
                    print("Moving to home position...")
                    print(f"Target initial angles: {self.INITIAL_ANGLES}")
                    self.robot.set_reduced_mode(True)
                    self.robot.set_reduced_max_joint_speed(0.2)  # rad/s
                    self.robot.set_reduced_max_tcp_speed(100)     # mm/s

                    # Initialize ret variable
                    ret = 0
                    
                    while True:
                        current = self.robot.get_servo_angle(is_radian=True)[1]
                        step = [(t - c) * 0.05 for t, c in zip(self.INITIAL_ANGLES, current)]
                        next_pos = [c + s for c, s in zip(current, step)]
                        ret = self.robot.set_servo_angle_j(angles=next_pos, is_radian=True)
                        if ret != 0:
                            print(f"Error in set_servo_angle_j during initialization: {ret}")
                            self._clear_error_states()
                        time.sleep(0.05)  # Lower frequency → slower motion
                        
                        # Check after movement if we've reached the target
                        current = self.robot.get_servo_angle(is_radian=True)[1]
                        if all(abs(t - c) < self.TOLERANCE for t, c in zip(self.INITIAL_ANGLES, current)):
                            print("Target reached.")
                            break

                self.robot.set_reduced_mode(False)
                time.sleep(1)
                
                # Start the command thread now that initialization is complete
                if self.command_thread is None:
                    # Save the current position as the target command to prevent immediate movement
                    code, current_joints = self.robot.get_servo_angle(is_radian=True)
                    if code != 0:
                        print(f"Error getting joint angles: {code}")
                        self._clear_error_states()
                        code, current_joints = self.robot.get_servo_angle(is_radian=True)
                    
                    # Update last_state to match the current position
                    self.last_state = self._update_last_state()
                    
                    # Update target_command with the actual current position
                    with self.target_command_lock:
                        self.target_command = {
                            "joints": np.array(current_joints),
                            "gripper": self.target_command["gripper"],
                        }
                        # Initialize joint history with current position
                        self.joint_history = [np.array(current_joints)] * self.history_size
                        print(f"Target command set to current position: {current_joints}")
                    
                    # Now start the command thread
                    self.command_thread = threading.Thread(target=self._robot_thread)
                    self.command_thread.start()
                
                print("Robot is ready and running.")
                self.initialization_complete = True
                self.initialization_event.set()  # Signal that initialization is complete
                break

    def _verify_connection(self) -> bool:
        """Verify robot connection is working properly"""
        if self.robot is None:
            return False
        
        try:
            # Try to get basic robot information
            version = self.robot.get_version()
            state = self.robot.get_state()
            print(f"Robot connection verified. Version: {version}, State: {state}")
            return True
        except Exception as e:
            print(f"Robot connection verification failed: {e}")
            return False

    def wait_for_initialization(self, timeout=None):
        """Wait for robot initialization to complete
        
        Args:
            timeout: Maximum time to wait in seconds, or None to wait indefinitely
            
        Returns:
            True if initialization completed, False if timed out
        """
        return self.initialization_event.wait(timeout)

    def _emergency_stop_listener(self):
        """Listen for emergency stop command in a separate thread"""
        print("Emergency stop listener started. Press 'e' then Enter at any time to emergency stop the robot.")
        while self.running:
            try:
                # Only listen for emergency stop after initialization is complete
                if not self.initialization_complete:
                    time.sleep(0.1)  # Wait for initialization to complete
                    continue
                
                # Add a prompt to make it clear this is for emergency controls
                print("Robot operational. Commands: 'e' = emergency stop, 'q' = quit")
                user_input = input("Emergency control: ")
                if user_input.strip().lower() == 'e':
                    print("EMERGENCY STOP TRIGGERED!")
                    self.emergency_stop_robot()
                    break  # Exit the listener after emergency stop
                elif user_input.strip().lower() == 'q':
                    print("Shutting down robot...")
                    self.stop()
                    break
                else:
                    print("Invalid command. Use 'e' for emergency stop or 'q' to quit.")
            except (EOFError, KeyboardInterrupt):
                # Handle case where input is interrupted
                break
            except Exception as e:
                print(f"Error in emergency stop listener: {e}")
                time.sleep(0.1)  # Prevent rapid error loops
                
    def emergency_stop_robot(self):
        """Emergency stop the robot using xArm API built-in function"""
        if self.robot is not None:
            try:
                print("Executing emergency stop...")
                # Use xArm API's built-in emergency stop function
                self.robot.emergency_stop()
                print("Emergency stop executed successfully.")
                
                # Set robot to error state
                self.robot.set_state(state=4)  # Error state
                
                # Stop all our threads
                self.running = False
                self.locked_after_emergency = True
                self.monitoring = False  # Stop position monitoring too
                
                print("Robot is now in emergency stop state.")
                print("To recover:")
                print("  1. Press 'c' then Enter to clear errors and restart")
                print("  2. Or restart the program manually")
                print("  3. Or use xArm Studio to clear errors")
                
                # Offer recovery option
                self._emergency_recovery()
                
            except Exception as e:
                print(f"Error during emergency stop: {e}")
        else:
            print("Robot is not connected - cannot execute emergency stop.")
            
    def _emergency_recovery(self):
        """Handle recovery after emergency stop"""
        print("Emergency recovery options:")
        print("  'c' - Clear errors and restart robot")
        print("  'q' - Quit without recovery")
        
        while True:
            try:
                user_input = input("Recovery command: ")
                if user_input.strip().lower() == 'c':
                    print("Attempting to clear errors and restart...")
                    try:
                        # Clear errors with validation
                        print("Clearing errors...")
                        self.robot.clean_error()
                        self.robot.clean_warn()
                        time.sleep(1)
                        
                        # Check if errors are actually cleared
                        error_code = self.robot.get_err_warn_code()
                        if error_code[0] and len(error_code[1]) > 0:
                            print(f"Warning: Still have error codes: {error_code[1]}")
                        
                        # Reset state with validation
                        print("Resetting robot state...")
                        ret = self.robot.set_state(state=0)
                        if ret != 0:
                            print(f"Warning: Error setting state to 0, code: {ret}")
                        time.sleep(0.5)
                        
                        print("Errors cleared. Moving to home position...")
                        
                        # Move to home position (similar to initialization)
                        self.robot.set_reduced_mode(True)
                        self.robot.set_reduced_max_joint_speed(0.2)
                        self.robot.set_reduced_max_tcp_speed(100)
                        
                        ret = 0
                        max_attempts = 100  # Prevent infinite loops
                        attempt = 0
                        
                        while attempt < max_attempts:
                            current = self.robot.get_servo_angle(is_radian=True)[1]
                            step = [(t - c) * 0.05 for t, c in zip(self.INITIAL_ANGLES, current)]
                            next_pos = [c + s for c, s in zip(current, step)]
                            ret = self.robot.set_servo_angle_j(angles=next_pos, is_radian=True)
                            if ret != 0:
                                print(f"Error during recovery movement: {ret}")
                                self._clear_error_states()
                                if ret in [1, 9]:  # Common recoverable errors
                                    attempt += 1
                                    continue
                                else:
                                    break
                            time.sleep(0.05)
                            
                            current = self.robot.get_servo_angle(is_radian=True)[1]
                            if all(abs(t - c) < self.TOLERANCE for t, c in zip(self.INITIAL_ANGLES, current)):
                                print("Recovery complete - robot at home position.")
                                break
                            attempt += 1
                        
                        if attempt >= max_attempts:
                            print("Warning: Recovery movement may not have completed fully")
                        
                        self.robot.set_reduced_mode(False)
                        
                        # Reset flags
                        self.locked_after_emergency = False
                        self.running = True
                        
                        # Restart command thread if needed
                        if self.command_thread is None or not self.command_thread.is_alive():
                            current_joints = self.robot.get_servo_angle(is_radian=True)[1]
                            with self.target_command_lock:
                                self.target_command = {
                                    "joints": np.array(current_joints),
                                    "gripper": self.target_command["gripper"],
                                }
                                self.joint_history = [np.array(current_joints)] * self.history_size
                            
                            self.command_thread = threading.Thread(target=self._robot_thread)
                            self.command_thread.daemon = True
                            self.command_thread.start()
                            
                        print("Robot recovery complete and operational.")
                        break
                        
                    except Exception as e:
                        print(f"Error during recovery: {e}")
                        print("Recovery failed. Please restart the program or use xArm Studio.")
                        break
                        
                elif user_input.strip().lower() == 'q':
                    print("Exiting without recovery...")
                    break
                else:
                    print("Invalid command. Press 'c' to clear errors and recover, or 'q' to quit.")
                    
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"Error in recovery input: {e}")
                break

    def get_state(self) -> RobotState:
        with self.last_state_lock:
            return self.last_state

    def set_command(self, joints: np.ndarray, gripper: Optional[float] = None) -> None:
        if self.locked_after_emergency:
            if not hasattr(self, "_lock_printed") or not self._lock_printed:
                print("Robot is locked after emergency stop. Ignoring command.")
                self._lock_printed = True
            return
        self._lock_printed = False

        with self.target_command_lock:
            # Add current command to history
            self.joint_history.append(joints)
            if len(self.joint_history) > self.history_size:
                self.joint_history.pop(0)
            
            # Average the commands
            smoothed_joints = np.mean(self.joint_history, axis=0)
            
            self.target_command = {
                "joints": smoothed_joints,
                "gripper": gripper,
            }

    def _clear_error_states(self):
        if self.robot is None:
            return
        self.robot.clean_error()
        self.robot.clean_warn()
        self.robot.motion_enable(True)
        time.sleep(0.01)
        self.robot.set_mode(1)
        time.sleep(0.01)
        self.robot.set_collision_sensitivity(0)
        time.sleep(0.01)
        self.robot.set_state(state=0)
        time.sleep(0.01)
        self.robot.set_gripper_enable(True)
        time.sleep(0.01)
        self.robot.set_gripper_mode(0)
        time.sleep(0.01)
        self.robot.set_gripper_speed(10000)
        time.sleep(0.01)
        self.robot.set_joint_jerk(50)  # Default is 100
        time.sleep(0.01)
        self.robot.set_joint_maxacc(3)  # Default is 5
        time.sleep(0.01)

    def _get_gripper_pos(self) -> float:
        if self.robot is None:
            return 0.0
        code, gripper_pos = self.robot.get_gripper_position()
        while code != 0 or gripper_pos is None:
            print(f"Error code {code} in get_gripper_position(). {gripper_pos}")
            time.sleep(0.001)
            code, gripper_pos = self.robot.get_gripper_position()
            if code == 22:
                self._clear_error_states()

        normalized_gripper_pos = 1.0 - (gripper_pos - self.GRIPPER_OPEN) / (
            self.GRIPPER_CLOSE - self.GRIPPER_OPEN
        )
        return normalized_gripper_pos

    def _set_gripper_position(self, pos: int) -> None:
        if self.robot is None:
            return
        self.robot.set_gripper_position(pos, wait=False)
        # while self.robot.get_is_moving():
        #     time.sleep(0.01)

    def _robot_thread(self):
        rate = Rate(
            duration=1 / self._control_frequency
        )  # command and update rate for robot
        step_times = []
        count = 0

        while self.running:
            s_t = time.time()
            # update last state
            self.last_state = self._update_last_state()
            with self.target_command_lock:
                joint_delta = np.array(
                    self.target_command["joints"] - self.last_state.joints()
                )
                gripper_command = self.target_command["gripper"]

            norm = np.linalg.norm(joint_delta)

            # threshold delta to be at most 0.01 in norm space
            if norm > self.max_delta:
                delta = joint_delta / norm * self.max_delta
            else:
                delta = joint_delta

            # command position
            self._set_position(
                self.last_state.joints() + delta,
            )

            if gripper_command is not None:
                set_point = gripper_command
                self._set_gripper_position(
                    self.GRIPPER_OPEN
                    + set_point * (self.GRIPPER_CLOSE - self.GRIPPER_OPEN)
                )
            self.last_state = self._update_last_state()

            rate.sleep()
            step_times.append(time.time() - s_t)
            count += 1
            if count % 1000 == 0:
                # Mean, Std, Min, Max, only show 3 decimal places and string pad with 10 spaces
                frequency = 1 / np.mean(step_times)
                # print(f"Step time - mean: {np.mean(step_times):10.3f}, std: {np.std(step_times):10.3f}, min: {np.min(step_times):10.3f}, max: {np.max(step_times):10.3f}")
                print(
                    f"Low  Level Frequency - mean: {frequency:10.3f}, std: {np.std(frequency):10.3f}, min: {np.min(frequency):10.3f}, max: {np.max(frequency):10.3f}"
                )
                step_times = []

    def _update_last_state(self) -> RobotState:
        with self.last_state_lock:
            if self.robot is None:
                return RobotState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, np.zeros(3))

            gripper_pos = self._get_gripper_pos()

            code, servo_angle = self.robot.get_servo_angle(is_radian=True)
            while code != 0:
                print(f"Error code {code} in get_servo_angle().")
                self._clear_error_states()
                code, servo_angle = self.robot.get_servo_angle(is_radian=True)

            code, cart_pos = self.robot.get_position_aa(is_radian=True)
            while code != 0:
                print(f"Error code {code} in get_position().")
                self._clear_error_states()
                code, cart_pos = self.robot.get_position_aa(is_radian=True)

            cart_pos = np.array(cart_pos)
            aa = cart_pos[3:]
            cart_pos[:3] /= 1000

            return RobotState.from_robot(
                cart_pos,
                servo_angle,
                gripper_pos,
                aa,
            )

    def _set_position(
        self,
        joints: np.ndarray,
    ) -> None:
        if self.robot is None:
            return
        # threhold xyz to be in  min max
        ret = self.robot.set_servo_angle_j(joints, wait=False, is_radian=True)
        if ret in [1, 9]:
            self._clear_error_states()

    def get_observations(self) -> Dict[str, np.ndarray]:
        state = self.get_state()
        pos_quat = np.concatenate([state.cartesian_pos(), state.quat()])
        joints = self.get_joint_state()
        return {
            "joint_positions": joints,  # rotational joint + gripper state
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": np.array(state.gripper_pos()),
        }
    
    def _boundary_violation_recovery(self, check_interval):
        """Handle recovery from boundary violation"""
        while True:
            try:
                user_input = input("Recovery command ('r' to recover): ")
                if user_input.strip().lower() == 'r':
                    print("Clearing error and resuming...")
                    # Clear error and enable motion
                    self.robot.clean_error()
                    time.sleep(1)

                    print("Moving to home position...")
                    print(f"Target initial angles: {self.INITIAL_ANGLES}")
                    self.robot.set_reduced_mode(True)
                    self.robot.set_reduced_max_joint_speed(0.2)  # rad/s
                    self.robot.set_reduced_max_tcp_speed(100)     # mm/s

                    # Initialize ret variable
                    ret = 0
                    max_attempts = 100
                    attempt = 0
                    
                    while attempt < max_attempts:
                        current = self.robot.get_servo_angle(is_radian=True)[1]
                        step = [(t - c) * 0.05 for t, c in zip(self.INITIAL_ANGLES, current)]
                        next_pos = [c + s for c, s in zip(current, step)]
                        ret = self.robot.set_servo_angle_j(angles=next_pos, is_radian=True)
                        if ret != 0:
                            print(f"Error in set_servo_angle_j during recovery: {ret}")
                            self._clear_error_states()
                        time.sleep(0.05)  # Lower frequency → slower motion
                        
                        # Check after movement if we've reached the target
                        current = self.robot.get_servo_angle(is_radian=True)[1]
                        if all(abs(t - c) < self.TOLERANCE for t, c in zip(self.INITIAL_ANGLES, current)):
                            print("Target reached.")
                            break
                        attempt += 1

                    self.robot.set_reduced_mode(False)
                    time.sleep(1)
                    
                    if ret != 0:
                        print(f"Error moving to position, code: {ret}")
                    else:
                        print("Successfully moved to target position")

                    print("Resuming monitoring.")
                    self.locked_after_emergency = False
                    self.joint_history.clear()  # Clear history after emergency stop
                    time.sleep(1)

                    # Get the current joint angles to ensure command thread starts with correct position
                    current_joints = self.robot.get_servo_angle(is_radian=True)[1]
                    with self.target_command_lock:
                        self.target_command = {
                            "joints": np.array(current_joints),
                            "gripper": self.target_command["gripper"],
                        }
                        # Reinitialize joint history with current position
                        self.joint_history = [np.array(current_joints)] * self.history_size
                    
                    # Make sure robot is in the right mode before restarting thread
                    self._clear_error_states()  # Ensure robot is in a clean state
                    
                    # Restart the command thread
                    self.running = True  # Set running flag before starting thread
                    if self.command_thread is None or not self.command_thread.is_alive():
                        self.command_thread = threading.Thread(target=self._robot_thread)
                        self.command_thread.daemon = True
                        self.command_thread.start()
                        print("Command thread restarted")
                    
                    # Restart monitoring with the same boundaries
                    self.start_position_monitoring(
                        x_min=self.boundaries['x_min'], x_max=self.boundaries['x_max'],
                        y_min=self.boundaries['y_min'], y_max=self.boundaries['y_max'],
                        z_min=self.boundaries['z_min'], z_max=self.boundaries['z_max'],
                        check_interval=check_interval
                    )
                    return  # Exit this method after restarting monitoring
                else:
                    print("Invalid command. Enter 'r' to recover from boundary violation.")
            except (EOFError, KeyboardInterrupt):
                break

    def start_position_monitoring(self, x_min=None, x_max=None, y_min=None, y_max=None, 
                                z_min=None, z_max=None, check_interval=0.1):
        """Start continuous position monitoring with custom boundaries"""
        # Stop any existing monitoring
        self.stop_position_monitoring()
        
        # Use provided values or defaults
        self.boundaries = {
            'x_min': x_min if x_min is not None else self.DEFAULT_WORKSPACE_LIMITS['x_min'],
            'x_max': x_max if x_max is not None else self.DEFAULT_WORKSPACE_LIMITS['x_max'],
            'y_min': y_min if y_min is not None else self.DEFAULT_WORKSPACE_LIMITS['y_min'],
            'y_max': y_max if y_max is not None else self.DEFAULT_WORKSPACE_LIMITS['y_max'],
            'z_min': z_min if z_min is not None else self.DEFAULT_WORKSPACE_LIMITS['z_min'],
            'z_max': z_max if z_max is not None else self.DEFAULT_WORKSPACE_LIMITS['z_max'],
        }
        
        def monitor_position():
            consecutive_errors = 0
            max_consecutive_errors = 5
            
            while self.monitoring and self.running:
                try:
                    if self.robot is not None:
                        state = self.get_state()
                        x, y, z = state.cartesian_pos() * 1000  # Convert m to mm
                        
                        print(f"Current position: [{x:.2f}, {y:.2f}, {z:.2f}]")
                        code2, real_angles = self.robot.get_servo_angle(is_real=True)
                        if code2 == 0:
                            print("Real-time joint angles:", real_angles)
                        else:
                            print(f"Warning: Could not get real-time angles, code: {code2}")

                        # Update recorded max/min
                        if x > self.recorded_maxmin['x_max']:
                            self.recorded_maxmin['x_max'] = x
                        if x < self.recorded_maxmin['x_min']:
                            self.recorded_maxmin['x_min'] = x
                        if y > self.recorded_maxmin['y_max']:
                            self.recorded_maxmin['y_max'] = y
                        if y < self.recorded_maxmin['y_min']:
                            self.recorded_maxmin['y_min'] = y
                        if z > self.recorded_maxmin['z_max']:
                            self.recorded_maxmin['z_max'] = z
                        if z < self.recorded_maxmin['z_min']:
                            self.recorded_maxmin['z_min'] = z
                        print(f"Recorded max/min: {self.recorded_maxmin}")

                        # Check boundaries
                        if (x < self.boundaries['x_min'] or x > self.boundaries['x_max'] or
                            y < self.boundaries['y_min'] or y > self.boundaries['y_max'] or
                            z < self.boundaries['z_min'] or z > self.boundaries['z_max']):
                            print(f"EMERGENCY STOP: Position [{x:.2f}, {y:.2f}, {z:.2f}] is outside safe boundaries:")
                            print(f"Range X: {self.boundaries['x_min']} to {self.boundaries['x_max']}    Current X: {x:.2f}")
                            print(f"Range Y: {self.boundaries['y_min']} to {self.boundaries['y_max']}    Current Y: {y:.2f}")
                            print(f"Range Z: {self.boundaries['z_min']} to {self.boundaries['z_max']}    Current Z: {z:.2f}")
                            
                            if self.robot is not None:
                                self.robot.emergency_stop()
                                self.robot.set_state(state=4)
                            self.locked_after_emergency = True
                            self.monitoring = False
                            self.running = False
                            
                            print("Press 'r' then Enter to let robot go home and resume monitoring.")
                            self._boundary_violation_recovery(check_interval)
                            return
                        
                        # Reset error counter on successful iteration
                        consecutive_errors = 0
                        
                except Exception as e:
                    consecutive_errors += 1
                    print(f"Error in position monitoring (attempt {consecutive_errors}): {e}")
                    
                    if consecutive_errors >= max_consecutive_errors:
                        print(f"Too many consecutive errors ({consecutive_errors}). Stopping position monitoring.")
                        self.monitoring = False
                        break
                    
                    time.sleep(0.5)  # Wait before retrying
                    continue

                time.sleep(check_interval)
        
        self.monitoring = True
        self.running = True
        self.monitor_thread = threading.Thread(target=monitor_position)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        print(f"Position monitoring started with boundaries: {self.boundaries}")
        
    def stop_position_monitoring(self):
        """Stop position monitoring"""
        if hasattr(self, 'monitoring') and self.monitoring:
            self.monitoring = False
            if hasattr(self, 'monitor_thread') and self.monitor_thread is not None:
                self.monitor_thread.join(timeout=1.0)
                print("Position monitoring stopped")
                
    def get_position_boundaries(self):
        """Get current position boundaries"""
        if hasattr(self, 'boundaries'):
            return self.boundaries
        return self.DEFAULT_WORKSPACE_LIMITS


def main():
    ip = "192.168.1.221"
    robot = XArmRobot(ip)
    import time
    
    # Start position monitoring with default boundaries
    robot.start_position_monitoring()
    
    # Get current state and position
    state = robot.get_state()
    print(f"Current position: {state.cartesian_pos() * 1000}mm")
    print(f"Current boundaries: {robot.get_position_boundaries()}")
    
    # Allow time to see the output
    time.sleep(2)
    
    # Set custom boundaries
    robot.start_position_monitoring(
        x_min=-308, x_max=962,
        y_min=-615, y_max=610,
        z_min=-75, z_max=570
    )
    
    # Show custom boundaries
    print(f"Updated boundaries: {robot.get_position_boundaries()}")
    
    # Send a small joint movement
    current_joints = robot.get_state().joints()
    robot.set_command(current_joints + np.array([0.01, 0, 0, 0, 0, 0]))
    
    # Wait a bit
    time.sleep(2)
    
    # Temporarily disable monitoring
    robot.stop_position_monitoring()
    print("Monitoring disabled - robot can move anywhere")
    
    # Wait a bit
    time.sleep(2)
    
    # Re-enable with default boundaries
    robot.start_position_monitoring()
    
    # Clean up
    print("Stopping robot")
    robot.stop()
    print("Robot stopped")

if __name__ == "__main__":
    main()
