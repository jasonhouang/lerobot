#!/usr/bin/env python

"""
Test script for LeKiwi base (chassis) control only.
Keyboard controls:
  w/s - forward/backward
  a/d - left/right (strafe)
  z/x - rotate left/right
  r/f - speed up/down
  q   - quit
"""

import time
from pynput import keyboard

from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus
from lerobot.robots.lekiwi.lekiwi import LeKiwi


class LeKiwiBaseOnly(LeKiwi):
    """LeKiwi with only base motors (no arm)."""
    
    def __init__(self, port: str = "/dev/ttyACM0"):
        # Skip parent __init__, create minimal setup
        self.port = port
        self.base_motors = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]
        self.arm_motors = []  # No arm motors
        
        # Create bus with only base motors
        self.bus = SerialMotorsBus(
            port=port,
            motors={
                "base_left_wheel": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_back_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
            },
        )
        
        # No cameras
        self.cameras = {}
        self._is_calibrated = False
    
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected
    
    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated
    
    def connect(self, calibrate: bool = False):
        """Connect to base motors only."""
        self.bus.connect()
        
        # Configure base motors for velocity mode
        self.bus.disable_torque()
        self.bus.configure_motors()
        self.bus.set_operating_mode("velocity", self.base_motors)
        self.bus.enable_torque()
        
        # Wheels don't need calibration (endless rotation)
        from lerobot.motors import MotorCalibration
        self.calibration = {
            name: MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=self.bus.resolution(motor.model) - 1,
            )
            for name, motor in self.bus.motors.items()
        }
        self.bus.write_calibration(self.calibration)
        self._is_calibrated = True
        
        print(f"Connected to base motors on {self.port}")
    
    def disconnect(self):
        """Disconnect from base."""
        self.stop_base()
        self.bus.disconnect(disable_torque=True)
        print("Disconnected")
    
    def stop_base(self):
        """Stop all base motors."""
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)


def main():
    print("Connecting to LeKiwi base...")
    robot = LeKiwiBaseOnly(port="/dev/ttyACM0")
    robot.connect()
    
    # Speed settings (m/s for xy, deg/s for rotation)
    speed_levels = [
        {"xy": 0.05, "theta": 15},   # slow
        {"xy": 0.1, "theta": 30},    # medium
        {"xy": 0.2, "theta": 60},    # fast
    ]
    speed_index = 1  # Start at medium
    
    # Track pressed keys
    pressed_keys = set()
    running = True
    
    def on_press(key):
        try:
            k = key.char
        except AttributeError:
            k = key.name
        pressed_keys.add(k)
    
    def on_release(key):
        try:
            k = key.char
        except AttributeError:
            k = key.name
        pressed_keys.discard(k)
        if k == 'q':
            nonlocal running
            running = False
    
    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    print("\n=== LeKiwi Base Control ===")
    print("w/s: forward/backward")
    print("a/d: left/right (strafe)")
    print("z/x: rotate left/right")
    print("r/f: speed up/down")
    print("q: quit\n")
    
    try:
        while running:
            # Speed control
            if 'r' in pressed_keys:
                speed_index = min(speed_index + 1, 2)
                pressed_keys.discard('r')
                print(f"Speed: {['slow', 'medium', 'fast'][speed_index]}")
                time.sleep(0.2)
            if 'f' in pressed_keys:
                speed_index = max(speed_index - 1, 0)
                pressed_keys.discard('f')
                print(f"Speed: {['slow', 'medium', 'fast'][speed_index]}")
                time.sleep(0.2)
            
            speed = speed_levels[speed_index]
            xy_speed = speed["xy"]
            theta_speed = speed["theta"]
            
            # Compute velocity commands
            x_cmd = 0.0
            y_cmd = 0.0
            theta_cmd = 0.0
            
            if 'w' in pressed_keys:
                x_cmd += xy_speed
            if 's' in pressed_keys:
                x_cmd -= xy_speed
            if 'a' in pressed_keys:
                y_cmd += xy_speed
            if 'd' in pressed_keys:
                y_cmd -= xy_speed
            if 'z' in pressed_keys:
                theta_cmd += theta_speed
            if 'x' in pressed_keys:
                theta_cmd -= theta_speed
            
            # Convert to wheel velocities and send
            base_wheel_goal_vel = robot._body_to_wheel_raw(x_cmd, y_cmd, theta_cmd)
            robot.bus.sync_write("Goal_Velocity", base_wheel_goal_vel)
            
            # Print current command
            if x_cmd != 0 or y_cmd != 0 or theta_cmd != 0:
                print(f"x={x_cmd:+.2f} m/s, y={y_cmd:+.2f} m/s, theta={theta_cmd:+.1f} deg/s", end='\r')
            
            time.sleep(0.05)  # 20 Hz control loop
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        listener.stop()
        robot.stop_base()
        robot.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
