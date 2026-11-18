#!/usr/bin/env python

"""Quick non-interactive test: send base velocity commands for a few seconds."""

import time
from lerobot.motors import Motor, MotorNormMode, SerialMotorsBus, MotorCalibration
from lerobot.robots.lekiwi.lekiwi import LeKiwi


class LeKiwiBaseOnly(LeKiwi):
    """LeKiwi with only base motors."""

    def __init__(self, port: str = "/dev/ttyACM0"):
        self.port = port
        self.base_motors = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]
        self.arm_motors = []
        self.bus = SerialMotorsBus(
            port=port,
            motors={
                "base_left_wheel": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_back_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
            },
        )
        self.cameras = {}
        self._is_calibrated = False
        self.calibration = {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def connect(self, calibrate: bool = False):
        self.bus.connect()
        self.bus.disable_torque()
        self.bus.configure_motors()
        self.bus.set_operating_mode("velocity", self.base_motors)
        self.bus.enable_torque()
        self.calibration = {
            name: MotorCalibration(
                id=motor.id, drive_mode=0, homing_offset=0,
                range_min=0, range_max=self.bus.resolution(motor.model) - 1,
            )
            for name, motor in self.bus.motors.items()
        }
        self.bus.write_calibration(self.calibration)
        self._is_calibrated = True
        print(f"Connected to base on {self.port}")

    def disconnect(self):
        self.stop_base()
        self.bus.disconnect(disable_torque=True)
        print("Disconnected")

    def stop_base(self):
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)


def run_test(robot, x, y, theta, duration=2.0):
    """Send a velocity command for `duration` seconds, then stop."""
    print(f"  -> x={x:+.2f} m/s, y={y:+.2f} m/s, theta={theta:+.1f} deg/s  ({duration}s)")
    wheel_vel = robot._body_to_wheel_raw(x, y, theta)
    start = time.time()
    while time.time() - start < duration:
        robot.bus.sync_write("Goal_Velocity", wheel_vel)
        time.sleep(0.05)
    robot.stop_base()
    time.sleep(0.5)


def main():
    robot = LeKiwiBaseOnly(port="/dev/ttyACM0")
    print("Connecting...")
    robot.connect()

    print("\n--- Test: forward ---")
    run_test(robot, x=0.1, y=0.0, theta=0.0, duration=2)

    print("--- Test: backward ---")
    run_test(robot, x=-0.1, y=0.0, theta=0.0, duration=2)

    print("--- Test: left strafe ---")
    run_test(robot, x=0.0, y=0.1, theta=0.0, duration=2)

    print("--- Test: right strafe ---")
    run_test(robot, x=0.0, y=-0.1, theta=0.0, duration=2)

    print("--- Test: rotate left ---")
    run_test(robot, x=0.0, y=0.0, theta=30.0, duration=2)

    print("--- Test: rotate right ---")
    run_test(robot, x=0.0, y=0.0, theta=-30.0, duration=2)

    print("\nDone!")
    robot.disconnect()


if __name__ == "__main__":
    main()
