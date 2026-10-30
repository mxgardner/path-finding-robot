
"""Simple gyro-based straight driving for EV3."""

from math import pi
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
import config as cfg

gyro = GyroSensor(Port.S2)


def straight(left, right, distance):
    # Convert distance in mm to motor degrees
    target = distance * 360 / (pi * cfg.WHEEL_DIAMETER)

    # Remember starting wheel positions
    left_start = left.angle()
    right_start = right.angle()

    # Remember the direction we want to maintain
    target_heading = gyro.angle()

    kp = 1.0  # Steering correction strength
    speed = cfg.SPEED

    timer = StopWatch()
    timeout = 3000 + 3000 * abs(target) / speed

    try:
        while True:
            # Measure average distance traveled
            left_moved = left.angle() - left_start
            right_moved = right.angle() - right_start
            traveled = (left_moved + right_moved) / 2

            remaining = target - traveled

            # Stop when we reach our destination
            if abs(remaining) <= 2:
                break

            # Measure how much the robot has turned
            error = gyro.angle() - target_heading

            # Calculate steering correction
            correction = kp * error

            # Slow down near the destination
            base = max(-speed, min(speed, 4 * remaining))

            # Limit the steering correction
            correction = max(-speed / 2,
                             min(speed / 2, correction))

            # Adjust motor speeds
            left_speed = base + correction
            right_speed = base - correction

            left.run(max(-speed, min(speed, left_speed)))
            right.run(max(-speed, min(speed, right_speed)))

            if timer.time() > timeout:
                raise RuntimeError("Straight movement timed out")

            wait(20)

    finally:
        left.hold()
        right.hold()
