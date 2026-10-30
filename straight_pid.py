
"""Simple gyro-based straight driving for EV3."""

from math import pi
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
import config as cfg

brick = EV3Brick()
gyro = GyroSensor(Port.S4)


def straight(left, right, distance):
    # Convert distance in mm to motor degrees
    target = distance * 360 / (pi * cfg.WHEEL_DIAMETER)

    # Remember starting wheel positions
    left_start = left.angle()
    right_start = right.angle()

    # Remember the direction we want to maintain
    target_heading = gyro.angle()

    kp = 0.8
    speed = cfg.SPEED

    # Track gyro errors during the drive
    min_error = 0
    max_error = 0

    timer = StopWatch()
    timeout = 3000 + 3000 * abs(target) / speed

    try:
        while True:
            # Measure distance traveled
            left_moved = left.angle() - left_start
            right_moved = right.angle() - right_start
            traveled = (left_moved + right_moved) / 2

            remaining = target - traveled

            # Read gyro and calculate steering error
            error = gyro.angle() - target_heading

            # Record the smallest and largest errors
            min_error = min(min_error, error)
            max_error = max(max_error, error)

            # Stop when destination is reached
            if abs(remaining) <= 2:
                break

            # Steering correction
            correction = kp * error

            # Slow down near destination
            base = max(-speed * 0.7,
                       min(speed * 0.7, 4 * remaining))

            # Limit correction
            correction = max(-speed / 2,
                             min(speed / 2, correction))

            # Corrected motor speeds
            left_speed = base - correction
            right_speed = base + correction

            left.run(max(-speed, min(speed, left_speed)))
            right.run(max(-speed, min(speed, right_speed)))

            if timer.time() > timeout:
                raise RuntimeError("Straight movement timed out")

            wait(20)

    finally:
        left.hold()
        right.hold()

        left_total = left.angle() - left_start
        right_total = right.angle() - right_start

        # brick.screen.clear()
        # brick.screen.print("Left:", round(left_total))
        # brick.screen.print("Right:", round(right_total))
        # brick.screen.print("Diff:", round(left_total - right_total))
        # brick.screen.print("Gyro:", gyro.angle() - target_heading)

        # wait(15000)
