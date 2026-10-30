#!/usr/bin/env pybricks-micropython
# Launch this file on the EV3 to run ONE movement test, then stop.
# Use clear floor. For the directions test, lift the drive wheels first.
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch
import config as cfg
from straight_pid import straight

# CHANGE THESE to choose the test and its inputs. Only the selected test's
# settings are used. These distances are millimeters even in meter mode.
TEST = "straight"      # "gyro", "directions", "straight", "turn", or "square"
GYRO_PORT = Port.S4  # Change this if your gyro is connected elsewhere.
DISTANCE = 30000     # straight: robot travel in mm; negative means backward.
ANGLE = 90          # turn: ROBOT rotation in degrees; +left, -right.
SQUARE_SIDE = 2 * cfg.TILE  # square: length of EACH side in mm (currently 610).
SQUARE_TURN = 90    # square: 90 for left turns, -90 for right turns.
WHEEL_ROTATION = 90 # directions: EACH WHEEL'S rotation in degrees; +forward.


# Straight driving shares the PID controller used by navigation. Copy
# config.py and straight_pid.py beside this file; main.py is not launched.
def move_wheels(left, right, left_angle, right_angle):
    # Wheel angles are in degrees. Start both motors before waiting so they
    # move together. Their built-in encoders control the requested rotation.
    left.run_angle(cfg.SPEED, left_angle, wait=False)
    right.run_angle(cfg.SPEED, right_angle, wait=False)
    timer = StopWatch()
    # Three times the estimated movement time, plus three seconds.
    limit = 3000 + 3000 * max(abs(left_angle), abs(right_angle)) / cfg.SPEED
    while not (left.control.done() and right.control.done()):
        if timer.time() > limit:
            raise RuntimeError("Motor movement timed out")
        wait(10)
    wait(100)  # Let the robot settle before its next movement.


def turn(left, right, angle):
    # angle is the robot's turn in degrees: positive left, negative right.
    # Opposite wheel rotations turn the robot around the wheel midpoint.
    wheel_angle = cfg.AXLE_TRACK * angle / cfg.WHEEL_DIAMETER
    move_wheels(left, right, -wheel_angle, wheel_angle)


# In every function, left and right are Motor objects, not numbers or ports.
# main() creates and passes them automatically using the ports in config.py.
def test_directions(left, right, wheel_degrees=90):
    # wheel_degrees: rotation of each wheel, tested one wheel at a time.
    # 90 = quarter wheel revolution; 360 = one full wheel revolution.
    # This is NOT the robot's turning angle. Keep the drive wheels lifted.
    move_wheels(left, right, wheel_degrees, 0)
    wait(1000)
    move_wheels(left, right, 0, wheel_degrees)


def test_straight(left, right, distance_mm):
    # distance_mm: how far the robot should travel, measured in millimeters.
    # Examples: 152.5 = half tile, 305 = one tile, 1000 = one meter,
    # and -1000 = one meter backward.
    # Mark the wheel midpoint before and after, then measure actual travel.
    straight(left, right, distance_mm)


def test_turn(left, right, angle_deg):
    # angle_deg: how far the ROBOT should turn around its wheel midpoint.
    # Examples: 90 = left 90 degrees, -90 = right 90 degrees,
    # 360 = full left rotation, -360 = full right rotation.
    # Mark the starting heading and measure the actual total turn.
    turn(left, right, angle_deg)


def test_square(left, right, side_mm, turn_deg=90):
    # side_mm: distance along EACH of the four sides, in millimeters.
    # turn_deg: the turn after EACH side; use 90 (left) or -90 (right).
    # Example: test_square(left, right, 610, -90) runs a right-turning square.
    # Should return to the starting midpoint and heading. Allow clear space
    # for the whole robot beyond the square's sides.
    for _ in range(4):
        straight(left, right, side_mm)
        turn(left, right, turn_deg)


def test_gyro(brick):
    gyro = GyroSensor(GYRO_PORT)

    # Run the test for 60 seconds
    timer = StopWatch()

    while timer.time() < 60000:
        angle = gyro.angle()

        brick.screen.clear()
        brick.screen.print("GYRO TEST")
        brick.screen.print("Angle:", angle)

        wait(200)


def main():
    if TEST not in ("gyro", "directions", "straight", "turn", "square"):
        raise ValueError("TEST must be gyro, directions, straight, turn, or square")
    if any(not 0 < v < float("inf") for v in
           (cfg.WHEEL_DIAMETER, cfg.AXLE_TRACK, cfg.SPEED, cfg.TILE)):
        raise ValueError("Robot measurements and speed must be positive")
    if cfg.LEFT_PORT == cfg.RIGHT_PORT:
        raise ValueError("The wheel motors must use different ports")
    brick = EV3Brick()
    left = Motor(cfg.LEFT_PORT, positive_direction=cfg.LEFT_DIRECTION)
    right = Motor(cfg.RIGHT_PORT, positive_direction=cfg.RIGHT_DIRECTION)
    try:
        brick.screen.print("Test:", TEST)
        brick.screen.print("Starting in 1 sec")
        wait(1000)
        # Pass the chosen test's inputs. The motor objects are automatic;
        # the numeric inputs come from the settings at the top of this file.
        if TEST == "gyro":
            test_gyro(brick)
        elif TEST == "directions":
            test_directions(left, right, WHEEL_ROTATION)
        elif TEST == "straight":
            test_straight(left, right, DISTANCE)
        elif TEST == "turn":
            test_turn(left, right, ANGLE)
        elif TEST == "square":
            test_square(left, right, SQUARE_SIDE, SQUARE_TURN)
        brick.screen.print("Test finished")
        brick.speaker.beep()
    finally:
        # Stop both motors after the test or an error.
        try:
            left.hold()
        finally:
            right.hold()


# Measure results yourself, then edit config.py and repeat:
# WHEEL_DIAMETER = old_diameter * actual_distance / requested_distance
# AXLE_TRACK = old_track * requested_turn / actual_turn
# Use positive magnitudes in these formulas; calibrate wheel diameter first.
if __name__ == "__main__":
    main()