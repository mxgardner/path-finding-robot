#!/usr/bin/env pybricks-micropython
# Launch this file on the EV3 to run ONE movement test.
# Use clear floor. For the directions test, lift the wheels.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
import config as cfg

# -----------------------------------
# TEST SETTINGS
# -----------------------------------

TEST = "gyro_turn"  # Options: "gyro", "gyro_turn", "directions",
# Options: "gyro", "gyro_turn", "directions",
#          "straight", "turn", "square"

GYRO_PORT = Port.S4

DISTANCE = 3 * cfg.TILE
ANGLE = 90
SQUARE_SIDE = 2 * cfg.TILE
SQUARE_TURN = 90
WHEEL_ROTATION = 90


# -----------------------------------
# MOTOR MOVEMENT
# -----------------------------------

def move_wheels(left, right, left_angle, right_angle):
    left.run_angle(cfg.SPEED, left_angle, wait=False)
    right.run_angle(cfg.SPEED, right_angle, wait=False)

    timer = StopWatch()
    limit = 3000 + 3000 * max(
        abs(left_angle), abs(right_angle)
    ) / cfg.SPEED

    while not (left.control.done() and right.control.done()):
        if timer.time() > limit:
            raise RuntimeError("Motor movement timed out")
        wait(10)

    wait(100)


def turn(left, right, angle):
    # Positive angle = left turn
    wheel_angle = cfg.AXLE_TRACK * angle / cfg.WHEEL_DIAMETER
    move_wheels(left, right, -wheel_angle, wheel_angle)


# -----------------------------------
# EXISTING MOVEMENT TESTS
# -----------------------------------

def test_directions(left, right, wheel_degrees=90):
    # Test each wheel separately.
    # Keep wheels lifted during this test.
    move_wheels(left, right, wheel_degrees, 0)
    wait(1000)
    move_wheels(left, right, 0, wheel_degrees)


def test_straight(left, right, distance_mm):
    # Import only when straight driving is selected.
    from straight_pid import straight
    straight(left, right, distance_mm)


def test_turn(left, right, angle_deg):
    turn(left, right, angle_deg)


def test_square(left, right, side_mm, turn_deg=90):
    from straight_pid import straight

    for _ in range(4):
        straight(left, right, side_mm)
        turn(left, right, turn_deg)


# -----------------------------------
# GYRO TEST
# -----------------------------------

def test_gyro(brick):
    # Display gyro readings for 60 seconds.
    # No motor movement.
    gyro = GyroSensor(GYRO_PORT)
    timer = StopWatch()

    while timer.time() < 60000:
        brick.screen.clear()
        brick.screen.print("GYRO TEST")
        brick.screen.print("Angle:", gyro.angle())
        wait(200)


# -----------------------------------
# NEW: GYRO TURN TEST
# -----------------------------------

def test_gyro_turn(left, right, brick):
    # Measure the actual rotation during a motor-powered turn.
    gyro = GyroSensor(GYRO_PORT)

    # Keep robot stationary during initialization.
    wait(2000)
    gyro.reset_angle(0)

    brick.screen.clear()
    brick.screen.print("GYRO TURN")
    brick.screen.print("Before:", gyro.angle())
    wait(1000)

    # Command the robot to turn.
    turn(left, right, ANGLE)

    # Display the measured angle.
    brick.screen.clear()
    brick.screen.print("Requested:", ANGLE)
    brick.screen.print("Gyro:", gyro.angle())
    brick.screen.print("Left=- / Right=+")

    # Keep result visible for 10 seconds.
    wait(10000)


# -----------------------------------
# MAIN
# -----------------------------------

def main():
    valid_tests = (
        "gyro", "gyro_turn", "directions",
        "straight", "turn", "square"
    )

    if TEST not in valid_tests:
        raise ValueError("Invalid TEST setting")

    if any(not 0 < v < float("inf") for v in
           (cfg.WHEEL_DIAMETER, cfg.AXLE_TRACK,
            cfg.SPEED, cfg.TILE)):
        raise ValueError("Robot measurements and speed must be positive")

    if cfg.LEFT_PORT == cfg.RIGHT_PORT:
        raise ValueError("The wheel motors must use different ports")

    brick = EV3Brick()

    left = Motor(
        cfg.LEFT_PORT,
        positive_direction=cfg.LEFT_DIRECTION
    )

    right = Motor(
        cfg.RIGHT_PORT,
        positive_direction=cfg.RIGHT_DIRECTION
    )

    try:
        brick.screen.clear()
        brick.screen.print("Test:", TEST)
        brick.screen.print("Starting in 1 sec")
        wait(1000)

        if TEST == "gyro":
            test_gyro(brick)

        elif TEST == "gyro_turn":
            test_gyro_turn(left, right, brick)

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
        try:
            left.hold()
        finally:
            right.hold()


# Calibration:
# WHEEL_DIAMETER = old_diameter * actual_distance / requested_distance
# AXLE_TRACK = old_track * requested_turn / actual_turn

if __name__ == "__main__":
    main()