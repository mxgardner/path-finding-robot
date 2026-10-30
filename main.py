#!/usr/bin/env pybricks-micropython
# Project 1
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Button
from pybricks.tools import wait, StopWatch
# Robot measurements and the course layout are edited in config.py.
import config as cfg
from straight_pid import straight

# START, GOAL and OBSTACLES may use tiles or meters. All distance calculations
# below use millimeters, so they can be compared with the robot's clearance.
if cfg.COORDINATE_UNITS not in ("tiles", "meters"):
    raise ValueError('COORDINATE_UNITS must be "tiles" or "meters"')
# One input unit equals 305 mm in tile mode, or 1000 mm in meter mode.
MM_PER_UNIT = cfg.TILE if cfg.COORDINATE_UNITS == "tiles" else 1000
# A search step is 152.5 mm with the project tiles. The physical floor is
# sized by WIDTH and HEIGHT; this gives the robot more possible path positions.
STEP = cfg.TILE / 2  # Half-tile step in millimeters for movement.


def validate_config():
    # Check settings before planning or creating motor objects.
    # Reject zero, negative, infinite and nonnumeric measurements or speed.
    values = (cfg.WHEEL_DIAMETER, cfg.AXLE_TRACK, cfg.SPEED,
              cfg.CLEARANCE, cfg.TILE, cfg.WIDTH, cfg.HEIGHT)
    if any(not isinstance(v, (int, float)) or not 0 < v < float("inf") for v in values):
        raise ValueError("Robot dimensions, speed, clearance and floor size must be positive numbers")
    # The two wheel motors must be controlled through separate ports.
    if cfg.LEFT_PORT == cfg.RIGHT_PORT:
        raise ValueError("Left and right motors must use different ports")
    # Every position must be a pair of usable numbers: (x, y).
    for position in [cfg.START, cfg.GOAL] + list(cfg.OBSTACLES):
        if (not isinstance(position, (tuple, list)) or len(position) != 2 or
                any(not isinstance(v, (int, float)) or not abs(v) < float("inf") for v in position)):
            raise ValueError("Each position must contain two finite numbers: (x, y)")
    # Each obstacle is one tile wide, centered on its supplied position.
    # Check the whole square fits on the floor, not just its center.
    half = cfg.TILE / 2
    for ox, oy in cfg.OBSTACLES:
        if not (half <= ox * MM_PER_UNIT <= cfg.WIDTH * cfg.TILE - half and
                half <= oy * MM_PER_UNIT <= cfg.HEIGHT * cfg.TILE - half):
            raise ValueError("Obstacle must fit inside the floor; check coordinates and COORDINATE_UNITS")


def free(point):
    # A grid point represents the midpoint BETWEEN the drive wheels.
    # Convert its half-tile indices to millimeters: (2, 8) -> (305, 1220).
    x, y = point[0] * STEP, point[1] * STEP
    # Keep the robot's reference point far enough from every floor boundary
    # to allow space for the body and the configured margin.
    if not (cfg.CLEARANCE < x < cfg.WIDTH * cfg.TILE - cfg.CLEARANCE and
            cfg.CLEARANCE < y < cfg.HEIGHT * cfg.TILE - cfg.CLEARANCE):
        return False
    for ox, oy in cfg.OBSTACLES:
        # Convert the obstacle center to millimeters using the selected units.
        # The square extends half a tile from its center; expand it further
        # by CLEARANCE so the robot's body has room, including during turns.
        # Being inside BOTH expanded coordinate ranges means this point is
        # blocked. This conservative model can reject some feasible positions.
        if (abs(x - ox * MM_PER_UNIT) <= cfg.TILE / 2 + cfg.CLEARANCE and
                abs(y - oy * MM_PER_UNIT) <= cfg.TILE / 2 + cfg.CLEARANCE):
            return False
    # This point passed the floor-boundary and obstacle checks.
    return True


def grid_point(position):
    # Convert START or GOAL from input units into integer half-tile indices.
    # Example in tile mode: (1, 4) -> (2, 8).
    # round() handles the small coordinate rounding in the PDF's meter values.
    point = tuple(int(round(value * MM_PER_UNIT / STEP)) for value in position)
    # Allow the few millimeters of rounding in the PDF's intersection coordinates.
    if any(abs(position[i] * MM_PER_UNIT - point[i] * STEP) > 5 for i in (0, 1)):
        raise ValueError("START and GOAL must match the grid; check COORDINATE_UNITS")
    return point


def find_path():
    # Breadth-first search (BFS): explore nearby positions before farther ones.
    # It finds the fewest grid steps in this model; it does not minimize turns.
    validate_config()
    start = grid_point(cfg.START)
    goal = grid_point(cfg.GOAL)
    # A route cannot start or finish inside a forbidden area in this model.
    if not free(start) or not free(goal):
        raise ValueError("Start or goal blocked: check map and clearance")
    # queue holds positions waiting to be explored. parent records where each
    # position was reached from, and also lets us recognize visited positions.
    queue = [start]
    parent = {start: None}
    for x, y in queue:
        # No need to search farther once the goal is reached.
        if (x, y) == goal:
            break
        # Try one half-tile step right, up, left and down. No diagonal moves.
        for next_point in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            if next_point not in parent and free(next_point):
                parent[next_point] = (x, y)
                # Appended positions are visited later by this same loop.
                queue.append(next_point)
    if goal not in parent:
        raise ValueError("No path: check map and clearance")
    # Reconstruct the route backward: goal -> its parent -> ... -> start.
    path = []
    point = goal
    while point is not None:
        path.append(point)
        point = parent[point]
    # Reverse it so the robot receives positions from start to goal.
    return path[::-1]


def move_wheels(left, right, left_angle, right_angle):
    # These angles are wheel/motor rotations, not the robot's turning angle.
    # The motor controllers use their built-in encoders to reach the targets.
    # wait=False starts each motor without waiting for it to finish, allowing
    # the two wheels to move together instead of one after the other.
    left.run_angle(cfg.SPEED, left_angle, wait=False)
    right.run_angle(cfg.SPEED, right_angle, wait=False)
    # Allow three times the estimated travel time, plus three extra seconds.
    # A stuck or unfinished movement will raise an error instead of waiting forever.
    timer = StopWatch()
    limit = 3000 + 3000 * max(abs(left_angle), abs(right_angle)) / cfg.SPEED
    # Continue only when BOTH motors have finished their requested rotations.
    while not (left.control.done() and right.control.done()):
        if timer.time() > limit:
            raise RuntimeError("Motor movement timed out")
        wait(10)
    # Brief settling pause before the next move. run_angle holds the target
    # position by default when it completes.
    wait(100)


def turn(left, right, angle):
    # angle is the ROBOT's turn in degrees. Each wheel follows a circle with
    # radius AXLE_TRACK / 2, giving the wheel-rotation formula below.
    wheel_angle = cfg.AXLE_TRACK * angle / cfg.WHEEL_DIAMETER
    # Opposite rotations turn around the wheel midpoint. Positive angle turns
    # left (counterclockwise): left wheel backward, right wheel forward.
    move_wheels(left, right, -wheel_angle, wheel_angle)


def main():
    # Plan first. Invalid settings or a blocked route stop the program before
    # any motor movement is requested.
    path = find_path()
    print("Path in half-tile steps:", path)
    # Connect to the brick and the wheel motors using the configured ports.
    # Each motor's positive direction must make its wheel roll forward.
    brick = EV3Brick()
    left = Motor(cfg.LEFT_PORT, positive_direction=cfg.LEFT_DIRECTION)
    right = Motor(cfg.RIGHT_PORT, positive_direction=cfg.RIGHT_DIRECTION)
    brick.screen.print("CENTER to start")
    try:
        # Wait for a fresh CENTER press and release. Releasing an already-held
        # launch button first avoids starting the robot accidentally.
        while Button.CENTER in brick.buttons.pressed():
            wait(20)
        while Button.CENTER not in brick.buttons.pressed():
            wait(20)
        while Button.CENTER in brick.buttons.pressed():
            wait(20)
        # heading is the intended orientation, not a gyro measurement.
        # Physically place the wheel midpoint at START, facing START_HEADING.
        # distance accumulates straight travel until the next corner.
        heading, distance = cfg.START_HEADING, 0
        # Map a change in grid position to a heading: right, up, left or down.
        headings = {(1, 0): 0, (0, 1): 90, (-1, 0): 180, (0, -1): -90}
        # Examine adjacent route positions: first->second, second->third, etc.
        for a, b in zip(path, path[1:]):
            next_heading = headings[(b[0] - a[0], b[1] - a[1])]
            # Express the required turn between -180 and +180 degrees so the
            # robot takes the shorter turn rather than rotating the long way.
            angle = (next_heading - heading + 180) % 360 - 180
            if angle:
                # At a corner, finish the PREVIOUS straight section first.
                if distance:
                    straight(left, right, distance)
                # Turn to face the new section, then begin its distance count.
                turn(left, right, angle)
                heading, distance = next_heading, 0
            # Consecutive steps in the same direction become one longer move.
            # Four half-tile steps, for example, become one 610 mm straight move.
            distance += STEP
        if distance:
            # The last section has no following corner to trigger its movement.
            straight(left, right, distance)
        # This confirms the commands finished. Without an external position
        # sensor, it does not independently confirm the actual floor position.
        brick.screen.print("Route finished")
        brick.speaker.beep()
    finally:
        # Hold both motors after completion or an error. The nested finally
        # still attempts the right motor's hold if the left hold raises an error.
        try:
            left.hold()
        finally:
            right.hold()


if __name__ == "__main__":
    # Start navigation when this file is launched, rather than merely imported.
    main()
