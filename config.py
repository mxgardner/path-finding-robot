"""Edit these settings for your robot and the assigned course."""
from pybricks.parameters import Port, Direction

# Motors. Change CLOCKWISE to COUNTERCLOCKWISE if a wheel rolls backward.
LEFT_PORT = Port.B
RIGHT_PORT = Port.C
LEFT_DIRECTION = Direction.CLOCKWISE
RIGHT_DIRECTION = Direction.CLOCKWISE

# Robot measurements in millimeters; speed in motor degrees per second.
WHEEL_DIAMETER = 81.6
AXLE_TRACK = 136.0
SPEED = 50

# Straight PID: encoder error in degrees, time in seconds, output in deg/s.
# Starting gains; tune on the robot using test.py's straight test.
STRAIGHT_KP = 2.0
STRAIGHT_KI = 0.1
STRAIGHT_KD = 0.05
STRAIGHT_INTERVAL_MS = 20
STRAIGHT_TOLERANCE_DEG = 2.0
STRAIGHT_POSITION_KP = 4.0

# Estimated body: about 200 mm long and 161 mm wide.
# Distance from axle midpoint to obstacle edge, including body and extra room.
# With an approximately centered axle, half the body diagonal is 128.4 mm.
# Round up to 140 mm for a margin during in-place turns; measure to confirm.
CLEARANCE = 140.0

# Floor size is always in tiles; tile size is always in millimeters.
TILE = 305
WIDTH, HEIGHT = 16, 5

# Applies to START, GOAL and OBSTACLES only: "tiles" or "meters".
# When using meters, enter ALL of those coordinates in meters.
COORDINATE_UNITS = "tiles"

# Coordinates are measured from the bottom-left corner of the workspace.
# Place the midpoint between the drive wheels at START.
START = (1, 1)
GOAL = (14, 3)
START_HEADING = 0  # 0 = right, 90 = up, 180 = left, -90 = down.

# Obstacle CENTERS in the selected units. Replace with your assigned layout.
OBSTACLES = [
    (8, 3)
]

# Meter examples (use with COORDINATE_UNITS = "meters"):
# START = (0.305, 1.219)
# GOAL = (3.658, 1.829)
# OBSTACLES = [(0.610, 2.743)]  # Replace with all assigned centers in meters.
