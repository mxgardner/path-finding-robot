# Straight driving PID

Copy `straight_pid.py`, `main.py`, `test.py`, and `config.py` to the EV3 together.
The configuration uses ports B/C, 81.6 mm wheels, a 136 mm axle track, and a
speed of 50 motor degrees/sec (about 35.6 mm/sec). A 500 mm straight test takes
about 14 seconds plus settling time.
The code targets the Pybricks EV3 MicroPython 2.0 API used by the original files.

Both navigation and the movement test now use the same straight controller.
Turns continue to use the original encoder angle commands.

The straight controller reads each wheel's rotation relative to the start of
the segment. Its error is left rotation minus right rotation. A positive
correction slows the left wheel and speeds the right wheel. The PID uses elapsed
seconds for its integral and derivative, limits correction and motor speed,
and bounds the integral to reduce windup. Separate position feedback slows both
wheels near the requested distance. Both wheels must stay within the endpoint
tolerance for 100 ms; completion and runtime errors hold both motors. A timeout
stops a movement that cannot finish.

## Test and tune on the robot

1. Verify that positive rotation makes each wheel roll forward. Use
   `TEST = "directions"` with the wheels lifted if needed.
2. Set `TEST = "straight"` (the default) in `test.py`. It drives 500 mm after a
   one-second delay. Use clear floor and a moderate `SPEED` (motor degrees/sec).
3. Mark the starting midpoint and heading, then measure the distance and drift.
   Repeat with `DISTANCE = -500` to check reverse travel.
4. To tune, edit these settings in `config.py`. The current values are
   starting values, not gains calibrated on this robot:

   ```python
   STRAIGHT_KP = 2.0
   STRAIGHT_KI = 0.1
   STRAIGHT_KD = 0.05
   STRAIGHT_INTERVAL_MS = 20
   STRAIGHT_TOLERANCE_DEG = 2.0  # Each wheel's encoder endpoint tolerance.
   STRAIGHT_POSITION_KP = 4.0   # Common distance control, in 1/seconds.
   ```

   Begin tuning with KI and KD at zero. Increase KP gradually until wheel
   synchronization improves; reduce it if steering oscillates. Add a small KD
   if needed to damp oscillation, then a small KI only for persistent encoder
   mismatch. Change one setting at a time. Excessive KD can amplify encoder
   quantization. Reduce POSITION_KP if the robot overshoots the endpoint.

`CLEARANCE` is in millimeters regardless of `COORDINATE_UNITS`. It is set to
140 mm to cover the stated 200 x 161 mm body with a small margin while turning,
assuming the axle midpoint is near the body center. Measure the farthest body
point from the axle midpoint and allow extra room if that assumption is wrong.
The map remains 16 x 5 tiles, starting at (1, 1), ending at (14, 3), with an
obstacle centered at (8, 3).

Encoder synchronization assumes equal effective wheel diameters and no slip.
It cannot measure the robot's actual heading or correct an inaccurate starting
heading. Persistent floor drift with matching encoders requires mechanical
calibration or heading feedback such as a gyro.

## Desktop checks

Run `python3 -m unittest discover -s tests -v`. The tests simulate unequal motor
response and speed-servo lag, reverse travel, a stalled motor, encoder failure,
zero distance, and invalid settings. They do not replace testing on the EV3.
