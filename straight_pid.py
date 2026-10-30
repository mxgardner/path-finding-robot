"""Encoder-based straight driving for Pybricks EV3 MicroPython 2.0."""
from math import pi
from pybricks.tools import StopWatch, wait
import config as cfg


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def straight(left, right, distance):
    """Drive distance in mm, synchronizing relative wheel encoder angles.

    Both motors' positive directions must roll the robot forward. Gains use
    encoder degrees and seconds; run() commands are motor degrees/second.
    Encoder feedback cannot detect wheel slip or unequal wheel diameters.
    """
    kp = getattr(cfg, "STRAIGHT_KP", 2.0)
    ki = getattr(cfg, "STRAIGHT_KI", 0.1)
    kd = getattr(cfg, "STRAIGHT_KD", 0.05)
    interval = getattr(cfg, "STRAIGHT_INTERVAL_MS", 20)
    tolerance = getattr(cfg, "STRAIGHT_TOLERANCE_DEG", 2.0)
    # Common position feedback slows the robot as it approaches the endpoint.
    position_kp = getattr(cfg, "STRAIGHT_POSITION_KP", 4.0)
    settings = (cfg.SPEED, cfg.WHEEL_DIAMETER, interval, tolerance, position_kp)
    if any(not isinstance(v, (int, float)) or not 0 < v < float("inf")
           for v in settings):
        raise ValueError("Straight speed, diameter, timing and tolerance must be positive")
    if any(not isinstance(v, (int, float)) or not 0 <= v < float("inf")
           for v in (kp, ki, kd)):
        raise ValueError("Straight PID gains must be finite and nonnegative")
    if not isinstance(distance, (int, float)) or not abs(distance) < float("inf"):
        raise ValueError("Straight distance must be finite")

    target = distance * 360 / (pi * cfg.WHEEL_DIAMETER)
    timer = StopWatch()
    limit = 3000 + 3000 * abs(target) / cfg.SPEED
    integral = 0.0
    previous_error = 0.0
    previous_time = 0
    settled_since = None
    correction_limit = cfg.SPEED / 2
    try:
        # Relative readings preserve the encoders across turns and segments.
        left_start, right_start = left.angle(), right.angle()
        while True:
            now = timer.time()
            left_travel = left.angle() - left_start
            right_travel = right.angle() - right_start
            left_remaining = target - left_travel
            right_remaining = target - right_travel
            if abs(left_remaining) <= tolerance and abs(right_remaining) <= tolerance:
                # Require both wheels to remain near the target for 100 ms.
                left.hold()
                right.hold()
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= 100:
                    break
                # Holding pauses the PID; discard its stale derivative/integral.
                integral = 0.0
                previous_error = left_travel - right_travel
                previous_time = now
            else:
                settled_since = None
                dt = max(1, now - previous_time) / 1000
                error = left_travel - right_travel
                derivative = (error - previous_error) / dt
                candidate = integral + error * dt
                # Bound the integral contribution and suppress windup when
                # either steering or the motor speed command saturates.
                if ki:
                    candidate = _clamp(candidate, correction_limit / ki)
                else:
                    candidate = 0.0
                base = _clamp(position_kp * (left_remaining + right_remaining) / 2,
                              cfg.SPEED)
                proposed = kp * error + ki * candidate + kd * derivative
                if (abs(proposed) <= correction_limit and
                        abs(base) + abs(proposed) <= cfg.SPEED):
                    integral = candidate
                correction = _clamp(kp * error + ki * integral + kd * derivative,
                                    correction_limit)
                # Left ahead => slow left, speed right. Signed encoder error
                # gives the same correction rule for reverse travel.
                left.run(_clamp(base - correction, cfg.SPEED))
                right.run(_clamp(base + correction, cfg.SPEED))
                previous_error, previous_time = error, now
            if now > limit:
                raise RuntimeError("Straight PID movement timed out")
            wait(interval)
    finally:
        try:
            left.hold()
        finally:
            right.hold()
