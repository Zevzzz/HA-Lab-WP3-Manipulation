"""Utility helpers for motion and demos."""

import time
from typing import Optional

import rclpy

DEFAULT_SLEEP_CHECK_INTERVAL_S = 0.1


def sleep_until_ok(
    duration_sec: float,
    *,
    check_interval_s: float = DEFAULT_SLEEP_CHECK_INTERVAL_S,
) -> None:
    """
    Sleep for duration_sec, but return early if rclpy is shutting down.

    Use this instead of time.sleep() so that Ctrl+C and node shutdown
    are respected during waits.
    """
    elapsed = 0.0
    while elapsed < duration_sec and rclpy.ok():
        remaining = duration_sec - elapsed
        step = min(check_interval_s, remaining)
        time.sleep(step)
        elapsed += step
