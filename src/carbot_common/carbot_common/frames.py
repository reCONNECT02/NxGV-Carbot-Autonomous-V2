"""Coordinate frames.

track      Prior map frame (rulebook drawing, V4 core.js Course). Metres.
venue      UWB anchor frame (origin anchor 1782). Metres. track<->venue comes
           from calibration step 11 (map-to-UWB alignment lap).
odom       Wheel + IMU dead-reckoning frame (base servo_controller /odom).
base_link  Rear-axle centre on the ground, +x forward, +y left (V4 bicycle model).
laser_frame, cam_front: sensor frames (front camera only since 2026-09-24).
"""
TRACK = 'track'
VENUE = 'venue'
ODOM = 'odom'
BASE = 'base_link'
LASER = 'laser_frame'


def camera_frame(role: str) -> str:
    return f'cam_{role}'
