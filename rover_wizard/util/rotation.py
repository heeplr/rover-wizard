"""rover_wizard.util.rotation - board-orientation maths for the AHRS_ORIENTATION auto-probe.

ArduPilot's AHRS_ORIENTATION names a rotation that turns vectors measured in the flight controller's own (sensor)
frame into the vehicle frame (x forward, y right, z down). This module builds those rotation matrices from the
names, and finds which name best explains two accelerometer readings taken in known poses.

The per-axis formulas and the "roll first, then pitch, then yaw" order of compound names were checked against
ArduPilot's AP_Math implementation (e.g. Roll180 negates y and z; Yaw45 maps x -> (x-y)/sqrt2, y -> (x+y)/sqrt2).
"""

from __future__ import annotations

import math
import re
from typing import Optional

# (x, y, z)
Vector = tuple
# three row tuples
Matrix = tuple

# AHRS_ORIENTATION / COMPASS_ORIENT enum: (name, parameter value). The value is the list position.
ORIENTATION_CHOICES = [
    (name, value)
    for value, name in enumerate(
        [
            "None",
            "Yaw45",
            "Yaw90",
            "Yaw135",
            "Yaw180",
            "Yaw225",
            "Yaw270",
            "Yaw315",
            "Roll180",
            "Yaw45Roll180",
            "Yaw90Roll180",
            "Yaw135Roll180",
            "Pitch180",
            "Yaw225Roll180",
            "Yaw270Roll180",
            "Yaw315Roll180",
            "Roll90",
            "Yaw45Roll90",
            "Yaw90Roll90",
            "Yaw135Roll90",
            "Roll270",
            "Yaw45Roll270",
            "Yaw90Roll270",
            "Yaw135Roll270",
            "Pitch90",
            "Pitch270",
        ]
    )
]
ORIENTATION_NAME_BY_VALUE = {value: name for name, value in ORIENTATION_CHOICES}

IDENTITY: Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
# vehicle frame: z points down
VEHICLE_DOWN: Vector = (0.0, 0.0, 1.0)
# vehicle frame: x points forward
VEHICLE_FORWARD: Vector = (1.0, 0.0, 0.0)
_NAME_PART = re.compile(r"(Roll|Pitch|Yaw)(\d+)")


def axis_rotation(axis: str, degrees: float) -> Matrix:
    """Right-handed rotation matrix for `degrees` about the vehicle's x (roll), y (pitch) or z (yaw) axis."""
    radians = math.radians(degrees)
    cos_a = math.cos(radians)
    sin_a = math.sin(radians)
    # roll: y and z rotate, x stays
    if axis == "x":
        return ((1.0, 0.0, 0.0), (0.0, cos_a, -sin_a), (0.0, sin_a, cos_a))
    # pitch: z and x rotate, y stays
    if axis == "y":
        return ((cos_a, 0.0, sin_a), (0.0, 1.0, 0.0), (-sin_a, 0.0, cos_a))
    # yaw: x and y rotate, z stays
    if axis == "z":
        return ((cos_a, -sin_a, 0.0), (sin_a, cos_a, 0.0), (0.0, 0.0, 1.0))
    raise ValueError(f"unknown rotation axis {axis!r}")


def matrix_product(left: Matrix, right: Matrix) -> Matrix:
    """3x3 matrix product left @ right (apply `right` first, then `left`)."""
    # Element (row, col) = dot product of left's row with right's column.
    return tuple(tuple(sum(left[row][k] * right[k][col] for k in range(3)) for col in range(3)) for row in range(3))


def transform(matrix: Matrix, vector: Vector) -> Vector:
    """3x3 matrix times a 3-vector."""
    # Component i = dot product of the matrix's row i with the vector.
    return tuple(sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3))


def transpose(matrix: Matrix) -> Matrix:
    """Transposed matrix - for a rotation matrix this is its inverse (rotates back)."""
    return tuple(zip(*matrix))


def normalize(vector: Vector) -> Optional[Vector]:
    """Unit vector in the same direction, or None if the vector is too short to have a meaningful direction."""
    # Euclidean length = sqrt(x² + y² + z²).
    length = math.sqrt(sum(component * component for component in vector))
    # (almost) zero: no direction to speak of
    if length <= 1e-6:
        return None
    # Dividing every component by the length gives the same direction with length 1.
    return tuple(component / length for component in vector)


def rotation_matrix_for_name(name: str) -> Matrix:
    """Rotation matrix (sensor frame -> vehicle frame) for a named orientation such as 'Yaw45Roll180'."""
    # Parse "Roll<deg>", "Pitch<deg>", "Yaw<deg>" parts; a part that is absent means 0 deg ("None" has none).
    angles = {axis_name: int(degrees) for axis_name, degrees in _NAME_PART.findall(name)}
    matrix = IDENTITY
    # Compose in ArduPilot's order: roll is applied first, then pitch, then yaw (AP_Math/vector3.cpp). Pre-multiplying
    # each new rotation (new @ matrix) means it acts after the ones already in `matrix`. ArduPilot's names list them
    # as in the matrix product ("Yaw45Roll90" = yaw45 @ roll90: roll first) - the order in a name doesn't matter here.
    for axis_name, axis in (("Roll", "x"), ("Pitch", "y"), ("Yaw", "z")):
        if angles.get(axis_name):
            matrix = matrix_product(axis_rotation(axis, angles[axis_name]), matrix)
    return matrix


def gravity_direction_from_accelerometer(acceleration: Vector) -> Vector:
    """Direction of gravity ("down") in the sensor frame, from a stationary accelerometer reading.

    An accelerometer measures specific force, which at rest points *up* (a level ArduPilot board reads z ≈ -1 g).
    Gravity - the direction we want - is therefore the reading negated.
    """
    return tuple(-component for component in acceleration)


def best_matching_rotation(names: list, down_in_sensor_frame: Vector, forward_in_sensor_frame: Vector) -> tuple:
    """Which named rotation best maps the measured 'down' to vehicle-down and 'forward' to vehicle-forward.

    `down_in_sensor_frame` is gravity's direction with the vehicle level; `forward_in_sensor_frame` is gravity's
    direction with the vehicle nose-down (then the vehicle's forward axis points at the ground).
    Returns (name, confidence 0..1); confidence 1.0 means both directions matched perfectly.
    """
    down = normalize(down_in_sensor_frame)
    forward = normalize(forward_in_sensor_frame)
    if down is None or forward is None:
        return None, 0.0
    best_name = None
    best_score = -2.0
    for name in names:
        matrix = rotation_matrix_for_name(name)
        # Rotate both measured directions into the vehicle frame using this candidate.
        rotated_down = transform(matrix, down)
        rotated_forward = transform(matrix, forward)
        # Score = cos(angle to vehicle-down) + cos(angle to vehicle-forward). For unit vectors the cosine with
        # (0,0,1) is just the z component and with (1,0,0) the x component. Perfect match -> 1 + 1 = 2.
        score = rotated_down[2] + rotated_forward[0]
        if score > best_score:
            best_name = name
            best_score = score
    # Map the score range [-2, 2] onto a confidence in [0, 1] (negative scores are hopeless -> 0).
    return best_name, max(0.0, best_score / 2.0)
