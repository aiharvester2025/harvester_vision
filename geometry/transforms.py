"""Dependency-free rigid transforms with an explicit robotics convention.

``T_parent_child`` maps coordinates expressed in ``child`` into ``parent``:

    p_parent = R_parent_child @ p_child + t_parent_child

Translations are metres and rotations are right-handed roll, pitch, yaw radians.
This module deliberately has no ROS dependency so the same configuration can be
checked on the Orin now and used by a future ROS 2/RViz integration.
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence, Tuple


class TransformConfigurationError(ValueError):
    """Raised when a frame configuration is unsafe or structurally invalid."""


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]


def _as_vector3(value: Sequence[float], name: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise TransformConfigurationError(f"{name} must contain exactly three numbers")
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise TransformConfigurationError(f"{name} must contain only numbers") from error
    if not all(math.isfinite(component) for component in result):
        raise TransformConfigurationError(f"{name} must contain finite numbers")
    return result  # type: ignore[return-value]


def _matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)  # type: ignore[return-value]


def _matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))  # type: ignore[return-value]


def rotation_from_rpy(rpy_rad: Sequence[float]) -> Matrix3:
    """Return the parent-from-child rotation for intrinsic roll, pitch, yaw.

    The resulting matrix is ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``, suitable for
    a transform definition using the ROS/robotics fixed-axis RPY convention.
    """
    roll, pitch, yaw = _as_vector3(rpy_rad, "rotation_rpy_rad")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


@dataclass(frozen=True)
class Transform:
    """A rigid transform named by its parent and child frames."""

    parent: str
    child: str
    translation_m: Vector3
    rotation: Matrix3

    @classmethod
    def from_definition(cls, definition: dict[str, Any]) -> "Transform":
        identifier = definition.get("id", "unnamed transform")
        parent = definition.get("parent")
        child = definition.get("child")
        if not isinstance(parent, str) or not parent:
            raise TransformConfigurationError(f"{identifier}: parent must be a non-empty frame name")
        if not isinstance(child, str) or not child:
            raise TransformConfigurationError(f"{identifier}: child must be a non-empty frame name")
        if parent == child:
            raise TransformConfigurationError(f"{identifier}: parent and child must differ")
        return cls(
            parent=parent,
            child=child,
            translation_m=_as_vector3(definition.get("translation_m"), f"{identifier}.translation_m"),
            rotation=rotation_from_rpy(definition.get("rotation_rpy_rad")),
        )

    @classmethod
    def identity(cls, frame: str) -> "Transform":
        return cls(frame, frame, (0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))

    def apply(self, point_child: Sequence[float]) -> Vector3:
        """Transform a point from this transform's child frame to its parent."""
        point = _as_vector3(point_child, "point")
        rotated = _matrix_vector(self.rotation, point)
        return tuple(rotated[index] + self.translation_m[index] for index in range(3))  # type: ignore[return-value]

    def inverse(self) -> "Transform":
        """Return ``T_child_parent``."""
        inverse_rotation = _transpose(self.rotation)
        inverse_translation = _matrix_vector(inverse_rotation, tuple(-value for value in self.translation_m))
        return Transform(self.child, self.parent, inverse_translation, inverse_rotation)

    def compose(self, right: "Transform") -> "Transform":
        """Return this transform after ``right``.

        For ``T_a_b.compose(T_b_c)``, the result is ``T_a_c``. Both transform
        names are checked so accidental inverse/order mistakes fail early.
        """
        if self.child != right.parent:
            raise TransformConfigurationError(
                f"cannot compose T_{self.parent}_{self.child} with T_{right.parent}_{right.child}"
            )
        rotated_translation = _matrix_vector(self.rotation, right.translation_m)
        translation = tuple(
            self.translation_m[index] + rotated_translation[index] for index in range(3)
        )
        return Transform(
            self.parent,
            right.child,
            translation,  # type: ignore[arg-type]
            _matrix_multiply(self.rotation, right.rotation),
        )


class FrameGraph:
    """A static transform forest that can look up either direction of any path."""

    def __init__(self, transforms: Iterable[Transform]):
        self.transforms = tuple(transforms)
        self._validate_tree()

    @classmethod
    def from_configuration(cls, path: str | Path) -> "FrameGraph":
        with Path(path).open(encoding="utf-8") as config_file:
            config = json.load(config_file)
        return cls(Transform.from_definition(item) for item in config.get("static_transforms", []))

    def _validate_tree(self) -> None:
        children: set[str] = set()
        parent_of: dict[str, str] = {}
        for transform in self.transforms:
            if transform.child in children:
                raise TransformConfigurationError(f"frame {transform.child!r} has more than one static parent")
            children.add(transform.child)
            parent_of[transform.child] = transform.parent
        for child in parent_of:
            visited: set[str] = set()
            current = child
            while current in parent_of:
                if current in visited:
                    raise TransformConfigurationError(f"static transform cycle includes frame {current!r}")
                visited.add(current)
                current = parent_of[current]

    def lookup(self, target: str, source: str) -> Transform:
        """Return the transform that maps points from ``source`` to ``target``."""
        if target == source:
            return Transform.identity(target)
        adjacency: dict[str, list[tuple[str, Transform]]] = {}
        for transform in self.transforms:
            # child -> parent uses the stored transform; parent -> child uses its inverse.
            adjacency.setdefault(transform.child, []).append((transform.parent, transform))
            adjacency.setdefault(transform.parent, []).append((transform.child, transform.inverse()))

        queue: deque[tuple[str, Transform]] = deque([(source, Transform.identity(source))])
        visited = {source}
        while queue:
            current, current_from_source = queue.popleft()
            for next_frame, next_from_current in adjacency.get(current, []):
                if next_frame in visited:
                    continue
                next_from_source = next_from_current.compose(current_from_source)
                if next_frame == target:
                    return next_from_source
                visited.add(next_frame)
                queue.append((next_frame, next_from_source))
        raise TransformConfigurationError(f"no static transform path from {source!r} to {target!r}")

    def transform_point(self, point: Sequence[float], source: str, target: str) -> Vector3:
        return self.lookup(target, source).apply(point)


def validate_configuration(path: str | Path, mode: str = "planning") -> list[str]:
    """Validate a configuration's safety policy and return human-readable issues.

    ``planning`` permits blank survey values, ``simulation`` permits only
    numeric nominal values, and ``deployment`` requires every static physical
    transform to be marked ``verified``.
    """
    if mode not in {"planning", "simulation", "deployment"}:
        raise TransformConfigurationError("mode must be planning, simulation, or deployment")
    with Path(path).open(encoding="utf-8") as config_file:
        config = json.load(config_file)

    issues: list[str] = []
    if config.get("schema") != "harvester.frames.v1":
        issues.append("unsupported or missing schema (expected harvester.frames.v1)")
    units = config.get("units", {})
    if units.get("length") != "m" or units.get("angle") != "rad":
        issues.append("units must declare metres and radians")
    if mode == "simulation" and config.get("deployment_policy") != "simulation_only":
        issues.append("simulation mode requires a configuration marked deployment_policy=simulation_only")
    if mode == "deployment" and config.get("deployment_policy") == "simulation_only":
        issues.append("a simulation-only configuration must never be used for deployment")

    allowed_statuses = {"convention", "survey_required", "provisional", "verified"}
    numeric_transforms: list[Transform] = []
    seen_ids: set[str] = set()
    declared_frames = set(config.get("frame_definitions", {}))
    for definition in config.get("static_transforms", []):
        identifier = definition.get("id", "unnamed transform")
        if identifier in seen_ids:
            issues.append(f"duplicate transform id {identifier!r}")
        seen_ids.add(identifier)
        status = definition.get("status")
        if status not in allowed_statuses:
            issues.append(f"{identifier}: unsupported status {status!r}")
        declared_frames.update((definition.get("parent"), definition.get("child")))
        has_values = definition.get("translation_m") is not None and definition.get("rotation_rpy_rad") is not None
        if has_values:
            try:
                numeric_transforms.append(Transform.from_definition(definition))
            except TransformConfigurationError as error:
                issues.append(str(error))
        elif mode in {"simulation", "deployment"}:
            issues.append(f"{identifier}: missing measured transform values")
        if mode == "deployment" and status not in {"convention", "verified"}:
            issues.append(f"{identifier}: status {status!r} is not allowed in deployment")

    for definition in config.get("dynamic_transforms", []):
        declared_frames.update((definition.get("parent"), definition.get("child")))
    declared_frames.discard(None)

    for role, binding in config.get("camera_bindings", {}).items():
        frame_id = binding.get("image_frame_id") if isinstance(binding, dict) else None
        if frame_id not in declared_frames:
            issues.append(f"camera binding {role!r} references unknown image frame {frame_id!r}")

    telemetry_keys: set[str] = set()
    sensor_ids: set[str] = set()
    for binding in config.get("sensor_telemetry_bindings", []):
        if not isinstance(binding, dict):
            issues.append("sensor telemetry binding must be an object")
            continue
        telemetry_key = binding.get("telemetry_key")
        sensor_id = binding.get("sensor_id")
        frame_id = binding.get("frame_id")
        if not isinstance(telemetry_key, str) or telemetry_key in telemetry_keys:
            issues.append(f"invalid or duplicate sensor telemetry key {telemetry_key!r}")
        telemetry_keys.add(telemetry_key)
        if not isinstance(sensor_id, str) or sensor_id in sensor_ids:
            issues.append(f"invalid or duplicate sensor id {sensor_id!r}")
        sensor_ids.add(sensor_id)
        if frame_id not in declared_frames:
            issues.append(f"sensor binding {sensor_id!r} references unknown frame {frame_id!r}")

    try:
        FrameGraph(numeric_transforms)
    except TransformConfigurationError as error:
        issues.append(str(error))
    return issues
