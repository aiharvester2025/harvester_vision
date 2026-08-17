#!/usr/bin/env python3
"""Validate and exercise the harvester frame configuration before ROS integration."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from geometry.transforms import FrameGraph, TransformConfigurationError, validate_configuration


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "calibration" / "frames.deployment.template.json")
    parser.add_argument("--mode", choices=("planning", "simulation", "deployment"), default="planning")
    parser.add_argument("--source", help="Source frame for a point lookup")
    parser.add_argument("--target", help="Target frame for a point lookup")
    parser.add_argument("--point", nargs=3, type=float, metavar=("X", "Y", "Z"), help="Point coordinates in --source")
    args = parser.parse_args()
    if any(value is not None for value in (args.source, args.target, args.point)) and not all(
        value is not None for value in (args.source, args.target, args.point)
    ):
        parser.error("--source, --target, and --point must be supplied together")

    issues = validate_configuration(args.config, args.mode)
    if issues:
        raise SystemExit("INVALID FRAME SETUP:\n- " + "\n- ".join(issues))
    print(f"PASS: {args.config} is valid for {args.mode} use")

    if args.point is not None:
        try:
            graph = FrameGraph.from_configuration(args.config)
            result = graph.transform_point(args.point, args.source, args.target)
        except TransformConfigurationError as error:
            raise SystemExit(f"LOOKUP FAILED: {error}") from error
        print(f"{args.source} point {tuple(args.point)} -> {args.target} point {result}")


if __name__ == "__main__":
    main()
