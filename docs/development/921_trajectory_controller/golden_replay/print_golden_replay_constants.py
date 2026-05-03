#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Print golden tuples for ``test_trajectory_golden_replay.py`` (921 P6-3 / T-20).

Must match the logic in that test: ``open_trajectory_odom_replay("fixture", ...)``,
``HolonomicTrackingController``, fixed origin reference pose, zero measured twist,
first ``--steps`` observations from the replay iterator.

Run from dimos repo root:

    uv run python docs/development/921_trajectory_controller/golden_replay/print_golden_replay_constants.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Repo root (directory containing ``pyproject.toml`` and package ``dimos/``).
_ROOT = Path(__file__).resolve().parents[4]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.trajectory_metrics import planar_position_divergence, pose_errors_vs_reference
from dimos.navigation.trajectory_replay_loader import open_trajectory_odom_replay
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
from dimos.robot.unitree.type.odometry import Odometry


def _pose_from_odom(odom: Odometry) -> Pose:
    return Pose(
        float(odom.position.x),
        float(odom.position.y),
        float(odom.position.z),
        float(odom.orientation.x),
        float(odom.orientation.y),
        float(odom.orientation.z),
        float(odom.orientation.w),
    )


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--steps", type=int, default=3, help="number of replay frames to emit")
    p.add_argument("--k-position", type=float, default=1.0, dest="k_position")
    p.add_argument("--k-yaw", type=float, default=0.75, dest="k_yaw")
    args = p.parse_args()

    replay = open_trajectory_odom_replay("fixture", autocast=Odometry.from_msg)
    ctrl = HolonomicTrackingController(k_position_per_s=args.k_position, k_yaw_per_s=args.k_yaw)
    zero_twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    ref_pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

    lx: list[float] = []
    ly: list[float] = []
    lz: list[float] = []
    divs: list[float] = []

    t0: float | None = None
    for i, (ts_wall, odom) in enumerate(replay.iterate_ts()):
        if i >= args.steps:
            break
        if t0 is None:
            t0 = ts_wall
        t_rel = float(ts_wall - t0)
        meas = TrajectoryMeasuredSample(
            time_s=t_rel,
            pose_plan=_pose_from_odom(odom),
            twist_body=zero_twist,
        )
        ref = TrajectoryReferenceSample(time_s=t_rel, pose_plan=ref_pose, twist_body=zero_twist)
        cmd = ctrl.control(ref, meas)
        lx.append(float(cmd.linear.x))
        ly.append(float(cmd.linear.y))
        lz.append(float(cmd.angular.z))
        pm = meas.pose_plan
        pr = ref.pose_plan
        e_at, e_ct, _ = pose_errors_vs_reference(
            float(pm.position.x),
            float(pm.position.y),
            float(pm.orientation.euler.z),
            float(pr.position.x),
            float(pr.position.y),
            float(pr.orientation.euler.z),
        )
        divs.append(float(planar_position_divergence(e_at, e_ct)))

    print("# Paste into test_trajectory_golden_replay.py")
    print(f"_GOLDEN_CMD_LINEAR_X = {tuple(lx)!r}")
    print(f"_GOLDEN_CMD_LINEAR_Y = {tuple(ly)!r}")
    print(f"_GOLDEN_CMD_ANGULAR_Z = {tuple(lz)!r}")
    print("_GOLDEN_PLANAR_DIV_M = (")
    for d in divs:
        print(f"    {d!r},")
    print(")")


if __name__ == "__main__":
    main()
