# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Golden replay regression (921 P6-3 / T-20).

Uses the default memory2 odom stream (``go2_bigoffice/odom``). For each
replay timestamp, measurement pose comes from recorded odom; reference is a
fixed origin pose and zero body twist. Expected ``cmd_vel`` components are
pinned to the analytic holonomic tracking law at
``k_position_per_s=1``, ``k_yaw_per_s=0.75``.

Requires ``data/go2_bigoffice.db`` (Git LFS). Skipped when the file is absent.
"""

from __future__ import annotations

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.trajectory_metrics import (
    planar_position_divergence,
    pose_errors_vs_reference,
)
from dimos.navigation.trajectory_replay_loader import open_trajectory_odom_replay
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
from dimos.robot.unitree.type.odometry import Odometry
from dimos.utils.data import get_data_dir

_GO2_DB = get_data_dir() / "go2_bigoffice.db"
requires_go2_replay_db = pytest.mark.skipif(
    not _GO2_DB.exists(),
    reason=f"go2_bigoffice memory2 DB missing (expect {_GO2_DB}; git lfs pull data/)",
)

# Pinned expected body-frame cmd at first three go2_bigoffice odom steps (origin ref, kp=1, ky=0.75).
_GOLDEN_CMD_LINEAR_X = (5.037694684170128, 5.038748018952001, 5.038820423351832)
_GOLDEN_CMD_LINEAR_Y = (17.93040535198446, 17.930060677347285, 17.93001898088994)
_GOLDEN_CMD_ANGULAR_Z = (1.5805450005188684, 1.5805012267602665, 1.5804979008736197)
# ``pose_errors_vs_reference`` then ``hypot(e_at, e_ct)`` at origin reference, yaw 0.
_GOLDEN_PLANAR_DIV_M = (
    18.6246558039978,
    18.624608921849823,
    18.62458836897774,
)


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


@requires_go2_replay_db
def test_mini_odom_replay_golden_cmd_vel_vs_origin_reference() -> None:
    replay = open_trajectory_odom_replay("fixture", autocast=Odometry.from_msg)
    ctrl = HolonomicTrackingController(k_position_per_s=1.0, k_yaw_per_s=0.75)
    zero_twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    ref_pose = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    t0: float | None = None
    for i, (ts_wall, odom) in enumerate(replay.iterate_ts()):
        if i >= 3:
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
        assert cmd.linear.x == pytest.approx(_GOLDEN_CMD_LINEAR_X[i], rel=0.0, abs=1e-9)
        assert cmd.linear.y == pytest.approx(_GOLDEN_CMD_LINEAR_Y[i], rel=0.0, abs=1e-9)
        assert cmd.angular.z == pytest.approx(_GOLDEN_CMD_ANGULAR_Z[i], rel=0.0, abs=1e-9)
        pr = ref.pose_plan
        pm = meas.pose_plan
        e_at, e_ct, _ = pose_errors_vs_reference(
            float(pm.position.x),
            float(pm.position.y),
            float(pm.orientation.euler.z),
            float(pr.position.x),
            float(pr.position.y),
            float(pr.orientation.euler.z),
        )
        div = planar_position_divergence(e_at, e_ct)
        assert div == pytest.approx(_GOLDEN_PLANAR_DIV_M[i], rel=0.0, abs=1e-9)
