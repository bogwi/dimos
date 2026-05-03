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

"""Tests for trajectory odom replay loader (921 P5-3)."""

import pytest

from dimos.navigation.trajectory_replay_loader import (
    TRAJECTORY_ODOM_MEMORY2_REPLAY,
    open_trajectory_odom_replay,
)
from dimos.robot.unitree.type.odometry import Odometry
from dimos.utils.data import get_data_dir

_GO2_DB = get_data_dir() / "go2_bigoffice.db"
requires_go2_replay_db = pytest.mark.skipif(
    not _GO2_DB.exists(),
    reason=f"go2_bigoffice memory2 DB missing (expect {_GO2_DB}; git lfs pull data/)",
)


@requires_go2_replay_db
def test_open_fixture_streams_go2_bigoffice_odom() -> None:
    replay = open_trajectory_odom_replay("fixture", autocast=Odometry.from_msg)
    msgs = []
    for ts, odom in replay.iterate_ts():
        msgs.append((ts, odom))
        if len(msgs) >= 3:
            break
    assert len(msgs) == 3
    assert all(isinstance(o, Odometry) for _, o in msgs)
    assert msgs[0][0] < msgs[1][0] < msgs[2][0]


@requires_go2_replay_db
def test_open_explicit_memory2_name_same_as_fixture() -> None:
    r1 = open_trajectory_odom_replay("fixture", autocast=Odometry.from_msg)
    r2 = open_trajectory_odom_replay(TRAJECTORY_ODOM_MEMORY2_REPLAY, autocast=Odometry.from_msg)
    assert r1.first_timestamp() == r2.first_timestamp()


def test_path_source_rejected() -> None:
    with pytest.raises(TypeError, match="not a Path"):
        open_trajectory_odom_replay(_GO2_DB.parent, autocast=Odometry.from_msg)


@pytest.mark.lfs_data
@pytest.mark.skipif(
    not (get_data_dir() / "go2_china_office.db").exists(),
    reason="go2_china_office.db not present",
)
def test_open_alternate_dataset_when_present() -> None:
    """Optional: second shipped replay DB, same stream layout as ``ReplayConnection``."""
    replay = open_trajectory_odom_replay("go2_china_office/odom", autocast=Odometry.from_msg)
    ts, odom = next(replay.iterate_ts())
    assert isinstance(ts, float)
    assert isinstance(odom, Odometry)
