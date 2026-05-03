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

"""Load Unitree-style timed odom replays for calibration and regression (921 P5-3).

Reads from memory2 SQLite via :class:`~dimos.utils.testing.replay.TimedSensorReplay`
(``<dataset>.db`` under :func:`dimos.utils.data.get_data_dir`, resolved the same
way as :class:`~dimos.robot.unitree.go2.connection.ReplayConnection`).

The default ``source="fixture"`` resolves to the published ``go2_bigoffice/odom``
stream (``get_data("go2_bigoffice.db")``), matching the project replay direction.
See ``docs/development/large_file_management.md`` for LFS and data layout.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any, Literal

from dimos.robot.unitree.type.odometry import Odometry
from dimos.utils.testing.replay import TimedSensorReplay

# Default memory2 odom stream (``ReplayConnection`` uses the same ``dataset/odom`` shape).
TRAJECTORY_ODOM_MEMORY2_REPLAY = "go2_bigoffice/odom"


def _normalize_odom_autocast(
    autocast: Callable[[Any], Any] | None,
) -> Callable[[Any], Any] | None:
    """Apply ``autocast`` for dict-style pickles; no-op if data is already ``Odometry``."""
    if autocast is None:
        return None

    def _wrapped(x: Any) -> Any:
        if isinstance(x, Odometry):
            return x
        return autocast(x)

    return _wrapped


def trajectory_odom_replay_mini_fixture_dir() -> Path:
    """Absolute path to the legacy in-tree pickle mini replay (optional; not used by memory2)."""
    return Path(__file__).resolve().parent / "fixtures" / "trajectory_odom_replay_mini"


def open_trajectory_odom_replay(
    source: Literal["fixture"] | str | Path,
    *,
    autocast: Callable[[Any], Any] | None = None,
) -> TimedSensorReplay:
    """Open a timed odom replay via memory2 (SQLite streams).

    Args:
        source: ``"fixture"`` loads ``TRAJECTORY_ODOM_MEMORY2_REPLAY`` (``go2_bigoffice``
            odom). Otherwise pass ``"<dataset>/<stream>"`` as for ``TimedSensorReplay``
            (for example ``"go2_china_office/odom"``).
            Path-based pickle directories are not supported; pass a string instead.
        autocast: Optional transform when payloads are raw dicts; decoded ``Odometry``
            from memory2 is passed through unchanged.

    Returns:
        ``TimedSensorReplay`` reading the requested memory2 stream.
    """
    wrap = _normalize_odom_autocast(autocast)
    if source == "fixture":
        return TimedSensorReplay(TRAJECTORY_ODOM_MEMORY2_REPLAY, autocast=wrap)
    if isinstance(source, Path):
        raise TypeError(
            "memory2 odom replay expects source='fixture' or a '<dataset>/<stream>' "
            f"str (e.g. {TRAJECTORY_ODOM_MEMORY2_REPLAY!r}), not a Path"
        )
    return TimedSensorReplay(source, autocast=wrap)
