# Golden replay constants (issue 921 P6-3 / T-20)

``dimos/navigation/test_trajectory_golden_replay.py`` pins expected ``cmd_vel`` body-frame components and planar position divergence for the first few odometry samples from the default memory2 replay stream (``go2_bigoffice/odom``).

When you change any of the following, regenerate the tuples before updating the test:

- ``HolonomicTrackingController`` law or gains used in the test
- ``pose_errors_vs_reference`` / ``planar_position_divergence`` in ``trajectory_metrics``
- The shipped ``go2_bigoffice.db`` content (LFS) or the default ``TRAJECTORY_ODOM_MEMORY2_REPLAY`` name in ``trajectory_replay_loader.py``

## How to print new constants

From the ``dimos`` repository root, with ``data/go2_bigoffice.db`` present (``git lfs pull`` if needed):

```sh
uv run python docs/development/921_trajectory_controller/golden_replay/print_golden_replay_constants.py
```

Copy the printed Python tuple literals into ``test_trajectory_golden_replay.py``.

Options: ``--steps N`` (default 3), ``--k-position 1.0``, ``--k-yaw 0.75``.

## Relation to the backlog

- **P5-3** (``trajectory_controller_backlog.md``): replay loader + ``get_data`` / memory2 alignment so calibration and regression can use the same replay stack as ``ReplayConnection``.
- **P6-3**: optional golden replay slice with pinned commands vs divergence so controller math does not drift silently.
- **P5-5 / CI**: Linux default workflows often omit LFS; tests skip when the DB is missing. Machines with ``go2_bigoffice.db`` run the full assertions.
