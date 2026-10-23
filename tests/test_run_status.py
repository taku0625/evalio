import os
import resource
import signal
from pathlib import Path

import numpy as np
import pytest
from evalio import datasets as ds
from evalio import pipelines as pl
from evalio import types as ty
from evalio.cli.run import run
from evalio.rerun import RerunVis
from rosbags.rosbag2 import StoragePlugin

from tests.test_local_rosbag import config_for, make_bag


class StatusPipeline(pl.Pipeline):
    @staticmethod
    def name() -> str:
        return "status_test"

    @staticmethod
    def default_params() -> dict[str, ty.Param]:
        return {"mode": "ok"}

    def initialize(self):
        self.count = 0

    def set_params(self, params):
        self.mode = params["mode"]
        return {}

    def set_lidar_params(self, params):
        pass

    def set_imu_params(self, params):
        pass

    def set_imu_T_lidar(self, T):
        pass

    def map(self):
        return {}

    def add_lidar(self, mm):
        self.count += 1
        if self.mode == "no_poses":
            return
        if self.count == 2:
            if self.mode == "exception":
                raise RuntimeError("simulated registration failure")
            if self.mode == "abort":
                resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
                os.kill(os.getpid(), signal.SIGABRT)
            if self.mode == "exit_zero":
                os._exit(0)
        translation = np.zeros(3)
        if self.mode == "nonfinite" and self.count == 2:
            translation[0] = np.nan
        self.save(mm.stamp, ty.SE3(ty.SO3.identity(), translation))


@pytest.mark.parametrize(
    ("mode", "requested"),
    [
        ("ok", 4),
        ("no_poses", 3),
        ("nonfinite", 3),
        ("exception", 3),
        ("abort", 3),
        ("exit_zero", 3),
    ],
)
def test_unsuccessful_runs_saved_as_failed(tmp_path: Path, mode: str, requested: int):
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3, frames=3)
    parsed = ds.parse_config(config_for(tmp_path / "bag"))
    assert isinstance(parsed, list)
    experiment = ty.Experiment(
        name="status_test",
        sequence=parsed[0][0],
        sequence_length=requested,
        pipeline=StatusPipeline,
        pipeline_version="test",
        pipeline_params={"mode": mode},
        file=tmp_path / "results" / "status_test.csv",
    )
    run([experiment], RerunVis(None, [experiment.name]), False)
    assert experiment.file is not None
    saved = ty.Trajectory.from_file(experiment.file)
    assert isinstance(saved, ty.Trajectory)
    assert isinstance(saved.metadata, ty.Experiment)
    assert saved.metadata.status == ty.ExperimentStatus.Fail
    if mode in ("exception", "nonfinite"):
        assert len(saved) == 1
        assert np.isfinite(saved.poses[0].to_mat()).all()
