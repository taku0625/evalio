import csv
from pathlib import Path

import numpy as np
from evalio import datasets as ds
from evalio import stats
from evalio import types as ty
from evalio.cli.evaluation import evaluate_runs


def test_builtin_oxford_spires_evaluation(tmp_path: Path):
    sequence = ds.OxfordSpires.blenheim_palace_01
    directory = tmp_path / sequence.full_name
    stamps = [ty.Stamp.from_sec(float(i)) for i in range(1, 4)]
    gt = ty.Trajectory(
        stamps=stamps,
        poses=[
            ty.SE3(ty.SO3.identity(), np.array([float(i), 0.0, 0.0])) for i in range(3)
        ],
        metadata=ty.GroundTruth(sequence=sequence.full_name),
    )
    gt.to_file(directory / "gt.csv")
    experiment = ty.Experiment(
        name="kiss",
        sequence=sequence,
        sequence_length=3,
        pipeline="kiss",
        pipeline_version="test",
        pipeline_params={},
        status=ty.ExperimentStatus.Complete,
        file=directory / "kiss.csv",
    )
    trajectory = ty.Trajectory(
        stamps=stamps,
        poses=[
            ty.SE3(ty.SO3.identity(), np.array([1.1 * i, 0.0, 0.0])) for i in range(3)
        ],
        metadata=experiment,
    )
    trajectory.to_file()
    assert evaluate_runs(
        [experiment], tmp_path, [stats.WindowSeconds(1.0)], stats.MetricKind.sse
    )
    with (tmp_path / "evaluation.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]["sequence"] == sequence.full_name
    assert 0 < float(rows[0]["ATEt"]) < 0.2
    assert 0 < float(rows[0]["RTEt_1.0s"]) < 0.2
