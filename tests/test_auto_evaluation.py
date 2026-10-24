import csv
import json
import subprocess
import sys
import zipfile
from pathlib import Path

import numpy as np
import pytest
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
    assert rows[0]["APE_evo_rmse"] == ""
    assert rows[0]["ATE_evo_rmse"] == ""


@pytest.fixture
def evaluation_pair(tmp_path: Path):
    sequence = ds.OxfordSpires.blenheim_palace_01
    directory = tmp_path / sequence.full_name
    positions = np.array(
        [[0, 0, 0], [1, 0, 0], [2, 1, 0], [1, 2, 0.5], [0, 1, 1], [0.2, -0.5, 0.3]]
    )
    stamps = [ty.Stamp.from_sec(1 + i * 0.1) for i in range(len(positions))]
    ground_truth = ty.Trajectory(
        stamps=stamps,
        poses=[ty.SE3(ty.SO3.identity(), p) for p in positions],
        metadata=ty.GroundTruth(sequence=sequence.full_name),
    )
    ground_truth.to_file(directory / "gt.csv")
    experiment = ty.Experiment(
        name="kiss",
        sequence=sequence,
        sequence_length=len(stamps),
        pipeline="kiss",
        pipeline_version="test",
        pipeline_params={},
        status=ty.ExperimentStatus.Complete,
        file=directory / "kiss.csv",
    )
    transform = ty.SE3(
        ty.SO3(qx=0, qy=0, qz=np.sqrt(0.5), qw=np.sqrt(0.5)),
        np.array([5.0, -3.0, 1.0]),
    )
    estimate = ty.Trajectory(
        stamps=[ty.Stamp.from_sec(s.to_sec() + 0.005) for s in stamps],
        poses=[transform * ty.SE3(ty.SO3.identity(), 2 * p) for p in positions],
        metadata=experiment,
    )
    estimate.stamps[1] = ty.Stamp.from_sec(stamps[1].to_sec() + 0.025)
    estimate.to_file()
    return experiment, ground_truth, estimate


def test_evo_ate_and_ape_match_aligned_cli(
    tmp_path: Path, evaluation_pair, monkeypatch, capsys
):
    monkeypatch.setenv("COLUMNS", "240")
    experiment, ground_truth, estimate = evaluation_pair
    assert evaluate_runs([experiment], tmp_path, [], stats.MetricKind.sse)
    assert "ATE_evo_rmse" in capsys.readouterr().out
    with (tmp_path / "evaluation.csv").open() as stream:
        row = next(csv.DictReader(stream))
    ape_rmse = float(row["APE_evo_rmse"])
    positions = np.array([p.trans for i, p in enumerate(ground_truth.poses) if i != 1])
    expected = np.sqrt(
        np.mean(np.sum((positions - positions.mean(axis=0)) ** 2, axis=1))
    )
    assert ape_rmse == pytest.approx(expected)
    assert 0 < ape_rmse < float(row["ATEt"])

    for file, trajectory in (
        ("reference.tum", ground_truth),
        ("estimate.tum", estimate),
    ):
        np.savetxt(
            tmp_path / file,
            [
                [s.to_sec(), *p.trans, p.rot.qx, p.rot.qy, p.rot.qz, p.rot.qw]
                for s, p in trajectory
            ],
            fmt="%.12f",
        )
    cli = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "from pathlib import Path\n"
                "from unittest.mock import patch\n"
                "import sys\n"
                "user_dir = Path(sys.argv.pop(1))\n"
                "with patch.object(Path, 'home', return_value=user_dir):\n"
                "    from evo.cli.entry_points import ape\n"
                "    ape()\n"
            ),
            str(tmp_path),
            "tum",
            str(tmp_path / "reference.tum"),
            str(tmp_path / "estimate.tum"),
            "-a",
            "--save_results",
            str(tmp_path / "evo.zip"),
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert cli.returncode == 0, cli.stdout + cli.stderr
    with zipfile.ZipFile(tmp_path / "evo.zip") as archive:
        cli_stats = json.loads(archive.read("stats.json"))
    assert ape_rmse == pytest.approx(cli_stats["rmse"])
    assert float(row["ATE_evo_rmse"]) == pytest.approx(cli_stats["rmse"])


def test_evo_no_timestamp_matches_keeps_existing_metrics(
    tmp_path: Path, evaluation_pair, capsys
):
    experiment, _, estimate = evaluation_pair
    estimate.stamps = [ty.Stamp.from_sec(1.03 + i * 0.1) for i in range(len(estimate))]
    estimate.to_file()
    assert evaluate_runs([experiment], tmp_path, [], stats.MetricKind.sse)
    with (tmp_path / "evaluation.csv").open() as stream:
        row = next(csv.DictReader(stream))
    assert row["APE_evo_rmse"] == ""
    assert row["ATE_evo_rmse"] == ""
    assert np.isfinite(float(row["ATEt"]))
    assert "evo APE unavailable" in capsys.readouterr().out
