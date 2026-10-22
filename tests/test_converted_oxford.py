import csv
import pickle
from pathlib import Path

import numpy as np
import pytest
import yaml
from evalio import datasets as ds
from evalio import pipelines as pl
from evalio import types as ty
from evalio.cli import app
from evalio.datasets.rosbag import OxfordSpiresRosbag
from rosbags.rosbag2 import StoragePlugin
from typer.testing import CliRunner

from tests.test_local_rosbag import make_bag


def test_oxford_example_paths(tmp_path: Path):
    config = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "examples/oxford_spires.yml").read_text()
    )
    assert (
        config["dataset_root"]
        == "/home/nakao/dataset/oxford_spires_dataset/data/sequences"
    )
    assert len(config["datasets"]) == 13
    for entry in config["datasets"]:
        assert entry["imu_topic"] is None
        assert entry["lidar_topic"] == "/hesai/pandar"
        assert entry["profile"] == "oxford_spires"
        bag, gt = tmp_path / entry["rosbag"], tmp_path / entry["ground_truth"]
        bag.mkdir(parents=True)
        gt.parent.mkdir(parents=True)
        gt.write_text("1 0 0 0 0 0 0 1\n2 1 0 0 0 0 0 1\n")
    sequences = ds.parse_config(config["datasets"], tmp_path)
    assert isinstance(sequences, list) and len(sequences) == 13
    names = []
    for sequence, _ in sequences:
        assert isinstance(sequence, OxfordSpiresRosbag)
        assert sequence.lidar_params().num_rows == 64
        assert sequence.lidar_params().num_columns == 1200
        assert pickle.loads(pickle.dumps(sequence)).full_name == sequence.full_name
        names.append(sequence.full_name)
    assert "oxford_spires/keble_college_05" in names
    assert "oxford_spires/christ_church_01" not in names


@pytest.mark.parametrize("pipeline", ["kiss", "ct", "form"])
def test_converted_oxford_lidar_only(tmp_path: Path, pipeline: str):
    if pipeline not in pl.all_pipelines():
        pytest.skip(f"{pipeline} is not compiled")
    root = tmp_path / "data" / "sequences"
    sequence_dir = root / "2024-03-12-keble-college-05"
    make_bag(
        sequence_dir / "rosbag_mcap",
        False,
        StoragePlugin.MCAP,
        lidar_topic="/hesai/pandar",
        oxford=True,
    )
    reference = ds.OxfordSpires.blenheim_palace_01
    gt_T_lidar = reference.imu_T_gt().inverse() * reference.imu_T_lidar()
    truth = sequence_dir / "processed" / "trajectory" / "gt-tum.txt"
    truth.parent.mkdir(parents=True)
    rows = []
    for i in range(4):
        pose = (
            ty.SE3(ty.SO3.identity(), np.array([i * 0.02, 0.0, 0.0]))
            * gt_T_lidar.inverse()
        )
        rows.append(
            " ".join(
                str(v)
                for v in [
                    1 + i * 0.1,
                    *pose.trans,
                    pose.rot.qx,
                    pose.rot.qy,
                    pose.rot.qz,
                    pose.rot.qw,
                ]
            )
        )
    truth.write_text("\n".join(rows))
    dataset = {
        "profile": "oxford_spires",
        "name": "keble_college_05",
        "rosbag": "2024-03-12-keble-college-05/rosbag_mcap",
        "ground_truth": "2024-03-12-keble-college-05/processed/trajectory/gt-tum.txt",
        "lidar_topic": "/hesai/pandar",
        "imu_topic": None,
        "lidar": {"num_rows": 16, "num_columns": 180},
    }
    parsed = ds.parse_config(dataset, root)
    assert isinstance(parsed, list)
    sequence = parsed[0][0]
    iterator = sequence.data_iter()
    assert list(iterator.imu_iter()) == []
    scans = list(iterator)
    assert len(scans) == 4
    assert all(isinstance(scan, ty.LidarMeasurement) for scan in scans)
    gt = sequence.ground_truth()
    assert gt is not None
    expected = reference.imu_T_lidar().inverse()
    np.testing.assert_allclose(gt.poses[0].trans, expected.trans, atol=1e-9)
    config = {
        "dataset_root": "data/sequences",
        "datasets": [dataset],
        "pipelines": [pipeline],
        "evaluation": {"w_seconds": [0.1]},
        "output_dir": str(tmp_path / "results"),
    }
    config_path = tmp_path / "run.yml"
    config_path.write_text(yaml.safe_dump(config))
    result = CliRunner().invoke(app, ["run", "-c", str(config_path)])
    assert result.exit_code == 0, f"{result.output}\n{result.exception}"
    with (tmp_path / "results" / "evaluation.csv").open() as stream:
        report = list(csv.DictReader(stream))
    assert len(report) == 1
    assert report[0]["sequence"] == "oxford_spires/keble_college_05"
    assert report[0]["sequence_length"] == "4"
    assert np.isfinite(float(report[0]["ATEt"]))
