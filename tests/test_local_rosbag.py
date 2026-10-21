import csv
import pickle
import struct
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from evalio import datasets as ds
from evalio import pipelines as pl
from evalio import stats
from evalio import types as ty
from evalio.cli import app
from evalio.cli.stats import eval_dataset
from evalio.datasets.rosbag import (
    CUSTOM_MSG,
    CUSTOM_POINT,
    Mid360RosbagIter,
    RosbagDataset,
    rosbag_dataset,
)
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from typer.testing import CliRunner


def make_bag(
    path: Path,
    custom: bool,
    storage: StoragePlugin,
    frames: int = 4,
    timed: bool = True,
    rows: int = 16,
    columns: int = 180,
    lidar_topic: str = "/livox/lidar",
) -> None:
    store = get_typestore(Stores.ROS2_JAZZY)
    package = "livox_ros_driver2/msg/"
    store.register(get_types_from_msg(CUSTOM_POINT, package + "CustomPoint"))
    store.register(get_types_from_msg(CUSTOM_MSG, package + "CustomMsg"))
    types = store.types
    elevation, azimuth = np.meshgrid(
        np.linspace(-6, 51, rows), np.linspace(-179, 179, columns), indexing="ij"
    )
    elevation, azimuth = np.radians(elevation.ravel()), np.radians(azimuth.ravel())
    rays = np.column_stack(
        [
            np.cos(elevation) * np.cos(azimuth),
            np.cos(elevation) * np.sin(azimuth),
            np.sin(elevation),
        ]
    )
    dtype = np.dtype(
        [
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
            ("offset_time", "<u4"),
        ]
    )
    msgtype = package + "CustomMsg" if custom else "sensor_msgs/msg/PointCloud2"
    with Writer(path, version=9, storage_plugin=storage) as writer:
        connection = writer.add_connection(lidar_topic, msgtype, typestore=store)
        for i in range(frames):
            origin = np.array([0.02 * i, 0, 1.5])
            bounds = np.where(rays > 0, [6, 6, 4], [-6, -6, 0])
            distance = np.min((bounds - origin) / rays, axis=1)
            xyz = rays * distance[:, None]
            offsets = np.arange(len(xyz), dtype=np.uint32) * 10000
            stamp = 1_000_000_000 + i * 100_000_000
            header = types["std_msgs/msg/Header"](
                types["builtin_interfaces/msg/Time"](1, i * 100_000_000), "livox_frame"
            )
            if custom:
                points = [
                    types[package + "CustomPoint"](
                        int(t), float(x), float(y), float(z), 20, 0, 0
                    )
                    for (x, y, z), t in zip(xyz, offsets)
                ]
                msg = types[msgtype](
                    header, stamp, len(points), 0, np.zeros(3, dtype=np.uint8), points
                )
            else:
                data = np.zeros(len(xyz), dtype=dtype)
                for axis, name in enumerate(("x", "y", "z")):
                    data[name] = xyz[:, axis]
                data["intensity"], data["offset_time"] = 20, offsets
                field_names = ("x", "y", "z", "intensity", "offset_time")
                fields = [
                    types["sensor_msgs/msg/PointField"](
                        name,
                        offset * 4,
                        6 if name == "offset_time" else 7,
                        1,
                    )
                    for offset, name in enumerate(field_names)
                    if timed or name != "offset_time"
                ]
                msg = types[msgtype](
                    header,
                    1,
                    len(xyz),
                    fields,
                    False,
                    dtype.itemsize,
                    dtype.itemsize * len(xyz),
                    np.frombuffer(data.tobytes(), dtype=np.uint8),
                    True,
                )
            writer.write(connection, stamp, store.serialize_cdr(msg, msgtype))


def config_for(path: Path) -> dict:
    return {
        "rosbag": str(path),
        "name": "garden",
        "lidar": {"num_rows": 16, "num_columns": 180},
    }


def test_garden_example_shares_lidar_topic(tmp_path: Path):
    example = Path(__file__).resolve().parents[1] / "examples/mid360-garden.yml"
    topic = "/custom/lidar"
    config = yaml.safe_load(
        example.read_text().replace("/livox/lidar_192_168_1_182", topic)
    )
    for entry in config["datasets"]:
        bag = tmp_path / entry["name"]
        make_bag(bag, True, StoragePlugin.SQLITE3, frames=1, lidar_topic=topic)
        entry["rosbag"] = str(bag)
    sequences = ds.parse_config(config["datasets"])
    assert isinstance(sequences, list)
    assert len(sequences) == 3
    for sequence, _ in sequences:
        assert len(sequence) == 1


def test_topic_error_identifies_dataset(tmp_path: Path):
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3, frames=1)
    config = {
        "datasets": [
            config_for(Path("bag")) | {"name": "garden1"},
            config_for(Path("bag")) | {"name": "garden2", "lidar_topic": "/missing"},
        ],
        "pipelines": ["kiss"],
    }
    config_path = tmp_path / "run.yml"
    config_path.write_text(yaml.safe_dump(config))
    result = CliRunner().invoke(app, ["run", "-c", str(config_path)])
    assert result.exit_code == 2
    assert "rosbag/garden2" in result.output
    assert "Missing LiDAR topic /missing" in result.output


@pytest.mark.parametrize("custom", [False, True], ids=["pointcloud2", "livox"])
@pytest.mark.parametrize("storage", [StoragePlugin.SQLITE3, StoragePlugin.MCAP])
def test_jazzy_mid360(tmp_path: Path, custom: bool, storage: StoragePlugin):
    bag = tmp_path / "bag"
    make_bag(bag, custom, storage)
    sequence = rosbag_dataset(config_for(bag))
    assert isinstance(sequence, RosbagDataset)
    assert len(sequence) == 4
    assert sequence.full_name == "rosbag/garden"
    restored = pickle.loads(pickle.dumps(sequence))
    assert restored.config == sequence.config
    scans = list(sequence.lidar())
    assert len(scans) == 4
    assert scans[0].stamp == ty.Stamp.from_sec(1.0)
    assert len(scans[0].points) == 16 * 180
    valid = [p for p in scans[0].points if p.x or p.y or p.z]
    assert len(valid) > 2500
    assert max(p.t.to_sec() for p in valid) == pytest.approx(0.02879)
    assert len(list(sequence.imu())) == 0


def test_invalid_config_and_topics(tmp_path: Path):
    assert isinstance(
        ds.parse_config({"rosbag": "missing"}, tmp_path), ds.InvalidDatasetConfig
    )
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3)
    config = config_for(tmp_path / "bag")
    assert isinstance(
        ds.parse_config(config | {"name": "../escape"}), ds.InvalidDatasetConfig
    )
    assert isinstance(ds.parse_config(config | {"length": 0}), ds.InvalidDatasetConfig)
    with pytest.raises(ValueError, match="Missing LiDAR topic"):
        len(rosbag_dataset(config | {"lidar_topic": "/missing"}))
    with pytest.raises(ValueError, match="Missing IMU topic"):
        len(rosbag_dataset(config | {"imu_topic": "/missing"}))


def test_projection_keeps_nearest_return(tmp_path: Path):
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3)
    sequence = rosbag_dataset(config_for(tmp_path / "bag"))
    iterator = sequence.data_iter()
    assert isinstance(iterator, Mid360RosbagIter)
    scan = iterator.project(
        ty.Stamp.from_sec(1),
        np.array([[1, 0, 0], [2, 0, 0], [np.nan, 0, 0], [0, 0, 0]]),
        np.array([10, 20, 0, 0]),
        np.array([0.01, 0.02, 0, 0]),
    )
    valid = [p for p in scan.points if p.x]
    assert len(valid) == 1
    assert valid[0].x == 1
    assert valid[0].intensity == 10
    assert valid[0].t.to_sec() == pytest.approx(0.01)


def test_pointcloud_byte_order_padding_and_epoch_time(tmp_path: Path):
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3)
    iterator = rosbag_dataset(config_for(tmp_path / "bag")).data_iter()
    assert isinstance(iterator, Mid360RosbagIter)
    data = bytearray(112)
    for i in range(4):
        struct.pack_into(
            ">fff4xd",
            data,
            (i // 2) * 56 + (i % 2) * 24,
            float(i),
            1.0,
            2.0,
            1_700_000_000 + i * 0.01,
        )
    fields = [
        SimpleNamespace(name=name, datatype=kind, count=1, offset=offset)
        for name, kind, offset in [
            ("x", 7, 0),
            ("y", 7, 4),
            ("z", 7, 8),
            ("timestamp", 8, 16),
        ]
    ]
    msg = SimpleNamespace(
        fields=fields,
        is_bigendian=True,
        height=2,
        width=2,
        point_step=24,
        row_step=56,
        data=data,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1_700_000_000, nanosec=0)),
    )
    xyz, intensity, times = iterator.pointcloud_arrays(msg)
    np.testing.assert_allclose(xyz[:, 0], [0, 1, 2, 3])
    np.testing.assert_allclose(intensity, 0)
    np.testing.assert_allclose(times, [0, 0.01, 0.02, 0.03], atol=1e-6)


def test_ct_without_point_times(tmp_path: Path):
    if "ct" not in pl.all_pipelines():
        pytest.skip("ct is not compiled in this build")
    make_bag(tmp_path / "bag", False, StoragePlugin.SQLITE3, timed=False)
    config = {
        "datasets": [config_for(Path("bag"))],
        "pipelines": [
            {
                "pipeline": "ct",
                "motion_compensation": "NONE",
                "distance": "POINT_TO_PLANE",
                "ls_num_threads": 2,
            }
        ],
        "output_dir": str(tmp_path / "results"),
    }
    config_path = tmp_path / "run.yml"
    config_path.write_text(yaml.safe_dump(config))
    result = CliRunner().invoke(app, ["run", "-c", str(config_path)])
    assert result.exit_code == 0, f"{result.output}\n{result.exception}"
    trajectory = ty.Trajectory.from_file(
        tmp_path / "results" / "rosbag" / "garden" / "ct.csv"
    )
    assert isinstance(trajectory, ty.Trajectory)
    assert len(trajectory) == 4
    assert np.isfinite([p.trans for p in trajectory.poses]).all()
    assert not (tmp_path / "results" / "evaluation.csv").exists()


@pytest.mark.parametrize("pipeline", ["kiss", "ct", "form"])
def test_run_rosbag_cli(tmp_path: Path, pipeline: str):
    if pipeline not in pl.all_pipelines():
        pytest.skip(f"{pipeline} is not compiled in this build")
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3)
    (tmp_path / "gt.tum").write_text(
        "\n".join(f"{1 + i * 0.1:.1f} {i * 0.02} 0 0 0 0 0 1" for i in range(4))
    )
    config = {
        "datasets": [config_for(Path("bag")) | {"ground_truth": "gt.tum"}],
        "evaluation": {"w_seconds": [0.1]},
        "pipelines": [pipeline],
        "output_dir": str(tmp_path / "results"),
    }
    config_path = tmp_path / "run.yml"
    config_path.write_text(yaml.safe_dump(config))
    result = CliRunner().invoke(app, ["run", "-c", str(config_path)])
    assert result.exit_code == 0, f"{result.output}\n{result.exception}"
    trajectory = ty.Trajectory.from_file(
        tmp_path / "results" / "rosbag" / "garden" / f"{pipeline}.csv"
    )
    assert isinstance(trajectory, ty.Trajectory)
    assert isinstance(trajectory.metadata, ty.Experiment)
    assert trajectory.metadata.status == ty.ExperimentStatus.Complete
    assert len(trajectory) == 4
    assert np.isfinite([p.trans for p in trajectory.poses]).all()
    tolerance = 0.05 if pipeline == "kiss" else 0.025
    assert (
        np.linalg.norm(trajectory.poses[-1].trans - np.array([0.06, 0, 0])) < tolerance
    )
    metrics = eval_dataset(
        tmp_path / "results" / "rosbag" / "garden",
        False,
        [stats.WindowSeconds(0.1)],
        stats.MetricKind.sse,
        None,
    )
    assert metrics is not None and len(metrics) == 1
    assert metrics[0]["ATEt"] < 0.15
    assert np.isfinite(metrics[0]["RTEt_0.1s"])
    assert "Ground-truth evaluation" in result.output
    with (tmp_path / "results" / "evaluation.csv").open() as stream:
        report = list(csv.DictReader(stream))
    assert len(report) == 1
    assert report[0]["name"] == pipeline
    assert float(report[0]["ATEt"]) == pytest.approx(metrics[0]["ATEt"])
    assert float(report[0]["RTEt_0.1s"]) == pytest.approx(metrics[0]["RTEt_0.1s"])


def test_add_and_refresh_ground_truth_for_completed_run(tmp_path: Path):
    if "kiss" not in pl.all_pipelines():
        pytest.skip("kiss is not compiled in this build")
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3)
    dataset = config_for(Path("bag"))
    config = {
        "datasets": [dataset],
        "pipelines": ["kiss"],
        "evaluation": {"w_seconds": [0.1]},
        "output_dir": str(tmp_path / "results"),
    }
    config_path = tmp_path / "run.yml"
    config_path.write_text(yaml.safe_dump(config))
    runner = CliRunner()
    assert runner.invoke(app, ["run", "-c", str(config_path)]).exit_code == 0
    report_path = tmp_path / "results" / "evaluation.csv"
    assert not report_path.exists()
    trajectory = tmp_path / "results" / "rosbag" / "garden" / "kiss.csv"
    original = trajectory.read_bytes()
    (trajectory.parent / "unrelated.csv").write_bytes(original)
    dataset["ground_truth"] = "gt.tum"
    config_path.write_text(yaml.safe_dump(config))
    for translation in (0.02, 0.2):
        (tmp_path / "gt.tum").write_text(
            "\n".join(
                f"{1 + i * 0.1:.1f} {i * translation} 0 0 0 0 0 1" for i in range(4)
            )
        )
        result = runner.invoke(app, ["run", "-c", str(config_path)])
        assert result.exit_code == 0, result.output
        assert "already finished" in result.output
        assert trajectory.read_bytes() == original
        gt = ty.Trajectory.from_file(trajectory.parent / "gt.csv")
        assert isinstance(gt, ty.Trajectory)
        assert gt.poses[-1].trans[0] == pytest.approx(3 * translation)
        with report_path.open() as stream:
            report = list(csv.DictReader(stream))
        assert len(report) == 1
    config["evaluation"] = {"w_seconds": [1.0]}
    config_path.write_text(yaml.safe_dump(config))
    assert runner.invoke(app, ["run", "-c", str(config_path)]).exit_code == 0
    with report_path.open() as stream:
        row = next(csv.DictReader(stream))
    assert row["RTEt_1.0s"] == ""
    assert np.isfinite(float(row["ATEt"]))
    (tmp_path / "gt.tum").write_text("100 0 0 0 0 0 0 1\n101 1 0 0 0 0 0 1\n")
    result = runner.invoke(app, ["run", "-c", str(config_path)])
    assert result.exit_code == 1
    assert "overlapping" in result.output
    with report_path.open() as stream:
        assert list(csv.DictReader(stream)) == []


def test_failed_pipeline_returns_error_and_can_retry(tmp_path: Path):
    if "kiss" not in pl.all_pipelines():
        pytest.skip("kiss is not compiled in this build")
    make_bag(tmp_path / "bag", True, StoragePlugin.SQLITE3)
    dataset = config_for(Path("bag"))
    dataset["ground_truth"] = "gt.tum"
    (tmp_path / "gt.tum").write_text(
        "\n".join(f"{1 + i * 0.1:.1f} {i * 0.02} 0 0 0 0 0 1" for i in range(4))
    )
    dataset["lidar"]["max_range"] = 0.2
    config = {
        "datasets": [dataset],
        "pipelines": ["kiss"],
        "evaluation": {"w_seconds": [0.1]},
        "output_dir": str(tmp_path / "results"),
    }
    config_path = tmp_path / "run.yml"
    config_path.write_text(yaml.safe_dump(config))
    runner = CliRunner()
    result = runner.invoke(app, ["run", "-c", str(config_path)])
    assert result.exit_code == 1
    trajectory = ty.Trajectory.from_file(
        tmp_path / "results" / "rosbag" / "garden" / "kiss.csv"
    )
    assert isinstance(trajectory, ty.Trajectory)
    assert isinstance(trajectory.metadata, ty.Experiment)
    assert trajectory.metadata.status == ty.ExperimentStatus.Fail
    with (tmp_path / "results" / "evaluation.csv").open() as stream:
        assert list(csv.DictReader(stream)) == []
    assert runner.invoke(app, ["run", "-c", str(config_path)]).exit_code == 1
    dataset["lidar"]["max_range"] = 70.0
    config_path.write_text(yaml.safe_dump(config))
    result = runner.invoke(app, ["run", "-c", str(config_path), "--rerun-failed"])
    assert result.exit_code == 0, result.output
    with (tmp_path / "results" / "evaluation.csv").open() as stream:
        assert next(csv.DictReader(stream))["status"] == "complete"
