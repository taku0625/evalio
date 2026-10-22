"""Configuration-backed ROS 2 bags, including Livox MID-360 messages."""

import json
import re
from collections.abc import Iterator, Sequence
from functools import cached_property
from itertools import pairwise
from pathlib import Path
from typing import Any, cast

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from evalio import types as ty
from evalio.datasets.base import Dataset, DatasetIterator, Measurement
from evalio.datasets.loaders import (
    LidarDensity,
    LidarFormatParams,
    LidarMajor,
    LidarPointStamp,
    LidarStamp,
    RosbagIter,
)
from evalio.datasets.oxford_spires import OxfordSpires

CUSTOM_POINT = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
CUSTOM_MSG = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
"""


def rosbag_dataset(config: dict[str, Any], base_dir: Path = Path(".")) -> Dataset:
    """Create a local sequence from a rosbag dataset entry in a run config."""
    config = dict(config)
    config.pop("length", None)
    profile = config.setdefault("profile", "mid360")
    if profile not in ("mid360", "oxford_spires"):
        raise ValueError(f"Unknown rosbag profile: {profile}")
    path = (base_dir / Path(config["rosbag"]).expanduser()).resolve()
    if not path.exists():
        raise ValueError(f"Rosbag does not exist: {path}")
    config["rosbag"] = str(path)
    config.setdefault("name", path.stem)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", config["name"]):
        raise ValueError("Rosbag name must contain only letters, digits, _, - or .")
    config.setdefault(
        "lidar_topic", "/hesai/pandar" if profile == "oxford_spires" else "/livox/lidar"
    )
    config.setdefault("imu_topic", None)
    lidar = dict(config.get("lidar", {}))
    defaults = {
        "num_rows": 32,
        "num_columns": 512,
        "rate": 10.0,
        "min_range": 0.1,
        "max_range": 70.0,
    }
    if profile == "oxford_spires":
        params = OxfordSpires.blenheim_palace_01.lidar_params()
        defaults = {key: getattr(params, key) for key in defaults}
    for key, value in defaults.items():
        lidar.setdefault(key, value)
    for key, limit in (("num_rows", 255), ("num_columns", 65535)):
        if type(lidar[key]) is not int or not 1 <= lidar[key] <= limit:
            raise ValueError(f"{key} must be an integer in [1, {limit}]")
    if not 0 < lidar["rate"] or not 0 < lidar["min_range"] < lidar["max_range"]:
        raise ValueError("Require rate > 0 and 0 < min_range < max_range")
    config["lidar"] = lidar
    projection = dict(config.get("projection", {}))
    projection.setdefault("min_elevation", -7.0)
    projection.setdefault("max_elevation", 52.0)
    if not -90 <= projection["min_elevation"] < projection["max_elevation"] <= 90:
        raise ValueError("Require -90 <= min_elevation < max_elevation <= 90")
    config["projection"] = projection
    config.setdefault("time_field", None)
    config.setdefault("time_unit", "auto")
    if config["time_unit"] not in ("auto", "s", "ms", "us", "ns"):
        raise ValueError("time_unit must be auto, s, ms, us or ns")
    if config.get("ground_truth"):
        gt = (base_dir / Path(config["ground_truth"]).expanduser()).resolve()
        if not gt.is_file():
            raise ValueError(f"Ground truth does not exist: {gt}")
        config["ground_truth"] = str(gt)
    dataset_class = OxfordSpiresRosbag if profile == "oxford_spires" else RosbagDataset
    sequence_type = dataset_class(
        "LocalRosbag", {"sequence": json.dumps(config, sort_keys=True)}
    )
    return cast(Dataset, sequence_type.sequence)


class RosbagDataset(Dataset):
    """An unregistered, configuration-backed local bag in the LiDAR frame."""

    @cached_property
    def config(self) -> dict[str, Any]:
        return json.loads(self.value)

    def __reduce_ex__(self, protocol: int):
        return rosbag_dataset, (self.config,)

    @classmethod
    def dataset_name(cls) -> str:
        return "rosbag"

    @property
    def seq_name(self) -> str:
        return self.config["name"]

    def files(self) -> Sequence[Path]:
        return [Path(self.config["rosbag"])]

    def data_iter(self) -> DatasetIterator:
        return Mid360RosbagIter(self.config)

    def imu_T_lidar(self) -> ty.SE3:
        return ty.SE3.identity()

    def imu_T_gt(self) -> ty.SE3:
        return ty.SE3.identity()

    def imu_params(self) -> ty.ImuParams:
        return ty.ImuParams()

    def lidar_params(self) -> ty.LidarParams:
        return ty.LidarParams(**self.config["lidar"])

    def ground_truth_raw(self) -> ty.Trajectory | None:
        if not self.config.get("ground_truth"):
            return None
        stamps, poses = [], []
        with Path(self.config["ground_truth"]).open() as stream:
            for line in stream:
                line = line.split("#", 1)[0].strip()
                if not line:
                    continue
                values = [float(v) for v in line.split()]
                if len(values) != 8 or not np.isfinite(values).all():
                    raise ValueError("TUM rows must contain eight finite numbers")
                stamp, x, y, z, qx, qy, qz, qw = values
                quat = np.array([qw, qx, qy, qz])
                if np.linalg.norm(quat) == 0:
                    raise ValueError("TUM quaternion must be nonzero")
                stamps.append(ty.Stamp.from_sec(stamp))
                qw, qx, qy, qz = quat / np.linalg.norm(quat)
                poses.append(
                    ty.SE3(
                        rot=ty.SO3(
                            qx=float(qx), qy=float(qy), qz=float(qz), qw=float(qw)
                        ),
                        trans=np.array([x, y, z]),
                    )
                )
        if len(stamps) < 2 or any(not a < b for a, b in pairwise(stamps)):
            raise ValueError(
                "TUM ground truth requires at least two increasing timestamps"
            )
        return ty.Trajectory(stamps=stamps, poses=poses)


class OxfordSpiresRosbag(RosbagDataset):
    """A converted Oxford Spires bag with its original calibration and point layout."""

    @classmethod
    def dataset_name(cls) -> str:
        return "oxford_spires"

    def data_iter(self) -> DatasetIterator:
        return RosbagIter(
            Path(self.config["rosbag"]),
            self.config["lidar_topic"],
            self.config["imu_topic"],
            self.lidar_params(),
            lidar_format=LidarFormatParams(
                stamp=LidarStamp.Start,
                point_stamp=LidarPointStamp.Start,
                major=LidarMajor.Column,
                density=LidarDensity.OnlyValidPoints,
            ),
        )

    def imu_T_lidar(self) -> ty.SE3:
        return OxfordSpires.blenheim_palace_01.imu_T_lidar()

    def imu_T_gt(self) -> ty.SE3:
        return OxfordSpires.blenheim_palace_01.imu_T_gt()

    def imu_params(self) -> ty.ImuParams:
        return OxfordSpires.blenheim_palace_01.imu_params()


class Mid360RosbagIter(DatasetIterator):
    """Read Jazzy bags without ROS and project MID-360 returns onto an angular grid."""

    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.path = Path(config["rosbag"])
        self.params = ty.LidarParams(**config["lidar"])
        self.typestore = get_typestore(Stores.ROS2_JAZZY)
        for package in ("livox_ros_driver2", "livox_ros_driver"):
            self.typestore.register(
                get_types_from_msg(CUSTOM_POINT, f"{package}/msg/CustomPoint")
            )
            self.typestore.register(
                get_types_from_msg(CUSTOM_MSG, f"{package}/msg/CustomMsg")
            )
        with self.reader() as reader:
            lidar = [c for c in reader.connections if c.topic == config["lidar_topic"]]
            if not lidar:
                topics = ", ".join(sorted({c.topic for c in reader.connections}))
                raise ValueError(
                    f"Missing LiDAR topic {config['lidar_topic']}; available: {topics}"
                )
            for connection in lidar:
                if (
                    connection.msgtype != "sensor_msgs/msg/PointCloud2"
                    and connection.msgtype
                    not in (
                        "livox_ros_driver2/msg/CustomMsg",
                        "livox_ros_driver/msg/CustomMsg",
                    )
                ):
                    raise ValueError(f"Unsupported LiDAR message: {connection.msgtype}")
            imu_topic = config["imu_topic"]
            if imu_topic and not any(
                c.topic == imu_topic and c.msgtype == "sensor_msgs/msg/Imu"
                for c in reader.connections
            ):
                raise ValueError(f"Missing IMU topic {imu_topic} (sensor_msgs/msg/Imu)")
            self.count = sum(c.msgcount for c in lidar)
            if self.count == 0:
                raise ValueError("LiDAR topic contains no messages")

    def reader(self) -> AnyReader:
        return AnyReader([self.path], default_typestore=self.typestore)

    def __len__(self) -> int:
        return self.count

    def __iter__(self) -> Iterator[Measurement]:
        with self.reader() as reader:
            connections = [
                c
                for c in reader.connections
                if c.topic in (self.config["lidar_topic"], self.config["imu_topic"])
            ]
            for connection, _, raw in reader.messages(connections=connections):
                msg = reader.deserialize(raw, connection.msgtype)
                if connection.topic == self.config["lidar_topic"]:
                    yield self.convert_lidar(msg, connection.msgtype)
                else:
                    stamp = ty.Stamp(
                        sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec
                    )
                    gyro, accel = msg.angular_velocity, msg.linear_acceleration
                    yield ty.ImuMeasurement(
                        stamp,
                        np.array([gyro.x, gyro.y, gyro.z]),
                        np.array([accel.x, accel.y, accel.z]),
                    )

    def imu_iter(self) -> Iterator[ty.ImuMeasurement]:
        return (m for m in self if isinstance(m, ty.ImuMeasurement))

    def lidar_iter(self) -> Iterator[ty.LidarMeasurement]:
        return (m for m in self if isinstance(m, ty.LidarMeasurement))

    def convert_lidar(self, msg: Any, msgtype: str) -> ty.LidarMeasurement:
        stamp = ty.Stamp(sec=msg.header.stamp.sec, nsec=msg.header.stamp.nanosec)
        if msgtype.endswith("/CustomMsg"):
            if msg.point_num != len(msg.points):
                raise ValueError("Livox point_num does not match points length")
            xyz = np.array([(p.x, p.y, p.z) for p in msg.points], dtype=float).reshape(
                -1, 3
            )
            intensity = np.array([p.reflectivity for p in msg.points])
            times = np.array([p.offset_time for p in msg.points], dtype=float) * 1e-9
        else:
            xyz, intensity, times = self.pointcloud_arrays(msg)
        return self.project(stamp, xyz, intensity, times)

    def pointcloud_arrays(self, msg: Any) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        formats = {
            1: "i1",
            2: "u1",
            3: "i2",
            4: "u2",
            5: "i4",
            6: "u4",
            7: "f4",
            8: "f8",
        }
        fields = {f.name: f for f in msg.fields}
        if not {"x", "y", "z"} <= fields.keys():
            raise ValueError("PointCloud2 requires x, y and z fields")

        def values(name: str) -> np.ndarray:
            field = fields[name]
            if field.count != 1 or field.datatype not in formats:
                raise ValueError(f"Unsupported PointCloud2 field {name}")
            dtype = np.dtype(
                (">" if msg.is_bigendian else "<") + formats[field.datatype]
            )
            if field.offset < 0 or field.offset + dtype.itemsize > msg.point_step:
                raise ValueError(f"PointCloud2 field {name} exceeds point_step")
            return np.ndarray(
                (msg.height, msg.width),
                dtype=dtype,
                buffer=msg.data,
                offset=field.offset,
                strides=(msg.row_step, msg.point_step),
            ).ravel()

        xyz = np.column_stack([values(name) for name in ("x", "y", "z")])
        intensity_field = next(
            (f for f in ("intensity", "reflectivity") if f in fields), None
        )
        intensity = values(intensity_field) if intensity_field else np.zeros(len(xyz))
        time_field = self.config["time_field"]
        if time_field is None:
            time_field = next(
                (f for f in ("offset_time", "timestamp", "time", "t") if f in fields),
                None,
            )
        if time_field is None:
            times = np.zeros(len(xyz))
        else:
            if time_field not in fields:
                raise ValueError(f"Missing PointCloud2 time field {time_field}")
            raw = values(time_field)
            unit = self.config["time_unit"]
            scale = (
                (1e-9 if raw.dtype.kind in "iu" else 1.0)
                if unit == "auto"
                else {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}[unit]
            )
            times = raw.astype(float) * scale
            if len(times) and np.nanmin(times) > 100.0:
                times -= msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return xyz, intensity, times

    def project(
        self, stamp: ty.Stamp, xyz: np.ndarray, intensity: np.ndarray, times: np.ndarray
    ) -> ty.LidarMeasurement:
        rows, columns = self.params.num_rows, self.params.num_columns
        ranges = np.linalg.norm(xyz, axis=1)
        valid = (
            np.isfinite(xyz).all(axis=1)
            & np.isfinite(times)
            & (ranges >= self.params.min_range)
            & (ranges <= self.params.max_range)
        )
        xyz, ranges, intensity, times = (
            xyz[valid],
            ranges[valid],
            intensity[valid],
            times[valid],
        )
        elevation = np.degrees(np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1])))
        low, high = (
            self.config["projection"][k] for k in ("min_elevation", "max_elevation")
        )
        valid = (elevation >= low) & (elevation <= high)
        xyz, ranges, intensity, times, elevation = (
            xyz[valid],
            ranges[valid],
            intensity[valid],
            times[valid],
            elevation[valid],
        )
        if len(xyz) == 0:
            raise ValueError(
                "LiDAR scan has no valid points within range and elevation limits"
            )
        row = np.minimum(
            ((elevation - low) / (high - low) * rows).astype(int), rows - 1
        )
        col = (
            (np.arctan2(xyz[:, 1], xyz[:, 0]) + np.pi) / (2 * np.pi) * columns
        ).astype(int) % columns
        cells = row * columns + col
        order = np.argsort(ranges, kind="stable")
        _, first = np.unique(cells[order], return_index=True)
        indices = order[first]
        points = [ty.Point(row=r, col=c) for r in range(rows) for c in range(columns)]
        for i in indices:
            x, y, z = xyz[i]
            points[cells[i]] = ty.Point(
                x=float(x),
                y=float(y),
                z=float(z),
                intensity=float(intensity[i]),
                t=ty.Duration.from_sec(float(times[i])),
                row=int(row[i]),
                col=int(col[i]),
            )
        return ty.LidarMeasurement(stamp, points)
