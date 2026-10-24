"""Automatic evaluation of the experiments selected by a run config."""

import csv
import math
from pathlib import Path
from typing import Any

from evo import EvoException
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.main_ape import ape
from rich.console import Console
from rich.table import Table

from evalio import stats
from evalio import types as ty
from evalio.datasets.base import Dataset
from evalio.datasets.rosbag import RosbagDataset
from evalio.utils import print_warning


def _evo_ape_rmse(estimate: ty.Trajectory, ground_truth: ty.Trajectory) -> float:
    def convert(trajectory: ty.Trajectory) -> PoseTrajectory3D:
        return PoseTrajectory3D(
            poses_se3=[pose.to_mat() for pose in trajectory.poses],
            timestamps=[stamp.to_sec() for stamp in trajectory.stamps],
        )

    reference, estimate_evo = sync.associate_trajectories(
        convert(ground_truth), convert(estimate), max_diff=0.01
    )
    result = ape(
        reference,
        estimate_evo,
        metrics.PoseRelation.translation_part,
        align=True,
        correct_scale=False,
    )
    return float(result.stats["rmse"])


def _evaluate_trajectory(
    file: Path,
    ground_truth: ty.Trajectory,
    windows: list[stats.WindowKind],
    metric: stats.MetricKind,
) -> dict[str, Any] | None:
    trajectory = ty.Trajectory.from_file(file)
    if not isinstance(trajectory, ty.Trajectory) or not isinstance(
        trajectory.metadata, ty.Experiment
    ):
        return None
    if trajectory.metadata.status != ty.ExperimentStatus.Complete:
        return trajectory.metadata.to_dict()
    if (
        len(trajectory) < 2
        or trajectory.stamps[-1] < ground_truth.stamps[0]
        or ground_truth.stamps[-1] < trajectory.stamps[0]
    ):
        raise ValueError(f"{file.stem}: require two poses with overlapping timestamps")
    aligned, truth = stats.align(trajectory, ground_truth)
    if len(aligned) < 2:
        raise ValueError(f"{file.stem}: fewer than two aligned poses")
    result = trajectory.metadata.to_dict()
    elapsed = trajectory.metadata.total_elapsed
    result["hz"] = len(trajectory) / elapsed if elapsed and elapsed > 0 else None
    ate = stats.ate(aligned, truth).summarize(metric)
    result.update(ATEt=ate.trans, ATEr=ate.rot)
    try:
        result["APE_evo_rmse"] = _evo_ape_rmse(trajectory, ground_truth)
    except EvoException as error:
        result["APE_evo_rmse"] = None
        print_warning(f"evo APE unavailable for {file}: {error}")
    result["ATE_evo_rmse"] = result["APE_evo_rmse"]
    for window in windows:
        rte = stats.rte(aligned, truth, window).summarize(metric)
        result.update(
            {f"RTEt_{window.name()}": rte.trans, f"RTEr_{window.name()}": rte.rot}
        )
    return result


def evaluation_options(config: Any) -> tuple[list[stats.WindowKind], stats.MetricKind]:
    if not isinstance(config, dict):
        raise TypeError("evaluation must be a mapping")
    unknown = config.keys() - {"w_seconds", "w_meters", "metric"}
    if unknown:
        raise ValueError(f"Unknown evaluation options: {sorted(unknown)}")
    windows: list[stats.WindowKind] = []
    for key, constructor, default in (
        ("w_seconds", stats.WindowSeconds, [1.0]),
        ("w_meters", stats.WindowMeters, []),
    ):
        values = config.get(key, default)
        if not isinstance(values, list):
            raise TypeError(f"evaluation.{key} must be a list")
        for value in values:
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(
                    f"evaluation.{key} must contain positive finite numbers"
                )
            window = constructor(float(value))
            if window not in windows:
                windows.append(window)
    return windows, stats.MetricKind(config.get("metric", "sse"))


def evaluate_runs(
    experiments: list[ty.Experiment],
    output: Path,
    windows: list[stats.WindowKind],
    metric: stats.MetricKind,
) -> bool:
    groups: dict[Path, list[Path]] = {}
    refreshed: set[Dataset] = set()
    success = True
    for experiment in experiments:
        if experiment.file is None or not isinstance(experiment.sequence, Dataset):
            continue
        sequence = experiment.sequence
        if isinstance(sequence, RosbagDataset) and sequence not in refreshed:
            gt = sequence.ground_truth()
            if gt is not None:
                gt.to_file(experiment.file.parent / "gt.csv")
            refreshed.add(sequence)
        trajectory = ty.Trajectory.from_file(experiment.file)
        if (
            not isinstance(trajectory, ty.Trajectory)
            or not isinstance(trajectory.metadata, ty.Experiment)
            or trajectory.metadata.status != ty.ExperimentStatus.Complete
        ):
            success = False
        if (
            sequence.config.get("ground_truth")
            if isinstance(sequence, RosbagDataset)
            else (experiment.file.parent / "gt.csv").is_file()
        ):
            groups.setdefault(experiment.file.parent, []).append(experiment.file)
    if not groups:
        return success

    columns = [
        "sequence",
        "name",
        "pipeline",
        "pipeline_version",
        "status",
        "metric",
        "sequence_length",
        "ATEt",
        "ATEr",
        "ATE_evo_rmse",
        "APE_evo_rmse",
    ]
    for window in windows:
        columns.extend([f"RTEt_{window.name()}", f"RTEr_{window.name()}"])
    columns.extend(["total_elapsed", "max_elapsed", "hz"])
    rows: list[dict[str, Any]] = []
    for directory, files in groups.items():
        try:
            ground_truth = ty.Trajectory.from_file(directory / "gt.csv")
            if not isinstance(ground_truth, ty.Trajectory) or len(ground_truth) < 2:
                raise ValueError("Ground truth requires at least two poses")
            results = [
                _evaluate_trajectory(file, ground_truth, windows, metric)
                for file in files
            ]
        except (ValueError, IndexError) as error:
            print_warning(f"Evaluation failed for {directory.name}: {error}")
            success = False
            continue
        for result in results:
            if result is None:
                continue
            result["metric"] = metric.value
            rows.append(
                {
                    key: None
                    if isinstance(value := result.get(key), float)
                    and not math.isfinite(value)
                    else value
                    for key in columns
                }
            )

    report = output / "evaluation.csv"
    report.parent.mkdir(parents=True, exist_ok=True)
    with report.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    table = Table(title=f"Ground-truth evaluation ({metric.value})")
    shown = [
        "sequence",
        "name",
        "status",
        "ATEt",
        "ATEr",
        "ATE_evo_rmse",
        "APE_evo_rmse",
        *[key for key in columns if key.startswith("RTE")],
    ]
    for column in shown:
        table.add_column(column)
    for row in rows:
        table.add_row(
            *[
                "—"
                if row[key] is None
                else f"{row[key]:.6f}"
                if isinstance(row[key], float)
                else str(row[key])
                for key in shown
            ]
        )
    console = Console()
    if rows:
        console.print(table)
    else:
        print_warning("No completed experiments available for ground-truth evaluation")
    console.print(f"Evaluation saved to {report}")
    return success
