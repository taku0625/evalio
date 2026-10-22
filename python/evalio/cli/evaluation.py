"""Automatic evaluation of the experiments selected by a run config."""

import csv
import math
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.table import Table

from evalio import stats
from evalio import types as ty
from evalio.cli.stats import eval_dataset
from evalio.datasets.base import Dataset
from evalio.datasets.rosbag import RosbagDataset
from evalio.utils import print_warning


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
    for experiment in experiments:
        if (
            experiment.file is not None
            and isinstance(experiment.sequence, Dataset)
            and (
                experiment.sequence.config.get("ground_truth")
                if isinstance(experiment.sequence, RosbagDataset)
                else (experiment.file.parent / "gt.csv").is_file()
            )
        ):
            groups.setdefault(experiment.file.parent, []).append(experiment.file)
    if not groups:
        return True

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
    ]
    for window in windows:
        columns.extend([f"RTEt_{window.name()}", f"RTEr_{window.name()}"])
    columns.extend(["total_elapsed", "max_elapsed", "hz"])
    rows: list[dict[str, Any]] = []
    success = True
    for directory, files in groups.items():
        try:
            results = eval_dataset(
                directory,
                False,
                windows,
                metric,
                None,
                files=files,
                only_complete=True,
                require_ground_truth=True,
            )
        except (ValueError, IndexError) as error:
            print_warning(f"Evaluation failed for {directory.name}: {error}")
            success = False
            continue
        for result in results or []:
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
        "ATEt",
        "ATEr",
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
