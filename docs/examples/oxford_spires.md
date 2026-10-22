# Converted Oxford Spires bags

`examples/oxford_spires.yml` uses existing MCAP bags and ground truth in this layout:

```text
data/sequences/2024-03-12-keble-college-02/
├── rosbag_mcap/
│   ├── metadata.yaml
│   └── rosbag_mcap_0.mcap
└── processed/trajectory/gt-tum.txt
```

Set `dataset_root` in the YAML to your `data/sequences` directory. The sample
lists the 13 available converted sequences, including `keble_college_05`.
Each `rosbag` and `ground_truth` path is relative to `dataset_root`; a relative
`dataset_root` is resolved from the YAML file's directory. This setting does not
require the `EVALIO_DATA` directory convention.

After updating the checkout, refresh the installed Python package and run:

```sh
bash scripts/setup_rosbag_pipelines.sh
uv run --no-sync evalio run -c examples/oxford_spires.yml
```

The build script is needed only to install or update evalio. After installation,
edit the YAML and run `evalio run -c ...`; loading, odometry and ground-truth
evaluation require no changes to evalio's source code.

The configuration runs KissICP, CT-ICP and FORM and writes trajectories under
`results/oxford_spires/oxford_spires/<sequence>/`. ATE/RTE comparisons are printed
and saved to `results/oxford_spires/evaluation.csv`. Remove dataset entries to
run a subset, or add `length: 100` to the first anchored entry for a short run
across all entries.

The `profile: oxford_spires` setting uses the original Hesai QT-64 layout
(64 rows, 1200 columns) and Oxford Spires calibration, preserving per-point
times and organizing the original returns for FORM. It does not apply MID-360
spherical projection. Ground truth is read in Oxford Spires' original base
frame and transformed using the dataset's existing extrinsics.

The default LiDAR topic is `/hesai/pandar`. The example explicitly sets
`imu_topic: null`, so no IMU messages are read or passed to the pipelines,
including when an IMU topic is present in the bag. The rigid calibration
transforms are still used to put estimates and ground truth in the same frame.
These three methods perform LiDAR-only odometry.

The separate built-in `oxford_spires/*` dataset selector remains available for
the original evalio download layout. Converted bags can use arbitrary sequence
names and explicit paths through the profile above.
