# Snapshot
- 2026-10-23 [USER] Add evo APE with -a to evaluation.csv, preserving existing metrics and keeping changes to main internals small.
- 2026-10-23 [CODE] New APE_evo_rmse column uses actual evo 1.38.1, original trajectories, 0.01s association and ape(align=True, correct_scale=False); missing associations/degenerate alignment leave only this column blank with a warning.
- 2026-10-22 [USER] Review all changes against main; retain config-only execution and minimize changes to existing internal code.
- 2026-10-22 [CODE] Reviewed against refreshed origin/main: run()/run_single() and CLI stats.py restored exactly. Existing-code diff reduced from 145 to 74 changed lines; retain only bag config integration, optional IMU, CT zero-padding removal and equal-time guard. Automatic evaluation uses unchanged stats.align/ate/rte APIs in the new helper.
- 2026-10-22 [USER] Existing MCAP layout supplied in tree.txt; root `/home/nakao/dataset/oxford_spires_dataset`, LiDAR `/hesai/pandar`, no IMU input.
- 2026-10-22 [CODE] Supersedes the original built-in-layout example: Oxford YAML directly lists all 13 date-named folders under data/sequences, rosbag_mcap and processed/trajectory/gt-tum.txt. Oxford profile preserves native point layout and calibration; dataset_root resolves relative paths.
- 2026-10-22 [USER] Add an Oxford Spires example YAML. [CODE] `examples/oxford_spires.yml` selects 13 built-in sequences and KissICP/CT-ICP/FORM; built-in saved ground truth is eligible for automatic evaluation.
- 2026-10-21 [USER] Automatically evaluate when ground_truth is supplied: run prints ATE/RTE and writes evaluation.csv; support adding/updating GT on completed runs.
- 2026-10-21 [USER] Actual LiDAR topic is `/livox/lidar_192_168_1_182`; editing only garden1 left garden2/3 on the loader default. Share the example topic through a YAML anchor; identify the dataset in topic errors.
- 2026-10-21 [USER] Create a branch from evalio main, support evaluating ROS 2 Jazzy Livox MID-360 bags with KissICP, CT-ICP and FORM, and push it.
- 2026-10-21 [USER] Target bags: `/home/nakao/dataset/rosbag/ral2026/garden1`, `garden2`, `garden3`.
- 2026-10-21 [TOOL] Those paths are absent in the cloud machine; validate generated ROS 2 bags and provide a runnable garden configuration.
- 2026-10-21 [TOOL] Branch `feat/rosbag-evaluation` starts at origin/main `70303e8`; Python 3.12, KissICP, 82 existing tests pass (2 skip without public datasets).
- 2026-10-21 [TOOL] Supersedes initial validation: all three pipelines compiled; 94 tests passed, 2 public-dataset cases skipped; lint, type checking and docs passed. Actual garden bags remain unavailable.
- 2026-08-27 [USER] Configure Zensical to continue publishing versioned documentation through the existing `mike` `gh-pages` workflow.
- 2026-08-27 [CODE] `mike` is installed from Squidfunk's Zensical-compatible fork; CI deploys `latest` from `main` and release versions with the `stable` alias.
- 2026-08-27 [CODE] `project.extra.version.provider = "mike"` enables Zensical's version selector.
- 2026-08-27 [CODE] `project.site_url` uses the canonical HTTPS GitHub Pages URL with a trailing slash, so mike generates correct version URLs.
- 2026-08-27 [USER] Requested the existing About content become the documentation root at `/evalio/`.
- 2026-08-27 [USER] Requested a compact, bottom-right copyright footer that uses the page colors.

# Done (recent)
- 2026-10-23 [CODE] Added evo -a translation APE RMSE to automatic CSV/table reports without modifying main run/stats/C++ internals; evo is a required dependency, YAML unchanged.
- 2026-10-22 [CODE] Reduced changes to existing execution and evaluation code; preserve YAML-only operation after installation, all three pipelines and GT refresh on completed runs.
- 2026-10-21 [CODE] Added config-backed MID-360 Jazzy bags (CustomMsg/PointCloud2, SQLite3/MCAP), spherical organization for FORM, optional TUM ground truth, reliable CLI failure status and garden1–3 config/build guide.
- 2026-08-27 [CODE] Completed the Zensical migration: navigation, generated reference pages, Markdown extensions, and the mike version selector are configured.
- 2026-08-27 [CODE] Added unique Lucide front-matter icons to authored and generated documentation pages.
- 2026-08-27 [CODE] Completed palette-aware compact and wordmark SVG logo generation and site integration.
- 2026-08-27 [CODE] Moved About to docs/index.md and homepage /evalio/; styled the compact bottom-right copyright with page colors while retaining Zensical attribution.

# Working set
- `python/evalio/cli/evaluation.py`
- `python/evalio/datasets/rosbag.py`
- `python/evalio/datasets/parser.py`
- `python/evalio/cli/run.py`
- `tests/test_local_rosbag.py`
- `scripts/setup_rosbag_pipelines.sh`
- `examples/mid360-garden.yml`
- `examples/oxford_spires.yml`
- `tests/test_auto_evaluation.py`
- `pyproject.toml`
- `docs/examples/oxford_spires.md`
- `uv.lock`

# Receipts
- 2026-10-23 [TOOL] evo APE: 104 tests passed, 2 skipped; Ruff, pyrefly and docs passed. Regression matches actual evo CLI -a and analytic RMSE with rotation/translation/scale error and unmatched timestamps. Installed uv run --no-sync refreshed three completed runs with finite APE_evo_rmse, no odometry rerun. Existing locked versions unchanged. Logs: /workspace/.setup/evo-ape-{full,install,docs,smoke}.log.
- 2026-10-22 [TOOL] Minimal-change review: 102 passed, 2 skipped (unavailable public datasets); Ruff, pyrefly, clang-format 20 and docs passed. Rebuilt wheel; standard uv run --no-sync evalio run produced 3 completed Oxford MCAP runs and 3 finite GT metrics with IMU disabled. Logs: /workspace/.setup/minimal-review-{build,tests,docs,smoke}.log. Actual user bags remain unavailable.
- 2026-10-22 [TOOL] Updated wheel installed; standard `uv run --no-sync evalio run` verified all three pipelines and automatic GT evaluation on a generated Oxford-format MCAP with no IMU input. Smoke log: `/workspace/.setup/converted-oxford-smoke.log`.
- 2026-10-22 [TOOL] Converted Oxford implementation: 102 tests passed, 2 public-dataset cases skipped; Ruff, pyrefly and docs passed. Generated MCAP tests run all three methods and evaluate GT with IMU disabled despite an IMU topic being present; attached tree's 13 sequence names match YAML exactly. Actual user MCAPs remain unavailable in cloud.
- 2026-10-22 [TOOL] Oxford YAML parsed into 13 sequences, three compiled pipelines and valid evaluation options; targeted auto-evaluation/rosbag suites: 16 passed. Ruff and pyrefly passed. Actual Oxford bags are not installed.
- 2026-10-21 [TOOL] Reinstalled the updated wheel; `uv run --no-sync evalio run` on the three generated bags skipped nine completed odometry runs and produced nine evaluation.csv rows with finite ATE. 1-second RTE was blank for these 0.3-second runs. Docs build passed.
- 2026-10-21 [TOOL] Automatic-evaluation checks: 97 passed, 2 public-dataset cases skipped; Ruff and pyrefly passed. Generated bag tests compare saved metrics with stats and cover late/changed GT, unrelated files, failed runs, short windows and nonoverlapping clocks.
- 2026-10-21 [TOOL] Topic regression tests cover editing the example's single shared value across all three bags and naming garden2 in CLI errors; local rosbag suite: 14 passed; Ruff and type checking verified.
- 2026-10-21 [TOOL] `/workspace/.setup/install_all.sh` rerun succeeded with verified Debian packages and local C++ prefixes; activation at `/workspace/.setup/activate.sh`. Install/start instructions saved to environment draft; not published.
- 2026-10-21 [TOOL] Generated garden1–3 bags: nine runs produced four poses each; CLI stats computed ATE/RTE; repeat skipped nine completed runs. Logs: `/workspace/.setup/garden-{run,stats,repeat}.log`.
- 2026-10-21 [TOOL] `PYTHONPATH=python pytest -q`: 94 passed, 2 skipped; pyrefly: 0 errors; Ruff and clang-format 20 passed; Zensical built without issues.
- 2026-08-27 [TOOL] `uv run python scripts/gen_docs.py && uv run zensical build` completed with no issues; generated canonical links use `https://contagon.github.io/evalio/`.
- 2026-08-27 [TOOL] `uv run mike list` reports `latest`, `0.6.0 [stable]`, and prior published versions on `gh-pages`; the installed mike identifies itself as Zensical-compatible.
- 2026-08-27 [TOOL] Zensical builds completed with mkdocstrings, math, navigation, palette-aware logo assets, and all generated documentation pages.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after moving the documentation homepage to `index.md`.
- 2026-08-27 [TOOL] `uv run zensical build` completed with no issues after the footer CSS override.
- 2026-08-27 [TOOL] `uv run zensical build && git diff --check` completed with no issues after separating the footer alignment.
- 2026-08-27 [TOOL] `uv run zensical build && git diff --check` completed with no issues after reducing the footer-bar height.
