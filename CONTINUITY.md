# Snapshot
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
- 2026-10-21 [CODE] Added config-backed MID-360 Jazzy bags (CustomMsg/PointCloud2, SQLite3/MCAP), spherical organization for FORM, optional TUM ground truth, reliable CLI failure status and garden1–3 config/build guide.
- 2026-08-27 [CODE] Completed the Zensical migration: navigation, generated reference pages, Markdown extensions, and the mike version selector are configured.
- 2026-08-27 [CODE] Added unique Lucide front-matter icons to authored and generated documentation pages.
- 2026-08-27 [CODE] Completed palette-aware compact and wordmark SVG logo generation and site integration.
- 2026-08-27 [CODE] Moved `docs/about.md` to `docs/index.md`, retained its `About` nav label, and set `project.extra.homepage` to `/evalio/`.
- 2026-08-27 [CODE] Styled footer metadata with page color tokens; only the compact copyright is bottom-right, while Zensical attribution remains left.
- 2026-08-27 [CODE] Reduced footer metadata-bar vertical padding to `0.15rem` without affecting horizontal layout.

# Working set
- `python/evalio/datasets/rosbag.py`
- `python/evalio/datasets/parser.py`
- `python/evalio/cli/run.py`
- `tests/test_local_rosbag.py`
- `scripts/setup_rosbag_pipelines.sh`
- `examples/mid360-garden.yml`
- `zensical.toml`
- `.github/workflows/reusable_docs.yml`
- `pyproject.toml`
- `docs/index.md`
- `docs/css/tweaks.css`

# Receipts
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
