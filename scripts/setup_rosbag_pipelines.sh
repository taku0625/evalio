#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
uv sync --frozen --dev --all-extras --no-install-project
export PATH="$PWD/.venv/bin:$PATH"
export CC="${CC:-cc}" CXX="${CXX:-c++}"
command -v "$CC" >/dev/null || export CC=cc
command -v "$CXX" >/dev/null || export CXX=c++

prepare() {
    local name="$1" url="$2" tag="$3" commit="$4" patch="$5"
    local source="$PWD/cpp/bindings/pipelines-src/$name"
    if [ ! -d "$source" ]; then
        git clone --branch "$tag" --depth 1 "$url" "$source"
    fi
    if [ "$(git -C "$source" rev-parse HEAD)" != "$commit" ]; then
        echo "Unexpected revision in $source; preserve your changes and inspect it before rebuilding." >&2
        exit 1
    fi
    local patch_path="$PWD/cpp/bindings/pipelines/$patch"
    if git -C "$source" apply --check "$patch_path" 2>/dev/null; then
        git -C "$source" apply "$patch_path"
    else
        git -C "$source" apply --reverse --check "$patch_path"
    fi
}

prepare kiss-icp https://github.com/PRBonn/kiss-icp.git v1.3.0 b16835283aee62f7d5e2bdf6c1c3bb2930de74ff kiss_icp.patch
prepare ct_icp https://github.com/jedeschaud/ct_icp.git icra_2022 70cea30713d8189a1693ea62fb22c980d8b30e6e ct_icp.patch
prepare form https://github.com/rpl-cmu/form.git v0.2.0 ec7094429f35c3244366815c7d4319be6b4367dc form.patch

uv sync --frozen --dev --all-extras --no-editable --reinstall-package evalio
cp -r .venv/lib/python3.12/site-packages/evalio/_cpp* python/evalio/
.venv/bin/evalio ls pipelines
.venv/bin/python - <<'PY'
from evalio.pipelines import all_pipelines
missing = {"kiss", "ct", "form"} - all_pipelines().keys()
if missing:
    raise SystemExit(f"Missing compiled pipelines: {sorted(missing)}")
PY
