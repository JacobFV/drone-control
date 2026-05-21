#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PYTHON_BIN="${PYTHON_BIN:-python3.12}"
VENV_DIR="${VENV_DIR:-$REPO_ROOT/.venv}"
OPEN3D_VERSION="${OPEN3D_VERSION:-v0.19.0}"
OPEN3D_SRC="${OPEN3D_SRC:-$REPO_ROOT/vendor/Open3D}"
OPEN3D_BUILD="${OPEN3D_BUILD:-$REPO_ROOT/vendor/Open3D-build}"
JOBS="${JOBS:-$(nproc)}"
PIP_RETRIES="${PIP_RETRIES:-10}"
PIP_TIMEOUT="${PIP_TIMEOUT:-120}"

sudo_run() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  elif [[ -n "${SUDO_PASSWORD:-}" ]]; then
    printf "%s\n" "$SUDO_PASSWORD" | sudo -S "$@"
  else
    sudo "$@"
  fi
}

ensure_apt_packages() {
  sudo_run apt-get update
  sudo_run apt-get install -y \
    colmap \
    build-essential \
    cmake \
    ninja-build \
    git \
    curl \
    patchelf \
    pkg-config \
    python3.12-dev \
    python3.12-venv \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libegl1-mesa-dev \
    libx11-dev \
    libxi-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxrender-dev \
    libusb-1.0-0-dev \
    libopenblas-dev \
    liblapack-dev \
    liblapacke-dev \
    libidn2-dev \
    libwayland-bin \
    libwayland-dev \
    wayland-protocols \
    libxkbcommon-dev \
    gfortran \
    nasm
}

ensure_venv() {
  if [[ ! -x "$VENV_DIR/bin/python" ]]; then
    "$PYTHON_BIN" -m venv "$VENV_DIR"
  fi
  "$VENV_DIR/bin/python" -m pip install --retries "$PIP_RETRIES" --timeout "$PIP_TIMEOUT" --upgrade pip "setuptools<82" wheel
}

pip_install_requirements() {
  "$VENV_DIR/bin/python" -m pip install --retries "$PIP_RETRIES" --timeout "$PIP_TIMEOUT" -r requirements.txt
}

open3d_imports() {
  "$VENV_DIR/bin/python" - <<'PY'
import open3d
print(open3d.__version__)
PY
}

patch_open3d_runtime_deps() {
  local open3d_lib

  open3d_lib="$("$VENV_DIR/bin/python" - <<'PY'
from pathlib import Path
import sys

site_packages = [Path(p) for p in sys.path if p.endswith("site-packages")]
for site in site_packages:
    for path in sorted((site / "open3d" / "cpu").glob("libOpen3D.so*")):
        if path.is_file():
            print(path)
            raise SystemExit(0)
PY
)"

  if [[ -z "$open3d_lib" || ! -f "$open3d_lib" ]]; then
    return 0
  fi

  if nm -D --undefined-only "$open3d_lib" | grep -q " idn2_"; then
    if ! command -v patchelf >/dev/null 2>&1; then
      echo "Open3D needs libidn2, but patchelf is not installed." >&2
      return 1
    fi
    if ! patchelf --print-needed "$open3d_lib" | grep -qx "libidn2.so.0"; then
      patchelf --add-needed libidn2.so.0 "$open3d_lib"
    fi
  fi
}

build_open3d_from_source() {
  mkdir -p "$(dirname "$OPEN3D_SRC")"
  if [[ ! -d "$OPEN3D_SRC/.git" ]]; then
    git clone --recursive --branch "$OPEN3D_VERSION" https://github.com/isl-org/Open3D.git "$OPEN3D_SRC"
  else
    git -C "$OPEN3D_SRC" fetch --tags
    git -C "$OPEN3D_SRC" checkout "$OPEN3D_VERSION"
    git -C "$OPEN3D_SRC" submodule update --init --recursive
  fi

  cmake -S "$OPEN3D_SRC" -B "$OPEN3D_BUILD" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON_MODULE=ON \
    -DBUILD_SHARED_LIBS=ON \
    -DUSE_SYSTEM_BLAS=ON \
    -DBUILD_GUI=OFF \
    -DBUILD_WEBRTC=OFF \
    -DBUILD_CUDA_MODULE=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_UNIT_TESTS=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DPython3_EXECUTABLE="$VENV_DIR/bin/python"

  cmake --build "$OPEN3D_BUILD" --target install-pip-package -j "$JOBS"
}

verify_reconstruction_tools() {
  export PATH="$VENV_DIR/bin:$PATH"
  "$VENV_DIR/bin/python" - <<'PY'
import shutil
import sys

missing = [name for name in ("ns-process-data", "ns-train", "ns-export", "colmap") if shutil.which(name) is None]
if missing:
    print("missing:", ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
print("reconstruction tools ready")
PY
}

ensure_apt_packages
ensure_venv

if ! pip_install_requirements; then
  if ! open3d_imports; then
    build_open3d_from_source
    patch_open3d_runtime_deps
  fi
  pip_install_requirements
fi

patch_open3d_runtime_deps
if ! open3d_imports; then
  build_open3d_from_source
  patch_open3d_runtime_deps
  open3d_imports
fi

verify_reconstruction_tools
