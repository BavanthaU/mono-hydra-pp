#!/usr/bin/env bash
# Install only the optional PTC refinement core; no depth model is downloaded.
set -euo pipefail
ptc_revision=6f071370396bf2fd50b4bd6815f0a5012b06e119
ptc_package_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ptc_patch="$ptc_package_dir/third_party/ptc_depth.patch"
ptc_patch_hash=$(sha256sum "$ptc_patch" | cut -c1-12)
ptc_python_tag=$(python3 -c 'import sys, platform; print("py%d%d-%s" % (*sys.version_info[:2], platform.machine()))')
ptc_base=${1:-${XDG_DATA_HOME:-$HOME/.local/share}/mono_hydra/ptc-depth}
ptc_install="$ptc_base/${ptc_revision:0:12}-$ptc_patch_hash-$ptc_python_tag"
if [[ ! -f "$ptc_install/complete" ]]; then
  if [[ -e "$ptc_install" ]]; then
    echo "Incomplete installation exists: $ptc_install. Use another install base to retain it." >&2
    exit 1
  fi
  mkdir -p "$ptc_install"
  git clone https://github.com/ajou-arrl/ptc-depth.git "$ptc_install/source"
  git -C "$ptc_install/source" checkout --detach "$ptc_revision"
  git -C "$ptc_install/source" apply --check "$ptc_patch"
  git -C "$ptc_install/source" apply "$ptc_patch"
  CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2} \
    python3 -m pip install --no-deps --target "$ptc_install/python" "$ptc_install/source"
  # Exact rotated-camera correspondence must have zero Sampson residual.
  read -r -a ptc_cv_flags <<< "$(pkg-config --cflags opencv4)"
  c++ -O2 -std=c++17 -I"$ptc_install/source/cpp/include" -I/usr/include/eigen3 \
    "${ptc_cv_flags[@]}" "$ptc_package_dir/third_party/ptc_triangulation_regression.cpp" \
    "$ptc_install/source/cpp/src/triangulation.cpp" -lopencv_core -o "$ptc_install/test_triangulation"
  "$ptc_install/test_triangulation"
  printf 'export PTC_DEPTH_PYTHONPATH=%q\n' "$ptc_install/python" > "$ptc_install/env.sh"
  touch "$ptc_install/complete"
fi
printf 'PTC core installed. Before launching, run:\nsource %q\n' "$ptc_install/env.sh"
