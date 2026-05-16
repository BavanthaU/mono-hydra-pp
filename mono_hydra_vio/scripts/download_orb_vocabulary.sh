#!/usr/bin/env bash
set -euo pipefail

DEFAULT_URL="https://www.dropbox.com/s/lyo0qgbdxn6eg6o/ORBvoc.zip?dl=1"
DEFAULT_SHA256="0e4efc3be84e05c18faec402b88b145c37d94fa41f3eb39999b56ae20f6f2da2"

usage() {
  cat <<'EOF'
Download and install the ORB vocabulary used by Mono Hydra loop closure.

Usage:
  download_orb_vocabulary [options]

Options:
  -d, --destination DIR   Directory where ORBvoc.yml will be written.
  -u, --url URL           Vocabulary archive URL.
  -f, --force             Re-download even if ORBvoc.yml already exists.
      --skip-checksum     Do not verify the archive SHA256.
  -h, --help              Show this help message.

Environment:
  ORB_VOCAB_URL           Overrides the default download URL.
  ORB_VOCAB_SHA256        Overrides the expected archive SHA256.

The default source is the vocabulary archive referenced by the upstream
Kimera-VIO/Mono Hydra ROS 1 installation notes. The archive is not committed to
this repository because it is larger than GitHub's normal file-size guidance.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
destination=""
url="${ORB_VOCAB_URL:-$DEFAULT_URL}"
expected_sha="${ORB_VOCAB_SHA256:-$DEFAULT_SHA256}"
force=0
verify_checksum=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--destination)
      destination="$2"
      shift 2
      ;;
    -u|--url)
      url="$2"
      shift 2
      ;;
    -f|--force)
      force=1
      shift
      ;;
    --skip-checksum)
      verify_checksum=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$destination" ]]; then
  if [[ "$(basename "$script_dir")" == "scripts" ]]; then
    destination="$(cd "$script_dir/../vocabulary" && pwd)"
  else
    prefix="$(cd "$script_dir/../.." && pwd)"
    destination="$prefix/share/mono_hydra_vio/vocabulary"
  fi
fi

mkdir -p "$destination"
output_yml="$destination/ORBvoc.yml"
local_archive="$destination/ORBvoc.zip"

if [[ -f "$output_yml" && "$force" -eq 0 ]]; then
  echo "ORB vocabulary already exists: $output_yml"
  exit 0
fi

tmp_dir="$(mktemp -d)"
cleanup() {
  rm -rf "$tmp_dir"
}
trap cleanup EXIT

archive="$tmp_dir/ORBvoc.zip"
if [[ -f "$local_archive" && "$force" -eq 0 ]]; then
  echo "Using existing local archive: $local_archive"
  cp "$local_archive" "$archive"
else
  echo "Downloading ORB vocabulary archive..."
  if command -v curl >/dev/null 2>&1; then
    curl -L --fail --retry 3 --output "$archive" "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "$archive" "$url"
  else
    echo "Neither curl nor wget is available. Install one of them and retry." >&2
    exit 1
  fi
fi

if [[ "$verify_checksum" -eq 1 && -n "$expected_sha" ]]; then
  actual_sha="$(sha256sum "$archive" | awk '{print $1}')"
  if [[ "$actual_sha" != "$expected_sha" ]]; then
    echo "Checksum mismatch for ORB vocabulary archive." >&2
    echo "Expected: $expected_sha" >&2
    echo "Actual:   $actual_sha" >&2
    echo "Use --skip-checksum only after verifying the archive source." >&2
    exit 1
  fi
fi

extract_dir="$tmp_dir/extract"
mkdir -p "$extract_dir"

if command -v unzip >/dev/null 2>&1; then
  unzip -q "$archive" -d "$extract_dir"
elif command -v cmake >/dev/null 2>&1; then
  (cd "$extract_dir" && cmake -E tar xzf "$archive" >/dev/null)
else
  echo "Neither unzip nor cmake is available to extract ORBvoc.zip." >&2
  exit 1
fi

found_yml="$(find "$extract_dir" -name ORBvoc.yml -type f | head -n 1)"
if [[ -z "$found_yml" ]]; then
  echo "Could not find ORBvoc.yml inside the downloaded archive." >&2
  exit 1
fi

cp "$found_yml" "$output_yml"
echo "Installed ORB vocabulary: $output_yml"
