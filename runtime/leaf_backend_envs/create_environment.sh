#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 4 || $# -gt 6 ]]; then
  echo "usage: $0 PYTHON EXPECTED_PYTHON ENVIRONMENT REQUIREMENTS [SOURCE_REPO EXPECTED_COMMIT]" >&2
  exit 2
fi

python_executable="$1"
expected_python="$2"
environment="$3"
requirements="$4"
framework_source="${5:-}"
expected_commit="${6:-}"

python_version="$($python_executable -c 'import platform; print(platform.python_version())')"
if [[ "$python_version" != "$expected_python" ]]; then
  echo "refusing unlocked Python $python_version; expected $expected_python" >&2
  exit 1
fi

if [[ ! -f "$requirements" ]]; then
  echo "requirements lock does not exist: $requirements" >&2
  exit 1
fi

if [[ -n "$framework_source" || -n "$expected_commit" ]]; then
  if [[ -z "$framework_source" || -z "$expected_commit" ]]; then
    echo "SOURCE_REPO and EXPECTED_COMMIT must be supplied together" >&2
    exit 2
  fi
  commit="$(git -C "$framework_source" rev-parse HEAD)"
  if [[ "$commit" != "$expected_commit" ]]; then
    echo "refusing framework checkout at unlocked commit $commit" >&2
    exit 1
  fi
fi

if [[ -e "$environment" ]]; then
  echo "refusing to overwrite existing environment: $environment" >&2
  exit 1
fi

"$python_executable" -m venv "$environment"
"$environment/bin/python" -m pip install --upgrade pip
"$environment/bin/python" -m pip install -r "$requirements"

if [[ -n "$framework_source" ]]; then
  printf '%s\n' "$framework_source" > "$environment/FRAMEWORK_SOURCE_PATH"
fi

echo "created isolated environment at $environment"
