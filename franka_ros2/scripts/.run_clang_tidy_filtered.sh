#!/bin/bash
# Wrapper to filter compile_commands.json before running ament_clang_tidy
# Usage: .run_clang_tidy_filtered.sh <build_dir> [ament_clang_tidy args...]

set -e

BUILD_DIR="$1"
shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILTER_SCRIPT="${SCRIPT_DIR}/.filter_compile_commands.py"

# Filter compile_commands.json to only include franka_* packages
echo "[Filter] Filtering compile_commands.json..."
python3 "$FILTER_SCRIPT" "$BUILD_DIR"

# Run ament_clang_tidy
echo "[clang-tidy] Running clang-tidy on filtered database..."
ament_clang_tidy "$BUILD_DIR" "$@"
RESULT=$?

# Restore original compile_commands.json (optional, for cleanliness)
python3 "$FILTER_SCRIPT" "$BUILD_DIR" --restore

exit $RESULT
