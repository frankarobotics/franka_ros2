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

# Ensure .clang-tidy config is accessible from workspace root and build directory
# clang-tidy searches up the directory tree for .clang-tidy starting from the working directory
WORKSPACE_ROOT=$(cd "$BUILD_DIR/../.." && pwd)

# Create symlink at workspace root if it doesn't exist (use absolute path for symlink target)
if [ ! -f "${WORKSPACE_ROOT}/.clang-tidy" ]; then
  if [ -f "${WORKSPACE_ROOT}/src/.clang-tidy" ]; then
    ln -s "${WORKSPACE_ROOT}/src/.clang-tidy" "${WORKSPACE_ROOT}/.clang-tidy" 2>/dev/null || true
  fi
fi

# Also create symlink at build parent directory for compatibility
if [ ! -f "$BUILD_DIR/../.clang-tidy" ]; then
  if [ -f "${WORKSPACE_ROOT}/.clang-tidy" ]; then
    ln -s "${WORKSPACE_ROOT}/.clang-tidy" "$BUILD_DIR/../.clang-tidy" 2>/dev/null || true
  elif [ -f "${WORKSPACE_ROOT}/src/.clang-tidy" ]; then
    ln -s "${WORKSPACE_ROOT}/src/.clang-tidy" "$BUILD_DIR/../.clang-tidy" 2>/dev/null || true
  fi
fi

# Run ament_clang_tidy
echo "[clang-tidy] Running clang-tidy on filtered database..."
ament_clang_tidy "$BUILD_DIR" "$@"
RESULT=$?

# Restore original compile_commands.json (optional, for cleanliness)
python3 "$FILTER_SCRIPT" "$BUILD_DIR" --restore

exit $RESULT
