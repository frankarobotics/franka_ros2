#!/usr/bin/env python3
"""Filter compile_commands.json to only include franka_* packages."""

import json
import sys
import os
from pathlib import Path


def filter_compile_commands(input_file, output_file, pattern="franka_"):
    """
    Filter compile_commands.json to only include packages matching pattern.
    
    Reduces clang-tidy analysis scope by filtering the compilation database to only
    include entries for Franka packages, excluding third-party libraries (gtest, ROS2, etc).
    This significantly accelerates clang-tidy execution.
    
    Args:
        input_file (str): Path to compile_commands.json to filter
        output_file (str): Path to write filtered database (can be same as input_file)
        pattern (str): Package name pattern to include (default: "franka_")
    """
    with open(input_file, "r") as f:
        commands = json.load(f)

    # Filter entries to only include franka_* packages
    filtered = []
    for entry in commands:
        file_path = entry.get("file", "")
        # Only include files that contain "franka_" in their path (our code)
        # Exclude system paths and third-party code
        if f"/src/{pattern}" in file_path or f"/{pattern}" in file_path:
            filtered.append(entry)

    # Backup original and replace with filtered version
    backup_file = input_file.replace(".json", "_original.json")
    if not os.path.exists(backup_file):
        import shutil
        shutil.copy(input_file, backup_file)
    
    with open(input_file, "w") as f:
        json.dump(filtered, f, indent=2)

    if output_file != input_file:
        with open(output_file, "w") as f:
            json.dump(filtered, f, indent=2)

    print(f"Filtered {len(commands)} entries to {len(filtered)} entries")
    if output_file != input_file:
        print(f"Written to {output_file}")


def restore_compile_commands(build_dir):
    """
    Restore compile_commands.json from backup if it exists.
    
    Used for cleanup after testing or to revert to full analysis scope.
    Restores the original unfiltered compilation database for cases where
    full clang-tidy analysis is needed.
    
    Args:
        build_dir (str): Build directory containing compile_commands.json
    """
    input_file = os.path.join(build_dir, "compile_commands.json")
    backup_file = input_file.replace(".json", "_original.json")
    
    if os.path.exists(backup_file):
        import shutil
        shutil.copy(backup_file, input_file)
        print(f"Restored {input_file} from backup")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: .filter_compile_commands.py <build_dir> [--restore]")
        sys.exit(1)

    build_dir = sys.argv[1]
    restore_mode = "--restore" in sys.argv
    input_file = os.path.join(build_dir, "compile_commands.json")

    if restore_mode:
        restore_compile_commands(build_dir)
        sys.exit(0)

    if not os.path.exists(input_file):
        print(f"Error: {input_file} not found")
        sys.exit(1)

    # Replace compile_commands.json in place
    filter_compile_commands(input_file, input_file)
