#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
from pathlib import Path

CPP_EXTENSIONS = {
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".h",
    ".hh",
    ".hpp",
    ".hxx",
    ".ipp",
    ".inl",
}
VENDORED_PREFIXES = ("shared/external/",)
IGNORED_FILES = {
    "sim/include/common/xoshiro.hpp",
}


def main() -> int:
    root = Path(
        subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    )

    parser = argparse.ArgumentParser(
        description="Check or apply clang-format across first-party C/C++ files."
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Format files in-place instead of failing on formatting differences.",
    )
    args = parser.parse_args()

    raw_paths = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
    ).stdout.split(b"\0")

    files: list[str] = []
    for raw in raw_paths:
        if not raw:
            continue
        rel = Path(os.fsdecode(raw)).as_posix()
        if rel in IGNORED_FILES:
            continue
        if Path(rel).suffix.lower() not in CPP_EXTENSIONS:
            continue
        if rel.startswith(VENDORED_PREFIXES):
            continue
        if not (root / rel).is_file():
            continue
        files.append(rel)

    if not files:
        print("No first-party C++ files found.")
        return 0

    clang_format = os.environ.get("CLANG_FORMAT", "clang-format")
    if args.fix:
        print(f"Formatting {len(files)} C++ files in-place.")
        cmd = [clang_format, "-i", *files]
    else:
        print(f"Checking clang-format for {len(files)} C++ files.")
        cmd = [clang_format, "--dry-run", "--Werror", *files]

    return subprocess.run(cmd, cwd=root, check=False).returncode


if __name__ == "__main__":
    sys.exit(main())