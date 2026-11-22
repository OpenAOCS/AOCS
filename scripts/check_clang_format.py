#!/usr/bin/env python3

import os
import subprocess
import sys
from pathlib import Path


CPP_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx", ".ipp", ".inl"}
VENDORED_PREFIXES = ("shared/external/",)


def main() -> int:
    root = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    paths = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
    ).stdout.split(b"\0")

    files = [
        os.fsdecode(path)
        for path in paths
        if path
        and Path(os.fsdecode(path)).suffix.lower() in CPP_EXTENSIONS
        and not os.fsdecode(path).startswith(VENDORED_PREFIXES)
        and (Path(root) / os.fsdecode(path)).is_file()
    ]
    if not files:
        print("No first-party C++ files found to format.")
        return 0

    print(f"Checking clang-format for {len(files)} C++ files.")
    clang_format = os.environ.get("CLANG_FORMAT", "clang-format")
    return subprocess.run(
        [clang_format, "--dry-run", "--Werror", *files],
        cwd=root,
        check=False,
    ).returncode


if __name__ == "__main__":
    sys.exit(main())
