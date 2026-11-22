#!/usr/bin/env python3

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


PROFILES = (
    ("GNC", "gnc_lib/.clang-tidy", r".*/gnc_lib/.*"),
    ("simulation", "sim/.clang-tidy", r".*/(sim/.*|shared/bindings/astro_bindings\.cpp)"),
)


def main() -> int:
    root = Path(
        subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    )
    parser = argparse.ArgumentParser(description="Run component clang-tidy profiles across the compile database.")
    parser.add_argument("--build-dir", type=Path, default=root / "build")
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()

    build_dir = args.build_dir if args.build_dir.is_absolute() else root / args.build_dir
    if not (build_dir / "compile_commands.json").is_file():
        print(
            f"No compile_commands.json in {build_dir}. Configure with "
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON first.",
            file=sys.stderr,
        )
        return 2

    runner = shutil.which("run-clang-tidy") or shutil.which("run-clang-tidy.py")
    if not runner:
        print("run-clang-tidy was not found; install it with clang-tidy/clang-tools.", file=sys.stderr)
        return 2
    if args.jobs < 1:
        parser.error("--jobs must be positive")

    failed = False
    for profile, config, source_filter in PROFILES:
        config_path = root / config
        command = [
            runner,
            "-p",
            str(build_dir),
            "-config-file",
            str(config_path),
            "-source-filter",
            source_filter,
            "-j",
            str(args.jobs),
            "-quiet",
        ]
        print(f"Running {profile} clang-tidy profile.", flush=True)
        if subprocess.run(command, cwd=root, check=False).returncode:
            failed = True

    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
