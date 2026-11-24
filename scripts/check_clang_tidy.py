#!/usr/bin/env python3

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

PROFILES = (
    (
        "GNC",
        "gnc_lib/.clang-tidy",
        r"^(?!.*/eigen/).*/gnc_lib/.*",
        r"^(?!.*/eigen/).*/gnc_lib/.*",
    ),
    (
        "simulation",
        "sim/.clang-tidy",
        r"^(?!.*/eigen/).*/(sim/.*|shared/bindings/astro_bindings\.cpp)",
        r"^(?!.*/eigen/).*/(sim|shared)/.*",
    ),
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

    parser = argparse.ArgumentParser(
        description="Run component clang-tidy profiles across the compile database."
    )
    parser.add_argument("--build-dir", type=Path, default=root / "build")
    parser.add_argument("--jobs", type=int, default=None)
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Apply clang-tidy fixes in place (forces -j 1 unless --jobs is set).",
    )
    args = parser.parse_args()

    jobs = args.jobs if args.jobs is not None else (1 if args.fix else 4)
    if jobs < 1:
        parser.error("--jobs must be positive")

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
        print(
            "run-clang-tidy was not found; install it with clang-tidy/clang-tools.",
            file=sys.stderr,
        )
        return 2

    failed = False
    for profile, config, source_filter, header_filter in PROFILES:
        config_path = root / config
        command = [
            runner,
            "-p",
            str(build_dir),
            "-config-file",
            str(config_path),
            "-source-filter",
            source_filter,
            "-header-filter",
            header_filter,
            "-j",
            str(jobs),
            "-quiet",
        ]
        if args.fix:
            command.append("-fix")

        print(f"Running {profile} clang-tidy profile.", flush=True)
        if subprocess.run(command, cwd=root, check=False).returncode:
            failed = True

    return int(failed)


if __name__ == "__main__":
    sys.exit(main())