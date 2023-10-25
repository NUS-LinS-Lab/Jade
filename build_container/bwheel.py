#!/usr/bin/env python
"""Build the python wheel of Adamas."""
import subprocess
import argparse
import os


def cmdline_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "--podman", action="store_true", help="Use podman instead of docker."
    )
    p.add_argument(
        "--cache", action="store_true", help="Keep the last CMake build cache."
    )

    return p.parse_args()


if __name__ == "__main__":
    args = cmdline_args()
    container_cmd = "podman" if args.podman else "docker"
    image_name = (
        "localhost/adamas-env-root:latest" if args.podman else "adamas-env:latest"
    )
    repo_dir = os.path.realpath("./..")
    print("building adamas wheel . . . ")
    subprocess_cmds = [
        container_cmd,
        "run",
        "-it",
        "--rm",
        "-v",
        f"{repo_dir}:/home/adamas/src",
    ]

    if args.cache:
        subprocess_cmds.extend(["--env", "KEEP_BUILD_CACHE=1"])
    subprocess_cmds.extend([image_name, "build_wheel"])
    subprocess.run(subprocess_cmds, cwd=repo_dir)
