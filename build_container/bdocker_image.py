#!/usr/bin/env python
"""Build the container image for building Adamas."""
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

    return p.parse_args()


if __name__ == "__main__":
    print("building docker container . . . ")
    args = cmdline_args()
    if args.podman:
        subprocess.run(
            [
                "podman",
                "build",
                "-f",
                "./Containerfile",
                "--tag",
                "adamas-env-root:latest",
                ".",
            ]
        )
    else:
        subprocess.run(
            [
                "docker",
                "build",
                "-f",
                "./Dockerfile",
                "--tag",
                "adamas-env:latest",
                "--build-arg",
                f"uid={os.getuid()}",
                "--build-arg",
                f"gid={os.getgid()}",
                ".",
            ]
        )
