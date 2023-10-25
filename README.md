# Jade

## Build and install Jade:

See [instruction here](build_container/README.md). It builds a `manylinux_2_28`
wheel. You can install the wheel via `pip`.

## Tutorial:

See [tutorial of nimble](https://nimblephysics.org/docs/). Most APIs are the same.

**Attention**: Object geometry (defined in URDF) should be convex meshes, primitives like sphere and box are forbidden.

## Debug Mode:

Set environment variable `SPDLOG_LEVEL`. e.g., `SPDLOG_LEVEL=debug python myscripyt.py`.


