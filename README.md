# Jade

## Build and install Jade:

See [instruction here](build_container/README.md). It builds a `manylinux_2_28`
wheel. You can install the wheel via `pip`.

## Tutorial:

See [tutorial of nimble](https://nimblephysics.org/docs/). Most APIs are the same.

**Attention**: Object geometry (defined in URDF) should be convex meshes, primitives like sphere and box are forbidden.

## Debug Mode:

Set environment variable `SPDLOG_LEVEL`. e.g., `SPDLOG_LEVEL=debug python myscripyt.py`.

## Citation

If you find our work useful, please consider citing:
```
@article{gang2023jade,
  title={Jade: A Differentiable Physics Engine for Articulated Rigid Bodies with Intersection-Free Frictional Contact},
  author={Yang, Gang and Luo, Siyuan and Shao, Lin},
  journal={arXiv preprint arXiv:2309.04710},
  year={2023}
}
```

