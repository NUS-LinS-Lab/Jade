# Jade

## Build and install Jade:

See [instruction here](build_container/README.md). It builds a `manylinux_2_28`
wheel. You can install the wheel via `pip`.

## Tutorial:

See [tutorial of nimble](https://nimblephysics.org/docs/). Most APIs are the same.

**Attention**: Object geometry (defined in URDF) should be convex meshes, primitives like sphere and box are forbidden.

## GUI
Nimble API has its own limited GUI. To use bullet-based GUI, you need to install [pybullet](https://pybullet.org/wordpress/). You can install by running

`pip install pybullet`

After that, you can launch GUI after loading URDFs in code by

`gui = nimble.NimbleGUI(world, useBullet=True)`

To visualize your states, simply call

`	gui.loopStates(states)`

You don't have to call other nimble GUI funcitons like `blockWhileServing` or `nativeAPI` but calling these will not cause any problem or having any actual effects.

If you want to use pybullet related functions you can use `gui.p` to access pybullet API, which is the same as `import pybullet as p` in normal setttings. For example, to change camera view, you can call

`gui.p.resetDebugVisualizerCamera(dist, yaw, pitch, target_pos)`

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

