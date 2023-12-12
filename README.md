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

After that, you can launch GUI after loading URDFs in code, visualize the states of Jade simluation and close the GUI by the below code.

```
gui = nimble.NimbleGUI(world, useBullet=True)
gui.loopStates(states)
gui.stopServing()
```
If you want to use pybullet related functions you can use `gui.p` to access pybullet API, which is the same as `import pybullet as p` in normal setttings. For example, to change camera view, you can call. 
```
gui.p.resetDebugVisualizerCamera(dist, yaw, pitch, target_pos)
```

If you want to record the gui session, simply add the record path in the init argument as below
```
gui = nimble.NimbleGUI(world, useBullet=True, videoLogFile='path/to.mp4')
gui.loopStates(states)
gui.stopServing()
```

If you want to save synthetic camera output in `{depth/rgb/segmentation}_{index}.npy`, pass the save folder path during init argument as below
```
gui = nimble.NimbleGUI(world, useBullet=True, saveCameraPath='path/to/save/foler')
```
If you need to do several loopStates and want to save the camera path together, you need to pass starting index when calling `loopStates`, otherwise it will by default start from `_0.npy`. For exmple, if you want to save the camera files starting with `_1000.npy`
```
gui.loopStates(states, save_start_idx=1000)
```

If you don't want to use the synthetic camera, disable it when init
```
gui = nimble.NimbleGUI(world, useBullet=True, useSyntheticCamera=False)
```

You can always open a new GUI after closing it, but if you just want to reset the gui, you can use the below function.

```
gui.bullet_reset(world)
```

You don't have to call other nimble GUI funcitons like `blockWhileServing`(no effect) or `nativeAPI` but calling these will not cause any problem or having any actual effects so it is easier for you to maintain multiple GUI settings.

Due to current limitaion, please refrain from manually changing your position and rotation when doing `world.loadSkeleton()`. If you met any problem regarding this, please raise an issue.

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

