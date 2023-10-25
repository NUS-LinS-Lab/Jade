# Developing Jade

We use [docker](https://www.docker.com/) to setup the development environment for
`Jade`. We need to build the docker image once; after that, we can iterate:
1. Develop `Jade` feature.
1. Build a python wheel of `Jade`.
1. Install the newly built wheel in a python envrionment (conda, virtualenv, e.t.c.).
1. Test the new feature.

You need to [install
docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
(we assume that you are using Ubuntu). Follow the instruction. After the
installation, you should be able to run `sudo docker run hello-world`.

Then you need to do the [docker post installation
config](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
so that you can invoke `docker` command without elevating to `root`. The key steps are:

``` sh
sudo groupadd docker
sudo usermod -aG docker $USER
```

To be sure, reboot after the post-installation config; after rebooting, you
should be able to run `docker run hello-world` without `sudo`.

You need to ensure your `glibc>=2.28`. To check, run

``` sh
ldd --version
```

## Build the docker image

You only need to execute this once. In this directory, run

``` sh
python bdocker_image.py
```

Alternatively, get a pre-built image

``` sh
docker pull linfenglee/adamas-env:latest
docker tag linfenglee/adamas-env:latest adamas-env:latest
```

## Build the python wheel

In this directory, run

``` bash
python bwheel.py
```

The output wheel should be located at
`<repo root>/wheelhouse/nimblephysics-0.7.7-cp38-cp38-manylinux_2_28_x86_64.whl`

### Reuse CMake build tree

The `bwheel.py` script has one opton `--cache`. When supplied, it will not clean
the previous `CMake` cache. This speeds up the build.

``` bash
python bwheel.py --cache
```

## Install the python wheel

In the this directory, with your own python environment activated, run

``` sh
pip uninstall -y nimblephysics && pip install ../wheelhouse/nimblephysics-0.7.7-cp38-cp38-manylinux_2_28_x86_64.whl
```

The flag `-I` removes existing installed `nimblephysics`.

You can install other dependencies (e.g., `torch`) as usual.
