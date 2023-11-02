#!/usr/bin/env bash
set -e

cd /home/adamas/src/

export VERSION=$(cat ./VERSION.txt)
export PYTHON_VERSION=cp38-cp38

rm -rf dist/*
if [[ -z "${KEEP_BUILD_CACHE}" ]]; then
    rm -rf build/*
fi
# rm -rf wheelhouse/*

# Actually build the code
python3 setup.py sdist bdist_wheel
# Fix dynamic lib. Fixed wheel is outputed at ./wheelhouse
python3 -m auditwheel repair --plat manylinux_2_28_x86_64 dist/nimblephysics-${VERSION}-${PYTHON_VERSION}-linux_x86_64.whl
