#!/bin/bash
set -e

VERSION=$(cat ../../../VERSION.txt)
GIT_HASH=$(git log --pretty=format:'%H' -n 1)
echo "Testing commit: $GIT_HASH"
pushd ../../..
docker build -f ci/manylinux/tests/Dockerfile . --build-arg VERSION=${VERSION} --build-arg GIT_HASH=${GIT_HASH}
popd
IMAGE_ID=$(docker images | awk '{print $3}' | awk 'NR==2')
echo "Build image ID $IMAGE_ID"
CONTAINER_ID=$(docker create $IMAGE_ID)
echo "Started container ID $CONTAINER_ID"
mkdir -p ../../../Testing
docker cp $CONTAINER_ID:/Testing ../../../Testing
docker rm -v $CONTAINER_ID
