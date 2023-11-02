FROM quay.io/pypa/manylinux_2_28_x86_64

ENV PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:/usr/local/lib64/pkgconfig/"

# Upgrade to the latest CMake version

RUN yum install -y wget

RUN yum install -y openssl-devel

RUN wget https://github.com/Kitware/CMake/releases/download/v3.19.0/cmake-3.19.0.tar.gz
RUN tar -xvzf cmake-3.19.0.tar.gz
RUN pushd cmake-3.19.0 && \
    ./bootstrap && \
    make -j4 && \
    make install
RUN rm -rf cmake-3.19.0
RUN rm cmake-3.19.0.tar.gz

# Install Boost
RUN yum install -y boost-devel

# Install Eigen
RUN curl https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz > eigen.tar.gz && \
    tar -zxf eigen.tar.gz && \
    pushd eigen-3.4.0 && \
    mkdir build && \
    pushd build && \
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install -j14 && \
    popd && \
    popd && \
    rm -rf eigen-3.4.0

# Install CCD
RUN git clone https://github.com/danfis/libccd.git && \
    pushd libccd && \
    mkdir build && \
    pushd build && \
    cmake .. -DENABLE_DOUBLE_PRECISION=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install -j14 && \
    popd && \
    popd && \
    rm -rf libccd

# Install ASSIMP
RUN git clone https://github.com/assimp/assimp.git && \
    pushd assimp && \
    git checkout v5.0.1 && \
    mkdir build && \
    pushd build && \
    # cmake .. -DASSIMP_BUILD_ZLIB=ON -DASSIMP_BUILD_TESTS=ON -DASSIMP_BUILD_ASSIMP_TOOLS=OFF -DCMAKE_BUILD_TYPE=Debug && \
    cmake .. -DASSIMP_BUILD_ZLIB=ON -DASSIMP_BUILD_TESTS=ON -DASSIMP_BUILD_ASSIMP_TOOLS=OFF && \
    make install -j10 && \
    popd && \
    popd && \
    rm -rf assimp

# Install LAPACK
RUN yum install -y lapack-devel

# Install MUMPS
# Cannot build in parallel
RUN export FC=$(which gfortran)
RUN git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git && \
    pushd ThirdParty-Mumps && \
    git checkout 5fcdbba365286dddbe7df0dfec38c63610058f40 &&\
    ./get.Mumps && \
    ./configure ADD_FCFLAGS=-fallow-argument-mismatch && \
    make all-am && \
    make install && \
    popd && \
    rm -rf ThirdParty-Mumps

# Install IPOPT
RUN git clone https://github.com/coin-or/Ipopt.git && \
    pushd Ipopt &&\
    git checkout 332132a4ab18e53153d85fd637880babf1d7ff03 &&\
    ./configure --with-mumps && \
    make -j14 && \
    make install && \
    popd && \
    rm -rf Ipopt && \
    ln -s /usr/local/include/coin-or /usr/local/include/coin

# Install tinyxml2
RUN git clone https://github.com/leethomason/tinyxml2.git && \
    pushd tinyxml2 && \
    git checkout bf15233ad88390461f6ab0dbcf046cce643c5fcb &&\
    mkdir build && \
    pushd build && \
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install -j10 && \
    popd && \
    popd && \
    rm -rf tinyxml2

# Install tinyxml1
RUN git clone https://github.com/robotology-dependencies/tinyxml.git && \
    pushd tinyxml && \
    git checkout f83a543bb42d3c75be3b7c1f9f74e1f8d5e6f86a &&\
    mkdir build && \
    pushd build && \
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install -j10 && \
    popd && \
    popd && \
    rm -rf tinyxml

# Install urdfdom_headers
RUN git clone https://github.com/ros/urdfdom_headers.git && \
    pushd urdfdom_headers && \
    mkdir build && \
    pushd build && \
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install -j10 && \
    popd && \
    popd && \
    rm -rf urdfdom_headers

# Install console_bridge
RUN git clone https://github.com/ros/console_bridge.git && \
    pushd console_bridge && \
    mkdir build && \
    pushd build && \
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install -j10 && \
    popd && \
    popd && \
    rm -rf console_bridge

# Install urdfdom
RUN git clone https://github.com/ros/urdfdom.git && \
    pushd urdfdom && \
    mkdir build && \
    pushd build && \
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install -j10 && \
    popd && \
    popd && \
    rm -rf urdfdom

# Install PerfUtils
RUN git clone https://github.com/PlatformLab/PerfUtils.git && \
    pushd PerfUtils && \
    sed -i 's/3.11/3.6.1/g' CMakeLists.txt && \
    sed -i '94,$d' CMakeLists.txt && \
    sed -i '30,33d' CMakeLists.txt && \
    sed -i '36i\ \ \ \ CXX_STANDARD 11' CMakeLists.txt && \
    sed -i '36i\ \ \ \ CXX_STANDARD_REQUIRED YES' CMakeLists.txt && \
    sed -i '36i\ \ \ \ CXX_EXTENSIONS NO' CMakeLists.txt && \
    mkdir build && \
    pushd build && \
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON && \
    make install && \
    popd && \
    popd && \
    rm -rf PerfUtils

ENV PROTOBUF_VERSION="3.14.0"

# RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOBUF_VERSION}/protobuf-all-${PROTOBUF_VERSION}.tar.gz && \
#     tar -xvzf protobuf-all-${PROTOBUF_VERSION}.tar.gz && \
#     rm protobuf-all-${PROTOBUF_VERSION}.tar.gz && \
#     pushd protobuf-${PROTOBUF_VERSION} && \
#     ./configure && \
#     make -j16 && \
#     make check -j16 && \
#     make install && \
#     popd && \
#     rm -rf protobuf-${PROTOBUF_VERSION}

RUN git clone --recurse-submodules -b v1.42.0 https://github.com/grpc/grpc
RUN pushd grpc && \
    pushd third_party/protobuf && \
    git checkout v${PROTOBUF_VERSION} && \
    popd && \
    mkdir -p cmake/build && \
    pushd cmake/build && \
    cmake -DgRPC_INSTALL=ON \
          -DgRPC_BUILD_TESTS=OFF \
          -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
          ../.. && \
    make -j3 && \
    make install && \
    popd && \
    popd
RUN rm -rf grpc

# Install Google benchmark
RUN git clone https://github.com/google/benchmark.git
RUN git clone https://github.com/google/googletest.git benchmark/googletest
RUN pushd benchmark && \
    mkdir build && \
    pushd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && \
    make install && \
    popd && \
    popd
RUN rm -rf benchmark

# Install ezc3d
RUN git clone https://github.com/pyomeca/ezc3d.git
RUN pushd ezc3d && \
    git checkout Release_1.4.7 && \
    mkdir build && \
    pushd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && \
    make install && \
    popd && \
    popd
RUN rm -rf ezc3d

# RUN ls /usr/local/lib64 | grep assimp
# RUN ls /usr/local/lib | grep assimp
# RUN rm -rf /usr/local/lib64/libassimp.so
# RUN rm -rf /usr/local/lib64/libassimp.5.so
# RUN rm -rf /usr/local/lib64/libassimp.5.0.1.so
RUN protoc --version

# Install spdlog
RUN git clone https://github.com/gabime/spdlog.git
RUN pushd spdlog &&\
    mkdir build &&\
    pushd build &&\
    cmake .. &&\
    make -j &&\
    make install &&\
    popd &&\
    popd
RUN rm -rf spdlog

# This is allowed to be empty string, but if it's not it must be prefixed by
ENV VERSION="0.7.7"

# Build Python 3.8

ENV PYTHON="/opt/python/cp38-cp38/bin/python3.8"
ENV PATH="/opt/python/cp38-cp38/bin/:${PATH}"
ENV PYTHON_VERSION="cp38-cp38"
ENV PYTHON_INCLUDE="/opt/python/cp38-cp38/include/python3.8/"
ENV PYTHON_LIB="/opt/python/cp38-cp38/lib/python3.8"
ENV PYTHON_VERSION_NUMBER="3.8"

# Install pybind11
ENV CPATH="${PYTHON_INCLUDE}"
RUN git clone https://github.com/pybind/pybind11.git && \
    pushd pybind11 && \
    mkdir build && \
    pushd build && \
    cmake .. && \
    make install -j10
# Install pytest
RUN ${PYTHON} -m pip install pytest
RUN ${PYTHON} -m pip install auditwheel
# RUN git clone https://github.com/keenon/nimblephysics.git
# RUN cd nimblephysics && \
#     git checkout ${GIT_HASH} && \
#     cat setup.py && \
#     ${PYTHON} setup.py sdist bdist_wheel && \
#     ${PYTHON} -m auditwheel repair dist/nimblephysics-${VERSION}-${PYTHON_VERSION}-linux_x86_64.whl
# RUN mv /nimblephysics/wheelhouse/nimblephysics-${VERSION}-${PYTHON_VERSION}-manylinux_2_17_x86_64.manylinux2014_x86_64.whl /wheelhouse

ARG uid
ARG gid
ARG user=adamas
ARG group=adamas
ARG home=/home/${user}
RUN mkdir -p /etc/sudoers.d \
    && groupadd -g ${gid} ${group} \
    && useradd -d ${home} -u ${uid} -g ${gid} -m -s /bin/bash ${user}
USER ${user}
WORKDIR ${home}
ENV HOME ${home}
COPY build_wheel.sh /usr/local/bin/build_wheel
