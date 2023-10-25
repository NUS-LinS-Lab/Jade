/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>

#include <dart/math/Geometry.hpp>
#include <pybind11/pybind11.h>

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

// TODO(JS): For some reason, passing const reference causes segfault errors.
// Use "pass-by-value" for now.
#define DARTPY_DEFINE_EULAERTOMATRIX(order)                                    \
  m.def(                                                                       \
      "euler" #order "ToMatrix",                                               \
      +[](Eigen::Vector3s angle) -> Eigen::Matrix3s {                          \
        return dart::math::euler##order##ToMatrix(angle);                      \
      },                                                                       \
      ::py::arg("angle"));

#define DARTPY_DEFINE_MATRIXTOEULAER(order)                                    \
  m.def(                                                                       \
      "matrixToEuler" #order,                                                  \
      +[](const Eigen::Matrix3s& R) -> Eigen::Vector3s {                       \
        return dart::math::matrixToEuler##order(R);                            \
      },                                                                       \
      ::py::arg("R"));

namespace py = pybind11;

namespace dart {
namespace python {

void Geometry(py::module& m)
{
  DARTPY_DEFINE_EULAERTOMATRIX(XYX);
  DARTPY_DEFINE_EULAERTOMATRIX(XYZ);
  DARTPY_DEFINE_EULAERTOMATRIX(XZX);
  DARTPY_DEFINE_EULAERTOMATRIX(XZY);
  DARTPY_DEFINE_EULAERTOMATRIX(YXY);
  DARTPY_DEFINE_EULAERTOMATRIX(YXZ);
  DARTPY_DEFINE_EULAERTOMATRIX(YZX);
  DARTPY_DEFINE_EULAERTOMATRIX(YZY);
  DARTPY_DEFINE_EULAERTOMATRIX(ZXY);
  DARTPY_DEFINE_EULAERTOMATRIX(ZYX);
  DARTPY_DEFINE_EULAERTOMATRIX(ZXZ);
  DARTPY_DEFINE_EULAERTOMATRIX(ZYZ);

  DARTPY_DEFINE_MATRIXTOEULAER(XYX);
  DARTPY_DEFINE_MATRIXTOEULAER(XYZ);
  // DARTPY_DEFINE_MATRIXTOEULAER(XZX);
  DARTPY_DEFINE_MATRIXTOEULAER(XZY);
  // DARTPY_DEFINE_MATRIXTOEULAER(YXY);
  DARTPY_DEFINE_MATRIXTOEULAER(YXZ);
  DARTPY_DEFINE_MATRIXTOEULAER(YZX);
  // DARTPY_DEFINE_MATRIXTOEULAER(YZY);
  DARTPY_DEFINE_MATRIXTOEULAER(ZXY);
  DARTPY_DEFINE_MATRIXTOEULAER(ZYX);
  // DARTPY_DEFINE_MATRIXTOEULAER(ZXZ);
  // DARTPY_DEFINE_MATRIXTOEULAER(ZYZ);

  m.def(
      "expMap",
      +[](const Eigen::Vector6s& _S) -> Eigen::Isometry3s {
        return dart::math::expMap(_S);
      },
      ::py::arg("S"));

  m.def(
      "logMap",
      +[](const Eigen::Matrix3s& _S) -> Eigen::Vector3s {
        return dart::math::logMap(_S);
      },
      ::py::arg("S"));

  m.def(
      "AdR",
      +[](const Eigen::Matrix3s& R,
          const Eigen::Vector6s& S) -> Eigen::Vector6s {
        Eigen::Isometry3s T = Eigen::Isometry3s::Identity();
        T.linear() = R;
        return dart::math::AdR(T, S);
      },
      ::py::arg("R"),
      ::py::arg("S"));

  m.def(
      "AdT",
      +[](const Eigen::Matrix3s& R,
          const Eigen::Vector3s& p,
          const Eigen::Vector6s& S) -> Eigen::Vector6s {
        Eigen::Isometry3s T = Eigen::Isometry3s::Identity();
        T.linear() = R;
        T.translation() = p;
        // std::cout << "T: " << std::endl << T.matrix() << std::endl;
        // std::cout << "S: " << std::endl << S << std::endl;
        Eigen::Vector6s result = dart::math::AdT(T, S);
        // std::cout << "AdT(T,S): " << std::endl << result << std::endl;
        return result;
      },
      ::py::arg("R"),
      ::py::arg("p"),
      ::py::arg("S"));

  m.def(
      "rightMultiplyInFreeJointSpace",
      +[](const Eigen::Matrix3s& R,
          const Eigen::Vector3s& p,
          const Eigen::Vector6s& S) -> Eigen::Vector6s {
        Eigen::Isometry3s q = dart::math::expMapDart(S);
        Eigen::Isometry3s T = Eigen::Isometry3s::Identity();
        T.linear() = R;
        T.translation() = p;

        Eigen::Isometry3s result = q * T;
        Eigen::Vector6s vec;
        vec.head<3>() = dart::math::logMap(result.linear());
        vec.tail<3>() = result.translation();
        return vec;
      },
      ::py::arg("R"),
      ::py::arg("p"),
      ::py::arg("S"));

  m.def(
      "leftMultiplyInFreeJointSpace",
      +[](const Eigen::Matrix3s& R,
          const Eigen::Vector3s& p,
          const Eigen::Vector6s& S) -> Eigen::Vector6s {
        Eigen::Isometry3s q = dart::math::expMapDart(S);
        Eigen::Isometry3s T = Eigen::Isometry3s::Identity();
        T.linear() = R;
        T.translation() = p;

        Eigen::Isometry3s result = T * q;
        Eigen::Vector6s vec;
        vec.head<3>() = dart::math::logMap(result.linear());
        vec.tail<3>() = result.translation();
        return vec;
      },
      ::py::arg("R"),
      ::py::arg("p"),
      ::py::arg("S"));

  m.def(
      "expMapJac",
      +[](const Eigen::Vector3s& _expmap) -> Eigen::Matrix3s {
        return dart::math::expMapJac(_expmap);
      },
      ::py::arg("expmap"));

  m.def(
      "expMapRot",
      +[](const Eigen::Vector3s& _expmap) -> Eigen::Matrix3s {
        return dart::math::expMapRot(_expmap);
      },
      ::py::arg("expmap"));

  m.def(
      "expToQuat",
      +[](const Eigen::Vector3s& _v) -> Eigen::Quaternion_s {
        return dart::math::expToQuat(_v);
      },
      ::py::arg("v"));

  m.def(
      "quatToExp",
      +[](const Eigen::Quaternion_s& _q) -> Eigen::Vector3s {
        return dart::math::quatToExp(_q);
      },
      ::py::arg("q"));

  m.def(
      "expAngular",
      +[](const Eigen::Vector3s& _s) -> Eigen::Isometry3s {
        return dart::math::expAngular(_s);
      },
      ::py::arg("s"));

  m.def(
      "verifyRotation",
      +[](const Eigen::Matrix3s& _R) -> bool {
        return dart::math::verifyRotation(_R);
      },
      ::py::arg("R"));

  m.def(
      "verifyTransform",
      +[](const Eigen::Isometry3s& _T) -> bool {
        return dart::math::verifyTransform(_T);
      },
      ::py::arg("T"));

  m.def(
      "transformBy",
      +[](const Eigen::Isometry3s& T,
          const Eigen::Vector3s& p) -> Eigen::Vector3s { return T * p; },
      ::py::arg("T"),
      ::py::arg("p"));
}

} // namespace python
} // namespace dart
