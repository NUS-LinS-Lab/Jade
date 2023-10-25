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

#include <Eigen/Dense>
#include <dart/biomechanics/SkeletonConverter.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace dart {
namespace python {

void SkeletonConverter(py::module& m)
{
  ::py::class_<
      dart::biomechanics::SkeletonConverter,
      std::shared_ptr<dart::biomechanics::SkeletonConverter>>(
      m, "SkeletonConverter")
      .def(
          ::py::init<
              std::shared_ptr<dynamics::Skeleton>,
              std::shared_ptr<dynamics::Skeleton>>(),
          ::py::arg("source"),
          ::py::arg("target"))
      .def(
          "linkJoints",
          &dart::biomechanics::SkeletonConverter::linkJoints,
          ::py::arg("sourceJoint"),
          ::py::arg("targetJoint"))
      .def(
          "rescaleAndPrepTarget",
          &dart::biomechanics::SkeletonConverter::rescaleAndPrepTarget,
          ::py::arg("addFakeMarkers") = 3,
          ::py::arg("weightFakeMarkers") = 0.1,
          ::py::arg("convergenceThreshold") = 1e-15,
          ::py::arg("maxStepCount") = 1000,
          ::py::arg("leastSquaresDamping") = 0.01,
          ::py::arg("lineSearch") = true,
          ::py::arg("logOutput") = false)
      .def(
          "fitSourceToTarget",
          &dart::biomechanics::SkeletonConverter::fitSourceToTarget,
          ::py::arg("convergenceThreshold") = 1e-7,
          ::py::arg("maxStepCount") = 100,
          ::py::arg("leastSquaresDamping") = 0.01,
          ::py::arg("lineSearch") = true,
          ::py::arg("logOutput") = false)
      .def(
          "fitTargetToSource",
          &dart::biomechanics::SkeletonConverter::fitTargetToSource,
          ::py::arg("convergenceThreshold") = 1e-7,
          ::py::arg("maxStepCount") = 100,
          ::py::arg("leastSquaresDamping") = 0.01,
          ::py::arg("lineSearch") = true,
          ::py::arg("logOutput") = false)
      .def(
          "convertMotion",
          &dart::biomechanics::SkeletonConverter::convertMotion,
          ::py::arg("targetMotion"),
          ::py::arg("logProgress") = true,
          ::py::arg("convergenceThreshold") = 1e-7,
          ::py::arg("maxStepCount") = 100,
          ::py::arg("leastSquaresDamping") = 0.01,
          ::py::arg("lineSearch") = true,
          ::py::arg("logIKOutput") = false)
      .def(
          "getSourceJointWorldPositions",
          &dart::biomechanics::SkeletonConverter::getSourceJointWorldPositions)
      .def(
          "getTargetJointWorldPositions",
          &dart::biomechanics::SkeletonConverter::getTargetJointWorldPositions)
      .def(
          "debugToGUI",
          &dart::biomechanics::SkeletonConverter::debugToGUI,
          ::py::arg("gui"),
          ::py::call_guard<py::gil_scoped_release>())
      .def(
          "getSourceJoints",
          &dart::biomechanics::SkeletonConverter::getSourceJoints)
      .def(
          "getTargetJoints",
          &dart::biomechanics::SkeletonConverter::getTargetJoints);
}

} // namespace python
} // namespace dart
