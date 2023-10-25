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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/neural/BackpropSnapshot.hpp>
#include <dart/neural/MappedBackpropSnapshot.hpp>
#include <dart/neural/Mapping.hpp>
#include <dart/neural/NeuralUtils.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace dart {
namespace python {

void NeuralUtils(py::module& m)
{
  ::py::class_<dart::neural::LossGradient>(m, "LossGradient")
      .def(::py::init<>())
      .def_readwrite(
          "lossWrtPosition", &dart::neural::LossGradient::lossWrtPosition)
      .def_readwrite(
          "lossWrtVelocity", &dart::neural::LossGradient::lossWrtVelocity)
      .def_readwrite(
          "lossWrtTorque", &dart::neural::LossGradient::lossWrtTorque);

  ::py::class_<dart::neural::LossGradientHighLevelAPI>(
      m, "LossGradientHighLevelAPI")
      .def(::py::init<>())
      .def_readwrite(
          "lossWrtState", &dart::neural::LossGradientHighLevelAPI::lossWrtState)
      .def_readwrite(
          "lossWrtAction",
          &dart::neural::LossGradientHighLevelAPI::lossWrtAction)
      .def_readwrite(
          "lossWrtMass", &dart::neural::LossGradientHighLevelAPI::lossWrtMass);

  ::py::class_<dart::neural::KnotJacobian>(m, "KnotJacobian")
      .def(::py::init<>())
      .def_readwrite(
          "knotPosEndPos", &dart::neural::KnotJacobian::knotPosEndPos)
      .def_readwrite(
          "knotVelEndPos", &dart::neural::KnotJacobian::knotVelEndPos)
      .def_readwrite(
          "knotPosEndVel", &dart::neural::KnotJacobian::knotPosEndVel)
      .def_readwrite(
          "knotVelEndVel", &dart::neural::KnotJacobian::knotVelEndVel)
      .def_readwrite(
          "torquesEndPos", &dart::neural::KnotJacobian::torquesEndPos)
      .def_readwrite(
          "torquesEndVel", &dart::neural::KnotJacobian::torquesEndVel);

  ::py::enum_<dart::neural::ConvertToSpace>(m, "ConvertToSpace")
      .value("COM_POS", dart::neural::ConvertToSpace::COM_POS)
      .value("COM_VEL_LINEAR", dart::neural::ConvertToSpace::COM_VEL_LINEAR)
      .value("COM_VEL_SPATIAL", dart::neural::ConvertToSpace::COM_VEL_SPATIAL)
      .value("VEL_LINEAR", dart::neural::ConvertToSpace::VEL_LINEAR)
      .value("VEL_SPATIAL", dart::neural::ConvertToSpace::VEL_SPATIAL)
      .value("POS_LINEAR", dart::neural::ConvertToSpace::POS_LINEAR)
      .value("POS_SPATIAL", dart::neural::ConvertToSpace::POS_SPATIAL)
      .export_values();

  m.def(
      "forwardPass",
      &dart::neural::forwardPass,
      ::py::arg("world"),
      ::py::arg("idempotent") = false);
  m.def(
      "mappedForwardPass",
      &dart::neural::mappedForwardPass,
      ::py::arg("world"),
      ::py::arg("mappings"),
      ::py::arg("idempotent") = false);
  m.def(
      "convertJointSpaceToWorldSpace",
      &dart::neural::convertJointSpaceToWorldSpace,
      ::py::arg("world"),
      ::py::arg("in"),
      ::py::arg("nodes"),
      ::py::arg("space"),
      ::py::arg("backprop") = false,
      ::py::arg("useIK") = true);
}

} // namespace python
} // namespace dart
