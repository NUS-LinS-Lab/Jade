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

#include <dart/neural/MappedBackpropSnapshot.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void MappedBackpropSnapshot(py::module& m)
{
  ::py::class_<
      dart::neural::MappedBackpropSnapshot,
      std::shared_ptr<dart::neural::MappedBackpropSnapshot>>(
      m, "MappedBackpropSnapshot")
      .def(
          "backprop",
          &dart::neural::MappedBackpropSnapshot::backprop,
          ::py::arg("world"),
          ::py::arg("thisTimestepLoss"),
          ::py::arg("nextTimestepLosses"),
          ::py::arg("perfLog") = nullptr,
          ::py::arg("exploreAlternateStrategies") = false)
      .def("getMappings", &dart::neural::MappedBackpropSnapshot::getMappings)
      .def(
          "getVelVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getVelVelJacobian,
          ::py::arg("world"),
          ::py::arg("perfLog") = nullptr)
      .def(
          "getControlForceVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getControlForceVelJacobian,
          ::py::arg("world"),
          ::py::arg("perfLog") = nullptr)
      .def(
          "getPosPosJacobian",
          &dart::neural::MappedBackpropSnapshot::getPosPosJacobian,
          ::py::arg("world"),
          ::py::arg("perfLog") = nullptr)
      .def(
          "getVelPosJacobian",
          &dart::neural::MappedBackpropSnapshot::getVelPosJacobian,
          ::py::arg("world"),
          ::py::arg("perfLog") = nullptr)
      .def(
          "getPosVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getPosVelJacobian,
          ::py::arg("world"),
          ::py::arg("perfLog") = nullptr)
      .def(
          "getMassVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getMassVelJacobian,
          ::py::arg("world"),
          ::py::arg("perfLog") = nullptr)
      .def(
          "getVelMappedVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getVelMappedVelJacobian,
          ::py::arg("world"),
          ::py::arg("mapAfter") = "identity",
          ::py::arg("perfLog") = nullptr)
      .def(
          "getControlForceMappedVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getControlForceMappedVelJacobian,
          ::py::arg("world"),
          ::py::arg("mapAfter") = "identity",
          ::py::arg("perfLog") = nullptr)
      .def(
          "getPosMappedPosJacobian",
          &dart::neural::MappedBackpropSnapshot::getPosMappedPosJacobian,
          ::py::arg("world"),
          ::py::arg("mapAfter") = "identity",
          ::py::arg("perfLog") = nullptr)
      .def(
          "getVelMappedPosJacobian",
          &dart::neural::MappedBackpropSnapshot::getVelMappedPosJacobian,
          ::py::arg("world"),
          ::py::arg("mapAfter") = "identity",
          ::py::arg("perfLog") = nullptr)
      .def(
          "getPosMappedVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getPosMappedVelJacobian,
          ::py::arg("world"),
          ::py::arg("mapAfter") = "identity",
          ::py::arg("perfLog") = nullptr)
      .def(
          "getMassMappedVelJacobian",
          &dart::neural::MappedBackpropSnapshot::getMassMappedVelJacobian,
          ::py::arg("world"),
          ::py::arg("mapAfter") = "identity",
          ::py::arg("perfLog") = nullptr)
      .def(
          "getPreStepPosition",
          &dart::neural::MappedBackpropSnapshot::getPreStepPosition,
          ::py::arg("mapping") = "identity")
      .def(
          "getPreStepVelocity",
          &dart::neural::MappedBackpropSnapshot::getPreStepVelocity,
          ::py::arg("mapping") = "identity")
      .def(
          "getPreStepTorques",
          &dart::neural::MappedBackpropSnapshot::getPreStepTorques,
          ::py::arg("mapping") = "identity")
      .def(
          "getPostStepPosition",
          &dart::neural::MappedBackpropSnapshot::getPostStepPosition,
          ::py::arg("mapping") = "identity")
      .def(
          "getPostStepVelocity",
          &dart::neural::MappedBackpropSnapshot::getPostStepVelocity,
          ::py::arg("mapping") = "identity");
}

} // namespace python
} // namespace dart
