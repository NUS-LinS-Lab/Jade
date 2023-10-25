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

#include <memory>

#include <dart/biomechanics/OpenSimParser.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dart/biomechanics/C3DLoader.hpp"

namespace py = pybind11;

namespace dart {
namespace python {

void OpenSimParser(py::module& m)
{
  ::py::class_<dart::biomechanics::OpenSimFile>(m, "OpenSimFile")
      .def(
          ::py::init<
              std::shared_ptr<dynamics::Skeleton>,
              std::map<
                  std::string,
                  std::pair<dynamics::BodyNode*, Eigen::Vector3s>>>(),
          ::py::arg("skeleton"),
          ::py::arg("markers"))
      .def_readwrite("skeleton", &dart::biomechanics::OpenSimFile::skeleton)
      .def_readwrite(
          "trackingMarkers", &dart::biomechanics::OpenSimFile::trackingMarkers)
      .def_readwrite(
          "anatomicalMarkers",
          &dart::biomechanics::OpenSimFile::anatomicalMarkers)
      .def_readwrite(
          "markersMap", &dart::biomechanics::OpenSimFile::markersMap);

  ::py::class_<dart::biomechanics::OpenSimMot>(m, "OpenSimMot")
      .def_readwrite("poses", &dart::biomechanics::OpenSimMot::poses)
      .def_readwrite("timestamps", &dart::biomechanics::OpenSimMot::timestamps);

  ::py::class_<dart::biomechanics::OpenSimTRC>(m, "OpenSimTRC")
      .def_readwrite(
          "markerTimesteps", &dart::biomechanics::OpenSimTRC::markerTimesteps)
      .def_readwrite(
          "markerLines", &dart::biomechanics::OpenSimTRC::markerLines)
      .def_readwrite("timestamps", &dart::biomechanics::OpenSimTRC::timestamps);

  ::py::class_<dart::biomechanics::OpenSimScaleAndMarkerOffsets>(
      m, "OpenSimScaleAndMarkerOffsets")
      .def_readwrite(
          "success", &dart::biomechanics::OpenSimScaleAndMarkerOffsets::success)
      .def_readwrite(
          "bodyScales",
          &dart::biomechanics::OpenSimScaleAndMarkerOffsets::bodyScales)
      .def_readwrite(
          "markerOffsets",
          &dart::biomechanics::OpenSimScaleAndMarkerOffsets::markerOffsets)
      .def_readwrite(
          "markers",
          &dart::biomechanics::OpenSimScaleAndMarkerOffsets::markers);

  auto sm = m.def_submodule("OpenSimParser");
  sm.def(
      "parseOsim",
      +[](const std::string& path) {
        return dart::biomechanics::OpenSimParser::parseOsim(path);
      },
      ::py::arg("path"));

  sm.def(
      "saveOsimScalingXMLFile",
      +[](const std::string& subjectName,
          std::shared_ptr<dynamics::Skeleton> skel,
          double massKg,
          double heightM,
          const std::string& osimInputPath,
          const std::string& osimOutputPath,
          const std::string& scalingInstructionsOutputPath) {
        return dart::biomechanics::OpenSimParser::saveOsimScalingXMLFile(
            subjectName,
            skel,
            massKg,
            heightM,
            osimInputPath,
            osimOutputPath,
            scalingInstructionsOutputPath);
      },
      ::py::arg("subjectName"),
      ::py::arg("skel"),
      ::py::arg("massKg"),
      ::py::arg("heightM"),
      ::py::arg("osimInputPath"),
      ::py::arg("osimOutputPath"),
      ::py::arg("scalingInstructionsOutputPath"));

  sm.def(
      "saveOsimInverseKinematicsXMLFile",
      +[](const std::string& subjectName,
          std::vector<std::string> markerNames,
          const std::string& osimInputModelPath,
          const std::string& osimInputTrcPath,
          const std::string& osimOutputMotPath,
          const std::string& ikInstructionsOutputPath) {
        return dart::biomechanics::OpenSimParser::
            saveOsimInverseKinematicsXMLFile(
                subjectName,
                markerNames,
                osimInputModelPath,
                osimInputTrcPath,
                osimOutputMotPath,
                ikInstructionsOutputPath);
      },
      ::py::arg("subjectName"),
      ::py::arg("markerNames"),
      ::py::arg("osimInputModelPath"),
      ::py::arg("osimInputTrcPath"),
      ::py::arg("osimOutputMotPath"),
      ::py::arg("ikInstructionsOutputPath"));

  sm.def(
      "saveOsimInverseDynamicsForcesXMLFile",
      +[](const std::string& subjectName,
          std::shared_ptr<dynamics::Skeleton> skel,
          const Eigen::MatrixXs& poses,
          const std::vector<biomechanics::ForcePlate> forcePlates,
          const std::string& grfForcesPath,
          const std::string& forcesOutputPath) {
        return dart::biomechanics::OpenSimParser::
            saveOsimInverseDynamicsForcesXMLFile(
                subjectName,
                skel,
                poses,
                forcePlates,
                grfForcesPath,
                forcesOutputPath);
      },
      ::py::arg("subjectName"),
      ::py::arg("skel"),
      ::py::arg("poses"),
      ::py::arg("forcePlates"),
      ::py::arg("grfForcePath"),
      ::py::arg("forcesOutputPath"));

  sm.def(
      "saveOsimInverseDynamicsXMLFile",
      +[](const std::string& subjectName,
          const std::string& osimInputModelPath,
          const std::string& osimInputMotPath,
          const std::string& osimForcesXmlPath,
          const std::string& osimOutputStoPath,
          const std::string& osimOutputBodyForcesStoPath,
          const std::string& idInstructionsOutputPath) {
        return dart::biomechanics::OpenSimParser::
            saveOsimInverseDynamicsXMLFile(
                subjectName,
                osimInputModelPath,
                osimInputMotPath,
                osimForcesXmlPath,
                osimOutputStoPath,
                osimOutputBodyForcesStoPath,
                idInstructionsOutputPath);
      },
      ::py::arg("subjectName"),
      ::py::arg("osimInputModelPath"),
      ::py::arg("osimInputMotPath"),
      ::py::arg("osimForcesXmlPath"),
      ::py::arg("osimOutputStoPath"),
      ::py::arg("osimOutputBodyForcesStoPath"),
      ::py::arg("idInstructionsOutputPath"));

  sm.def(
      "rationalizeJoints",
      +[](const common::Uri& uri, const std::string& outputPath) {
        return dart::biomechanics::OpenSimParser::rationalizeJoints(
            uri, outputPath);
      },
      ::py::arg("inputPath"),
      ::py::arg("outputPath"));

  sm.def(
      "moveOsimMarkers",
      +[](const common::Uri& uri,
          const std::map<std::string, Eigen::Vector3s>& bodyScales,
          const std::map<std::string, std::pair<std::string, Eigen::Vector3s>>&
              markerOffsets,
          const std::string& outputPath) {
        return dart::biomechanics::OpenSimParser::moveOsimMarkers(
            uri, bodyScales, markerOffsets, outputPath);
      },
      ::py::arg("inputPath"),
      ::py::arg("bodyScales"),
      ::py::arg("markerOffsets"),
      ::py::arg("outputPath"));

  sm.def(
      "loadTRC",
      +[](const std::string& path) {
        return dart::biomechanics::OpenSimParser::loadTRC(path);
      },
      ::py::arg("path"));

  sm.def(
      "saveTRC",
      +[](const std::string& outputPath,
          const std::vector<double>& timestamps,
          const std::vector<std::map<std::string, Eigen::Vector3s>>&
              markerTimesteps) {
        return dart::biomechanics::OpenSimParser::saveTRC(
            outputPath, timestamps, markerTimesteps);
      },
      ::py::arg("path"),
      ::py::arg("timestamps"),
      ::py::arg("markerTimestamps"));

  sm.def(
      "loadMot",
      +[](std::shared_ptr<dynamics::Skeleton> skel, const std::string& path) {
        return dart::biomechanics::OpenSimParser::loadMot(skel, path);
      },
      ::py::arg("skel"),
      ::py::arg("path"));

  sm.def(
      "loadMotAtLowestMarkerRMSERotation",
      +[](biomechanics::OpenSimFile osim,
          const std::string& path,
          biomechanics::C3D c3d) {
        return dart::biomechanics::OpenSimParser::
            loadMotAtLowestMarkerRMSERotation(osim, path, c3d);
      },
      ::py::arg("osim"),
      ::py::arg("path"),
      ::py::arg("c3d"));

  sm.def(
      "saveMot",
      +[](std::shared_ptr<dynamics::Skeleton> skel,
          const std::string& path,
          const std::vector<double>& timestamps,
          const Eigen::MatrixXs& poses) {
        return dart::biomechanics::OpenSimParser::saveMot(
            skel, path, timestamps, poses);
      },
      ::py::arg("skel"),
      ::py::arg("path"),
      ::py::arg("timestamps"),
      ::py::arg("poses"));

  sm.def(
      "saveGRFMot",
      +[](const std::string& outputPath,
          const std::vector<double>& timestamps,
          const std::vector<biomechanics::ForcePlate> forcePlates) {
        return dart::biomechanics::OpenSimParser::saveGRFMot(
            outputPath, timestamps, forcePlates);
      },
      ::py::arg("outputPath"),
      ::py::arg("timestamps"),
      ::py::arg("forcePlates"));

  sm.def(
      "getScaleAndMarkerOffsets",
      &dart::biomechanics::OpenSimParser::getScaleAndMarkerOffsets,
      ::py::arg("standardSkeleton"),
      ::py::arg("scaledSkeleton"));
}

} // namespace python
} // namespace dart
