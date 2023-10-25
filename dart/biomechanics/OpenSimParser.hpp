#ifndef DART_UTILS_OSIMPARSER_HPP_
#define DART_UTILS_OSIMPARSER_HPP_

#include <map>
#include <string>

#include <tinyxml2.h>

#include "dart/biomechanics/C3DLoader.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/CustomJoint.hpp"
#include "dart/dynamics/EulerFreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/XmlHelpers.hpp"

namespace dart {
using namespace utils;

namespace biomechanics {

struct OpenSimFile
{
  dynamics::SkeletonPtr skeleton;
  // Markers map
  dynamics::MarkerMap markersMap;
  // TODO: use these
  std::vector<std::string> anatomicalMarkers;
  std::vector<std::string> trackingMarkers;
  // TODO: eventually we'll want to record muscles here

  OpenSimFile();
  OpenSimFile(dynamics::SkeletonPtr skeleton, dynamics::MarkerMap markersMap);
};

struct OpenSimScaleAndMarkerOffsets
{
  bool success;
  Eigen::VectorXs bodyScales;
  dynamics::MarkerMap markers;
  std::map<std::string, Eigen::Vector3s> markerOffsets;
};

/// This holds marker trajectory information from an OpenSim TRC file
struct OpenSimTRC
{
  std::vector<double> timestamps;
  std::vector<std::map<std::string, Eigen::Vector3s>> markerTimesteps;
  std::map<std::string, std::vector<Eigen::Vector3s>> markerLines;
};

struct OpenSimMot
{
  std::vector<double> timestamps;
  Eigen::MatrixXs poses;
};

struct OpenSimGRF
{
  std::vector<double> timestamps;
  std::vector<Eigen::Matrix<s_t, 3, Eigen::Dynamic>> plateCOPs;
  std::vector<Eigen::Matrix<s_t, 6, Eigen::Dynamic>> plateGRFs;
};

class OpenSimParser
{
public:
  /// Read Skeleton from *.osim file
  static OpenSimFile parseOsim(
      const common::Uri& uri,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// This creates an XML configuration file, which you can pass to the OpenSim
  /// scaling tool to rescale a skeleton
  static void saveOsimScalingXMLFile(
      const std::string& subjectName,
      std::shared_ptr<dynamics::Skeleton> skel,
      double massKg,
      double heightM,
      const std::string& osimInputPath,
      const std::string& osimOutputPath,
      const std::string& scalingInstructionsOutputPath);

  /// This creates an XML configuration file, which you can pass to the OpenSim
  /// IK tool to recreate / validate the results of IK created from this tool
  static void saveOsimInverseKinematicsXMLFile(
      const std::string& subjectName,
      std::vector<std::string> markerNames,
      const std::string& osimInputModelPath,
      const std::string& osimInputTrcPath,
      const std::string& osimOutputMotPath,
      const std::string& ikInstructionsOutputPath);

  /// This creates an XML configuration file, which you can pass to the OpenSim
  /// ID tool to recreate / validate the results of ID created from this tool
  static void saveOsimInverseDynamicsForcesXMLFile(
      const std::string& subjectName,
      std::shared_ptr<dynamics::Skeleton> skel,
      const Eigen::MatrixXs& poses,
      const std::vector<biomechanics::ForcePlate> forcePlates,
      const std::string& grfForcesPath,
      const std::string& forcesOutputPath);

  /// This creates an XML configuration file, which you can pass to the OpenSim
  /// ID tool to recreate / validate the results of ID created from this tool
  static void saveOsimInverseDynamicsXMLFile(
      const std::string& subjectName,
      const std::string& osimInputModelPath,
      const std::string& osimInputMotPath,
      const std::string& osimForcesXmlPath,
      const std::string& osimOutputStoPath,
      const std::string& osimOutputBodyForcesStoPath,
      const std::string& idInstructionsOutputPath);

  /// This gets called by rationalizeJoints()
  static void updateRootJointLimits(
      tinyxml2::XMLElement* element, dynamics::EulerFreeJoint* joint);

  /// This gets called by rationalizeJoints()
  template <std::size_t Dimension>
  static void updateCustomJointXML(
      tinyxml2::XMLElement* element, dynamics::CustomJoint<Dimension>* joint);

  /// Read an *.osim file, move any transforms saved in Custom function
  /// translation elements into the joint offsets, and write it out to a new
  /// *.osim file. If there are no "irrational" CustomJoints, then this will
  /// just save a copy of the original skeleton.
  static void rationalizeJoints(
      const common::Uri& uri,
      const std::string& outputPath,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// Read an *.osim file, move the markers to new locations, and write it out
  /// to a new *.osim file
  static void moveOsimMarkers(
      const common::Uri& uri,
      const std::map<std::string, Eigen::Vector3s>& bodyScales,
      const std::map<std::string, std::pair<std::string, Eigen::Vector3s>>&
          markerOffsets,
      const std::string& outputPath,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// This grabs the marker trajectories from a TRC file
  static OpenSimTRC loadTRC(
      const common::Uri& uri,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// This saves the *.trc file from a motion for the skeleton
  static void saveTRC(
      const std::string& outputPath,
      const std::vector<double>& timestamps,
      const std::vector<std::map<std::string, Eigen::Vector3s>>&
          markerTimesteps);

  /// This grabs the joint angles from a *.mot file
  static OpenSimMot loadMot(
      std::shared_ptr<dynamics::Skeleton> skel,
      const common::Uri& uri,
      Eigen::Matrix3s rotateBy = Eigen::Matrix3s::Identity(),
      int downsampleByFactor = 1,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// This tries a number of rotations as it's loading a .mot file, and returns
  /// the one with the lowest marker error, since that's likely to be the
  /// correct orientation.
  static OpenSimMot loadMotAtLowestMarkerRMSERotation(
      OpenSimFile& osim,
      const common::Uri& uri,
      C3D& c3d,
      int downsampleByFactor = 1,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// This saves the *.mot file from a motion for the skeleton
  static void saveMot(
      std::shared_ptr<dynamics::Skeleton> skel,
      const std::string& outputPath,
      const std::vector<double>& timestamps,
      const Eigen::MatrixXs& poses);

  /// This saves the *.mot file for the ground reaction forces we've read from
  /// a C3D file
  static void saveGRFMot(
      const std::string& outputPath,
      const std::vector<double>& timestamps,
      const std::vector<biomechanics::ForcePlate> forcePlates);

  /// This saves the *.mot file with 3 columns for each body. This is
  /// basically only used for verifying consistency between Nimble and OpenSim.
  static void saveBodyLocationsMot(
      std::shared_ptr<dynamics::Skeleton> skel,
      const std::string& outputPath,
      const std::vector<double>& timestamps,
      const Eigen::MatrixXs& poses);

  /// This saves the *.mot file with 3 columns for each marker. This is
  /// basically only used for verifying consistency between Nimble and OpenSim.
  static void saveMarkerLocationsMot(
      std::shared_ptr<dynamics::Skeleton> skel,
      const dynamics::MarkerMap& markers,
      const std::string& outputPath,
      const std::vector<double>& timestamps,
      const Eigen::MatrixXs& poses);

  /// This grabs the GRF forces from a *.mot file
  static OpenSimGRF loadGRF(
      const common::Uri& uri,
      int downsampleByFactor = 1,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// When people finish preparing their model in OpenSim, they save a *.osim
  /// file with all the scales and offsets baked in. This is a utility to go
  /// through and get out the scales and offsets in terms of a standard
  /// skeleton, so that we can include their values in standard datasets, or
  /// compare their values to the results obtained from our automatic tools.
  static OpenSimScaleAndMarkerOffsets getScaleAndMarkerOffsets(
      const OpenSimFile& standardSkeleton, const OpenSimFile& scaledSkeleton);

protected:
  static OpenSimFile readOsim30(
      const common::Uri& uri,
      tinyxml2::XMLElement* docElement,
      const common::ResourceRetrieverPtr& retriever);
  static OpenSimFile readOsim40(
      const common::Uri& uri,
      tinyxml2::XMLElement* docElement,
      const common::ResourceRetrieverPtr& retriever);
}; // namespace OpenSimParser

} // namespace biomechanics
} // namespace dart

#endif