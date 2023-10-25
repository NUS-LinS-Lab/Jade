#include "dart/utils/UniversalLoader.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/utils/SkelParser.hpp"
#include "dart/utils/sdf/SdfParser.hpp"
#include "dart/utils/urdf/DartLoader.hpp"
#include "dart/utils/DartResourceRetriever.hpp"

namespace dart {
namespace utils {
namespace UniversalLoader {

/// Credit
/// https://stackoverflow.com/questions/20446201/how-to-check-if-string-ends-with-txt/20446257
/// keenon: Yes I'm embarassed to copy-paste something as simple as this, but I
/// was too lazy to write this myself early in the morning.
bool hasSuffix(const std::string& str, const std::string& suffix)
{
  return str.size() >= suffix.size()
         && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Source: https://stackoverflow.com/a/145309/13177487
#include <stdio.h> /* defines FILENAME_MAX */
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

//==============================================================================
std::shared_ptr<simulation::World> loadWorld(const std::string& path)
{
  return dart::utils::SkelParser::readWorld(path);
}

//==============================================================================
std::shared_ptr<dynamics::Skeleton> loadSkeleton(
    simulation::World* world,
    std::string path,
    Eigen::Vector3s basePosition,
    Eigen::Vector3s baseEulerAnglesXYZ)
{
  std::shared_ptr<dynamics::Skeleton> skel = nullptr;

  // Source: https://stackoverflow.com/a/145309/13177487
  if (path[0] == '.')
  {
    char cCurrentPath[FILENAME_MAX];

    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
    {
      // ignore, couldn't prefix the current working directory
    }
    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
    std::string cwd(cCurrentPath);

    path = cwd + "/" + path;
  }

  if (hasSuffix(path, ".skel"))
  {
    skel = SkelParser::readSkeleton(path);
  }
  else if (hasSuffix(path, ".urdf"))
  {
    dart::utils::DartLoader urdfLoader;
    skel = urdfLoader.parseSkeleton(path);
  }
  else if (hasSuffix(path, ".sdf"))
  {
    skel = dart::utils::SdfParser::readSkeleton(path);
  }
  else
  {
    dterr << "[UniversalLoader] Attempting to load a file [" << path
          << "] that is does not have a supported "
          << "extension. Currently, only \".skel\", \".urdf\" and \".sdf\" "
             "files are supported.\n";
    return nullptr;
  }

  if (skel == nullptr)
  {
    dterr << "[UniversalLoader] Error when ettempting to load a file [" << path
          << "]. Underlying loader returned a NULL skeleton. Please double "
             "check that the "
             "file being loaded is valid.\n";
    return skel;
  }

  // Update all the root joints with the new specified transform

  Eigen::Isometry3s baseTransform = Eigen::Isometry3s::Identity();
  baseTransform.translation() = basePosition;
  baseTransform.linear() = math::eulerXYZToMatrix(baseEulerAnglesXYZ);
  for (int i = 0; i < skel->getNumTrees(); i++)
  {
    dynamics::Joint* joint = skel->getRootJoint(i);
    joint->setTransformFromParentBodyNode(
        baseTransform * joint->getTransformFromParentBodyNode());
  }

  // Add the skeleton to the world

  world->addSkeleton(skel);

  return skel;
}

/// This loads a mesh from a file
std::shared_ptr<dynamics::MeshShape> loadMeshShape(std::string path)
{
  // Source: https://stackoverflow.com/a/145309/13177487
  if (path[0] == '.')
  {
    char cCurrentPath[FILENAME_MAX];

    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
    {
      // ignore, couldn't prefix the current working directory
    }
    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
    std::string cwd(cCurrentPath);

    path = cwd + "/" + path;
  }

  auto retriever = std::make_shared<utils::CompositeResourceRetriever>();
  retriever->addSchemaRetriever(
      "file", std::make_shared<common::LocalResourceRetriever>());
  retriever->addSchemaRetriever("dart", utils::DartResourceRetriever::create());
  return std::make_shared<dynamics::MeshShape>(
      Eigen::Vector3s::Ones(),
      dynamics::MeshShape::loadMesh(path, retriever),
      path,
      retriever);
}

} // namespace UniversalLoader
} // namespace utils
} // namespace dart