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

#include <gtest/gtest.h>

#include "dart/collision/collision.hpp"
#include "dart/common/common.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/dynamics.hpp"
#include "dart/math/math.hpp"
#include "dart/simulation/simulation.hpp"
#include "dart/utils/utils.hpp"

#include "TestHelpers.hpp"

using namespace dart;
using namespace common;
using namespace math;
using namespace collision;
using namespace dynamics;
using namespace simulation;
using namespace utils;

#define ALL_TESTS

class Collision : public testing::Test
{
public:
};

//==============================================================================
void testSimpleFrames(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame3 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3s(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3s(1.0, 1.0, 1.0)));
  ShapePtr shape3(new BoxShape(Eigen::Vector3s(1.0, 1.0, 1.0)));

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);
  simpleFrame3->setShape(shape3);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto group3 = cd->createCollisionGroup(simpleFrame3.get());

  auto groupAll
      = cd->createCollisionGroup(group1.get(), group2.get(), group3.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(group3->getNumShapeFrames(), 1u);
  EXPECT_EQ(
      groupAll->getNumShapeFrames(),
      group1->getNumShapeFrames() + group2->getNumShapeFrames()
          + group3->getNumShapeFrames());

  for (std::size_t i = 0; i < group1->getNumShapeFrames(); ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group1->getShapeFrame(i));

  std::size_t start = group1->getNumShapeFrames();
  std::size_t end = start + group2->getNumShapeFrames();
  for (std::size_t i = start; i < end; ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group2->getShapeFrame(i - start));

  start = start + group2->getNumShapeFrames();
  end = start + group3->getNumShapeFrames();
  for (std::size_t i = start; i < end; ++i)
    EXPECT_EQ(groupAll->getShapeFrame(i), group3->getShapeFrame(i - start));

  collision::CollisionOption option;
  collision::CollisionResult result;

  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(1.1, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3s(2.2, 0.0, 0.0));
  EXPECT_FALSE(group1->collide(option, &result));
  EXPECT_FALSE(group2->collide(option, &result));
  EXPECT_FALSE(group3->collide(option, &result));
  EXPECT_FALSE(groupAll->collide(option, &result));

  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(0.5, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3s(1.0, 0.0, 0.0));
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));
  EXPECT_TRUE(group2->collide(group3.get(), option, &result));
  EXPECT_TRUE(groupAll->collide(option, &result));

  auto group23
      = cd->createCollisionGroup(simpleFrame2.get(), simpleFrame3.get());
  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(1.1, 0.0, 0.0));
  simpleFrame3->setTranslation(Eigen::Vector3s(1.6, 0.0, 0.0));
  EXPECT_FALSE(group1->collide(group2.get()));
  EXPECT_FALSE(group1->collide(group3.get()));
  EXPECT_TRUE(group2->collide(group3.get()));
  EXPECT_TRUE(group23->collide());
#if HAVE_BULLET
  if (cd->getType() == BulletCollisionDetector::getStaticType())
  {
    dtwarn << "Skipping group-group test for 'bullet' collision detector. "
           << "Please see Issue #717 for the detail.\n";
  }
  else
#endif
  {
    EXPECT_FALSE(group1->collide(group23.get()));
  }
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, SimpleFrames)
{
  auto dart = DARTCollisionDetector::create();
  testSimpleFrames(dart);
}
#endif

//==============================================================================
void testSphereSphere(
    const std::shared_ptr<CollisionDetector>& cd, s_t tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new SphereShape(1.0));
  ShapePtr shape2(new SphereShape(0.5));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(2.0, 0.0, 0.0));

  //----------------------------------------------------------------------------
  // Test 1: No contact
  //----------------------------------------------------------------------------

  result.clear();
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(1.5, 0.0, 0.0));

  //----------------------------------------------------------------------------
  // Test 2: Point contact
  //----------------------------------------------------------------------------

  result.clear();
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 1u);

  const auto& contact = result.getContact(0);

  // Test contact location
  EXPECT_TRUE(contact.point.isApprox(Eigen::Vector3s::UnitX(), tol));

  // Test normal
  Eigen::Vector3s expectedNormal;
  if (result.getContact(0).collisionObject1->getShapeFrame()
      == simpleFrame1.get())
    expectedNormal << -1, 0, 0;
  else
    expectedNormal << 1, 0, 0;
  s_t tol2 = tol;
  EXPECT_TRUE(contact.normal.isApprox(expectedNormal, tol2));

  //----------------------------------------------------------------------------
  // Test 3: Corner case of that the bigger sphere completely encloses the
  // smaller sphere
  //----------------------------------------------------------------------------

  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s::Zero());
  result.clear();
  // Deep inter-penetration is clipped
  EXPECT_FALSE(group->collide(option, &result));
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, SphereSphere)
{
  auto dart = DARTCollisionDetector::create();
  testSphereSphere(dart);
}
#endif

//==============================================================================
bool checkBoundingBox(
    const Eigen::Vector3s& min,
    const Eigen::Vector3s& max,
    const Eigen::Vector3s& point,
    s_t tol = 1e-12)
{
  for (auto i = 0u; i < 3u; ++i)
  {
    if (min[i] - tol > point[i] || point[i] > max[i] + tol)
      return false;
  }

  return true;
}

//==============================================================================
void testBoxBox(const std::shared_ptr<CollisionDetector>& cd, s_t tol = 1e-12)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3s(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3s(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  Eigen::Vector3s pos1 = Eigen::Vector3s(0.0, 0.0, -0.5);
  Eigen::Vector3s pos2 = Eigen::Vector3s(0.0, 0.5, 0.25);
  simpleFrame1->setTranslation(pos1);
  simpleFrame2->setTranslation(pos2);

  auto group1 = cd->createCollisionGroup(simpleFrame1.get());
  auto group2 = cd->createCollisionGroup(simpleFrame2.get());
  auto groupAll = cd->createCollisionGroup(group1.get(), group2.get());

  EXPECT_EQ(group1->getNumShapeFrames(), 1u);
  EXPECT_EQ(group2->getNumShapeFrames(), 1u);
  EXPECT_EQ(
      groupAll->getNumShapeFrames(),
      group1->getNumShapeFrames() + group2->getNumShapeFrames());

  collision::CollisionOption option;
  collision::CollisionResult result;

  result.clear();
  EXPECT_TRUE(group1->collide(group2.get(), option, &result));

  result.clear();
  EXPECT_TRUE(groupAll->collide(option, &result));

  Eigen::Vector3s min = Eigen::Vector3s(-0.25, 0.25, 0.0);
  Eigen::Vector3s max = Eigen::Vector3s(0.25, 0.5, 0.0);

  const auto numContacts = result.getNumContacts();

  const auto checkNumContacts = (numContacts <= 4u);
  EXPECT_TRUE(checkNumContacts);
  if (!checkNumContacts)
    std::cout << "# of contants: " << numContacts << "\n";

  for (const auto& contact : result.getContacts())
  {
    const auto& point = contact.point;

    const auto result = checkBoundingBox(min, max, point, tol);
    EXPECT_TRUE(result);

    if (!result)
      std::cout << "point: " << point.transpose() << "\n";
  }
}

//==============================================================================
// #ifdef ALL_TESTS
TEST_F(Collision, BoxBox)
{
  auto dart = DARTCollisionDetector::create();
  testBoxBox(dart);
}
// #endif

//==============================================================================
void testOptions(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame3 = SimpleFrame::createShared(Frame::World());

  ShapePtr shape1(new BoxShape(Eigen::Vector3s(1.0, 1.0, 1.0)));
  ShapePtr shape2(new BoxShape(Eigen::Vector3s(0.5, 0.5, 0.5)));
  ShapePtr shape3(new BoxShape(Eigen::Vector3s(0.5, 0.5, 0.5)));
  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);
  simpleFrame3->setShape(shape3);

  Eigen::Vector3s pos1 = Eigen::Vector3s(0.0, 0.0, -0.5);
  Eigen::Vector3s pos2 = Eigen::Vector3s(0.0, 0.5, 0.25);
  Eigen::Vector3s pos3 = Eigen::Vector3s(0.0, -0.5, 0.25);
  simpleFrame1->setTranslation(pos1);
  simpleFrame2->setTranslation(pos2);
  simpleFrame3->setTranslation(pos3);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  collision::CollisionResult result;

  result.clear();
  option.maxNumContacts = 1000u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 4u);

  result.clear();
  option.maxNumContacts = 2u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 2u);

  group->addShapeFrame(simpleFrame3.get());
  result.clear();
  option.maxNumContacts = 1u;
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 1u);

  // Binary check without passing result
  EXPECT_TRUE(group->collide(option));

  // Binary check without passing option and result
  EXPECT_TRUE(group->collide());

  // Zero maximum number of contacts
  option.maxNumContacts = 0u;
  option.enableContact = true;
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));
  EXPECT_FALSE(group->collide(option, nullptr));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);
  EXPECT_FALSE(result.isCollision());
}

//==============================================================================
void testCylinderCylinder(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<CylinderShape>(1.0, 1.0);
  auto shape2 = std::make_shared<CylinderShape>(0.5, 1.0);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(2.0, 0.0, 0.0));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(0.75, 0.0, 0.0));
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() >= 1u);
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, testCylinderCylinder)
{
  // auto dart = DARTCollisionDetector::create();
  // testCylinderCylinder(dart);
}
#endif

//==============================================================================
void testCapsuleCapsule(const std::shared_ptr<CollisionDetector>& cd)
{
  auto simpleFrame1 = SimpleFrame::createShared(Frame::World());
  auto simpleFrame2 = SimpleFrame::createShared(Frame::World());

  auto shape1 = std::make_shared<CapsuleShape>(1.0, 1.0);
  auto shape2 = std::make_shared<CapsuleShape>(0.5, 1.0);

  simpleFrame1->setShape(shape1);
  simpleFrame2->setShape(shape2);

  auto group = cd->createCollisionGroup(simpleFrame1.get(), simpleFrame2.get());

  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(2.0, 0.0, 0.0));
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() == 0u);

  result.clear();
  simpleFrame1->setTranslation(Eigen::Vector3s::Zero());
  simpleFrame2->setTranslation(Eigen::Vector3s(0.74, 0.0, 0.0));
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_TRUE(result.getNumContacts() >= 1u);
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, testCapsuleCapsule)
{
  // auto dart = DARTCollisionDetector::create();
  // testCapsuleCapsule(dart);
}
#endif

//==============================================================================
void testPlane(const std::shared_ptr<CollisionDetector>& cd)
{
  auto planeFrame = SimpleFrame::createShared(Frame::World());
  auto sphereFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());

  auto plane = std::make_shared<PlaneShape>(Eigen::Vector3s::UnitZ(), 0.0);
  auto sphere = std::make_shared<SphereShape>(0.5);
  auto box = std::make_shared<BoxShape>(Eigen::Vector3s(1.0, 1.0, 1.0));

  planeFrame->setShape(plane);
  sphereFrame->setShape(sphere);
  boxFrame->setShape(box);

  auto group = cd->createCollisionGroup(
      planeFrame.get(), sphereFrame.get(), boxFrame.get());

  EXPECT_EQ(group->getNumShapeFrames(), 3u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;

  result.clear();
  sphereFrame->setTranslation(Eigen::Vector3s(-10.0, 0.0, 1.0));
  boxFrame->setTranslation(Eigen::Vector3s(-8.0, 0.0, 1.0));
  EXPECT_FALSE(group->collide(option, &result));

  result.clear();
  sphereFrame->setTranslation(Eigen::Vector3s(-10.0, 0.0, 0.49));
  boxFrame->setTranslation(Eigen::Vector3s(-8.0, 0.0, 0.49));
  EXPECT_TRUE(group->collide(option, &result));
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, testPlane)
{
#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  testPlane(ode);
#endif
}
#endif

//==============================================================================
/// \param[in] collidesUnderTerrain Set to true if the collision engine returns
/// collisions when a shape is underneath the terrain, but still above the
/// minimum height. If false, only intersections with the surface mesh will be
/// detected.
/// \param[in] extendsUntilGroundPlane Set to true if the collision engine
/// extends the terrain until the plane z=0
/// \param[in] odeThck: for ODE, use this thickness underneath the heightfield
/// to adjust collision checks.
///
/// \sa dGeomHeightfieldDataBuild*().
template <typename S>
void testHeightmapBox(
    CollisionDetector* cd,
    const bool collidesUnderTerrain = true,
    const bool extendsUntilGroundPlane = false,
    const S odeThck = 0)
{
  using Vector3 = Eigen::Matrix<S, 3, 1>;

  ///////////////////////////////////////
  // Set test parameters.
  // The height field will have a flat, even
  // slope spanned by four corner vertices
  ///////////////////////////////////////

  // size of box
  const S boxSize = S(0.1);
  // terrain scale in x and y direction
  const S terrainScale = S(2.0);
  // z values scale
  const S zScale = S(2.0);

  // minimum hand maximum height of terrain to use
  const S minH = 1.0; // note: ODE doesn't behave well with negative heights
  const S maxH = 3.0;
  // adjusted minimum height: If minH > 0, and extendsUntilGroundPlane true,
  // then the minimum height is actually 0.
  const S adjMinH = (extendsUntilGroundPlane && (minH > S(0))) ? 0.0 : minH;
  const S halfHeight = minH + (maxH - minH) / S(2);
  // ODE thickness is only used if there is not already a layer of this
  // thickness due to a minH > 0 (for ODE, extendsUntilGroundPlane is true)
  const S useOdeThck
      = (odeThck > S(1.0e-06)) ? std::max(odeThck - minH, S(0)) : 0.0;

  ///////////////////////////////////////
  // Create frames and shapes
  ///////////////////////////////////////

  // frames and shapes
  auto terrainFrame = SimpleFrame::createShared(Frame::World());
  auto boxFrame = SimpleFrame::createShared(Frame::World());
  auto terrainShape = std::make_shared<HeightmapShape<S>>();
  auto boxShape = std::make_shared<BoxShape>(
      Eigen::Vector3s::Constant(static_cast<s_t>(boxSize)));

  // make a terrain with a linearly increasing slope
  std::vector<S> heights = {minH, halfHeight, halfHeight, maxH};
  terrainShape->setHeightField(2u, 2u, heights);
  // set a scale to test this at the same time
  const S terrSize = terrainScale;
  terrainShape->setScale(Vector3(terrainScale, terrainScale, zScale));
  EXPECT_EQ(terrainShape->getHeightField().size(), heights.size());

  terrainFrame->setShape(terrainShape);
  boxFrame->setShape(boxShape);

  ///////////////////////////////////////
  // Test collisions
  ///////////////////////////////////////

  auto group = cd->createCollisionGroup(terrainFrame.get(), boxFrame.get());
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  collision::CollisionOption option;
  option.enableContact = true;

  collision::CollisionResult result;
  // the terrain is going to remain in the origin. During the tests,
  // we are only moving the box.
  terrainFrame->setTranslation(Eigen::Vector3s::Zero());

  // there should be no collision underneath the height field, which should be
  // on the x/y plane.
  result.clear();
  // Some tolerance (useOdeThck) has to be added for ODE because it adds an
  // extra piece on the bottom to prevent objects from falling through
  // lowest points.
  S transZ = adjMinH * zScale - boxSize * S(0.501) - useOdeThck;
  boxFrame->setTranslation(Vector3(0.0, 0.0, transZ).template cast<s_t>());
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  // expect collision if moved just slightly above the lower terrain bound
  if (collidesUnderTerrain)
  {
    result.clear();
    transZ = adjMinH * zScale - boxSize * S(0.499) - useOdeThck;
    boxFrame->setTranslation(Vector3(0.0, 0.0, transZ).template cast<s_t>());
    EXPECT_TRUE(group->collide(option, &result));
    EXPECT_GT(result.getNumContacts(), 0u);
  }

  ///////////////////////////////////////
  // test collisions when box is at extreme corner
  // points (lowest and highest)
  ///////////////////////////////////////

  // some helper vectors
  Vector3 slope(1.0, -1.0, maxH - minH);
  slope.normalize();
  Vector3 crossSection(1.0, 1.0, heights[1] - heights[2]);
  crossSection.normalize();
  const Vector3 normal = slope.cross(crossSection);
  // the two extreme corners:
  const Vector3 highCorner
      = Vector3(terrSize / S(2), -terrSize / S(2), maxH * zScale);
  const Vector3 lowCorner
      = Vector3(-terrSize / S(2), terrSize / S(2), maxH * zScale);

  // ODE doesn't do nicely when boxes are close to the border of the terrain.
  // Shift the boxes along the slope (or normal to slope for some tests)
  // by this length.
  // Technically we should compute this a bit more accurately than this.
  // it basically has to ensure the box is inside or outside the terrain
  // bounds, so the slope plays a role for this factor.
  // But since the box is small, an estimate is used for now.
  const S boxShift = boxSize * S(1.5);

  // expect collision at highest point (at max height)
  Vector3 cornerShift = highCorner - slope * boxShift;
  result.clear();
  boxFrame->setTranslation(cornerShift.template cast<s_t>());
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);

  // .. but not at opposite corner (lowest corner, at overall max height)
  result.clear();
  cornerShift = Vector3(lowCorner + slope * boxShift);
  boxFrame->setTranslation(cornerShift.template cast<s_t>());
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  ///////////////////////////////////////
  // test collisions for box on z axis
  ///////////////////////////////////////

  // box should collide where it intersects the slope
  result.clear();
  Vector3 inMiddle(0.0, 0.0, halfHeight * zScale);
  boxFrame->setTranslation(inMiddle.template cast<s_t>());
  EXPECT_TRUE(group->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);

  // ... but not if the box is translated away from the slope
  result.clear();
  Vector3 onTopOfSlope = inMiddle + normal * boxShift;
  boxFrame->setTranslation(onTopOfSlope.template cast<s_t>());
  EXPECT_FALSE(group->collide(option, &result));
  EXPECT_EQ(result.getNumContacts(), 0u);

  // ... however it still should collide if translated the
  // other way inside the slope
  if (collidesUnderTerrain)
  {
    result.clear();
    Vector3 underSlope = inMiddle - normal * boxShift;
    boxFrame->setTranslation(underSlope.template cast<s_t>());
    EXPECT_TRUE(group->collide(option, &result));
    EXPECT_GT(result.getNumContacts(), 0u);
  }
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, testHeightmapBox)
{
#if HAVE_ODE
  auto ode = OdeCollisionDetector::create();
  // TODO take this message out as soon as testing is done
  dtdbg << "Testing ODE (float)" << std::endl;
  testHeightmapBox<float>(ode.get(), true, true, 0.05f);

  // TODO take this message out as soon as testing is done
  dtdbg << "Testing ODE (s_t)" << std::endl;
  testHeightmapBox<s_t>(ode.get(), true, true, 0.05);
#endif

#if HAVE_BULLET
  auto bullet = BulletCollisionDetector::create();

  // TODO take this message out as soon as testing is done
  dtdbg << "Testing Bullet (float)" << std::endl;
  // bullet so far only supports float height fields, so don't test s_t here.
  testHeightmapBox<float>(bullet.get(), false, false);

#endif
}
#endif

//==============================================================================
// Tests HeightmapShape::flipY();
#ifdef ALL_TESTS
TEST_F(Collision, testHeightmapFlipY)
{
  using S = s_t;

  std::vector<S> heights1 = {-1, -2, 2, 1};
  auto shape = std::make_shared<HeightmapShape<S>>();
  shape->setHeightField(2, 2, heights1);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights1[2]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights1[3]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights1[0]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights1[1]);

  // test with odd number of rows
  std::vector<S> heights2 = {-1, -2, 3, 3, 2, 1};
  shape->setHeightField(2, 3, heights2);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights2[4]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights2[5]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights2[2]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights2[3]);
  EXPECT_EQ(shape->getHeightField().data()[4], heights2[0]);
  EXPECT_EQ(shape->getHeightField().data()[5], heights2[1]);

  // test higher number of rows
  std::vector<S> heights3 = {1, -1, 2, -2, 3, -3, 4, -4};
  shape->setHeightField(2, 4, heights3);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights3[6]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights3[7]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights3[4]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights3[5]);
  EXPECT_EQ(shape->getHeightField().data()[4], heights3[2]);
  EXPECT_EQ(shape->getHeightField().data()[5], heights3[3]);
  EXPECT_EQ(shape->getHeightField().data()[6], heights3[0]);
  EXPECT_EQ(shape->getHeightField().data()[7], heights3[1]);

  // test wider rows
  std::vector<S> heights4 = {1, -1, 1.5, 2, -2, 2.5, 3, -3, 3.5, 4, -4, 4.5};
  shape->setHeightField(3, 4, heights4);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights4[9]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights4[10]);
  EXPECT_EQ(shape->getHeightField().data()[2], heights4[11]);
  EXPECT_EQ(shape->getHeightField().data()[3], heights4[6]);
  EXPECT_EQ(shape->getHeightField().data()[4], heights4[7]);
  EXPECT_EQ(shape->getHeightField().data()[5], heights4[8]);
  EXPECT_EQ(shape->getHeightField().data()[6], heights4[3]);
  EXPECT_EQ(shape->getHeightField().data()[7], heights4[4]);
  EXPECT_EQ(shape->getHeightField().data()[8], heights4[5]);
  EXPECT_EQ(shape->getHeightField().data()[9], heights4[0]);
  EXPECT_EQ(shape->getHeightField().data()[10], heights4[1]);
  EXPECT_EQ(shape->getHeightField().data()[11], heights4[2]);

  // test mini (actually meaningless) height field
  std::vector<S> heights5 = {1, 2};
  shape->setHeightField(1, 2, heights5);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights5[1]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights5[0]);

  // test height field with only one row (which is actually meaningless)
  std::vector<S> heights6 = {1, 2};
  shape->setHeightField(2, 1, heights6);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights6[0]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights6[1]);

  // test height field with only one column (which is actually meaningless)
  std::vector<S> heights7 = {1, 2};
  shape->setHeightField(1, 2, heights7);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights7[1]);
  EXPECT_EQ(shape->getHeightField().data()[1], heights7[0]);

  // test height field with only one col and row (which is actually meaningless)
  std::vector<S> heights8 = {1};
  shape->setHeightField(1, 1, heights8);
  shape->flipY();
  EXPECT_EQ(shape->getHeightField().data()[0], heights8[0]);
}
#endif

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, Options)
{
  auto dart = DARTCollisionDetector::create();
  testOptions(dart);
}
#endif

//==============================================================================
void testFilter(const std::shared_ptr<CollisionDetector>& cd)
{
  // Create two bodies skeleton. The two bodies are placed at the same position
  // with the same size shape so that they collide by default.
  auto skel = Skeleton::create();
  auto shape = std::make_shared<BoxShape>(Eigen::Vector3s(1, 1, 1));
  auto pair0 = skel->createJointAndBodyNodePair<RevoluteJoint>(nullptr);
  auto* body0 = pair0.second;
  body0->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);
  auto pair1 = body0->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* body1 = pair1.second;
  body1->createShapeNodeWith<VisualAspect, CollisionAspect>(shape);

  // Create a world and add the created skeleton
  auto world = std::make_shared<simulation::World>();
  auto constraintSolver = world->getConstraintSolver();
  constraintSolver->setCollisionDetector(cd);
  world->addSkeleton(skel);

  // Get the collision group from the constraint solver
  auto group = constraintSolver->getCollisionGroup();
  EXPECT_EQ(group->getNumShapeFrames(), 2u);

  // Default collision filter for Skeleton
  auto& option = constraintSolver->getCollisionOption();
  auto bodyNodeFilter = std::make_shared<BodyNodeCollisionFilter>();
  option.collisionFilter = bodyNodeFilter;

  skel->enableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  EXPECT_TRUE(skel->isEnabledSelfCollisionCheck());
  EXPECT_TRUE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide()); // without filter, always collision
  EXPECT_TRUE(group->collide(option));

  skel->enableSelfCollisionCheck();
  skel->disableAdjacentBodyCheck();
  EXPECT_TRUE(skel->isEnabledSelfCollisionCheck());
  EXPECT_FALSE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  skel->disableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  EXPECT_FALSE(skel->isEnabledSelfCollisionCheck());
  EXPECT_TRUE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  skel->disableSelfCollisionCheck();
  skel->disableAdjacentBodyCheck();
  EXPECT_FALSE(skel->isEnabledSelfCollisionCheck());
  EXPECT_FALSE(skel->isEnabledAdjacentBodyCheck());
  EXPECT_TRUE(group->collide());
  EXPECT_FALSE(group->collide(option));

  // Test blacklist
  skel->enableSelfCollisionCheck();
  skel->enableAdjacentBodyCheck();
  bodyNodeFilter->addBodyNodePairToBlackList(body0, body1);
  EXPECT_FALSE(group->collide(option));
  bodyNodeFilter->removeBodyNodePairFromBlackList(body0, body1);
  EXPECT_TRUE(group->collide(option));
  bodyNodeFilter->addBodyNodePairToBlackList(body0, body1);
  EXPECT_FALSE(group->collide(option));
  bodyNodeFilter->removeAllBodyNodePairsFromBlackList();
  EXPECT_TRUE(group->collide(option));
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, Filter)
{
  auto dart = DARTCollisionDetector::create();
  testFilter(dart);
}
#endif

//==============================================================================
void testCreateCollisionGroups(const std::shared_ptr<CollisionDetector>& cd)
{
  Eigen::Vector3s size(1.0, 1.0, 1.0);
  Eigen::Vector3s pos1(0.0, 0.0, 0.0);
  Eigen::Vector3s pos2(0.5, 0.0, 0.0);

  auto boxSkeleton1 = createBox(size, pos1);
  auto boxSkeleton2 = createBox(size, pos2);

  auto boxBodyNode1 = boxSkeleton1->getBodyNode(0u);
  auto boxBodyNode2 = boxSkeleton2->getBodyNode(0u);

  auto boxShapeNode1 = boxBodyNode1->getShapeNodesWith<CollisionAspect>()[0];
  auto boxShapeNode2 = boxBodyNode2->getShapeNodesWith<CollisionAspect>()[0];

  collision::CollisionOption option;
  collision::CollisionResult result;

  auto skeletonGroup1 = cd->createCollisionGroup(boxSkeleton1.get());
  auto skeletonGroup2 = cd->createCollisionGroup(boxSkeleton2.get());

  auto bodyNodeGroup1 = cd->createCollisionGroup(boxBodyNode1);
  auto bodyNodeGroup2 = cd->createCollisionGroup(boxBodyNode2);

  auto shapeNodeGroup1 = cd->createCollisionGroup(boxShapeNode1);
  auto shapeNodeGroup2 = cd->createCollisionGroup(boxShapeNode2);

  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get(), option, &result));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get(), option, &result));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get(), option, &result));

  // Binary check without passing option
  auto oldMaxNumContacts = option.maxNumContacts;
  option.maxNumContacts = 1u;
  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get(), option));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get(), option));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get(), option));
  option.maxNumContacts = oldMaxNumContacts;

  // Binary check without passing option and result
  EXPECT_TRUE(skeletonGroup1->collide(skeletonGroup2.get()));
  EXPECT_TRUE(bodyNodeGroup1->collide(bodyNodeGroup2.get()));
  EXPECT_TRUE(shapeNodeGroup1->collide(shapeNodeGroup2.get()));

  // Regression test for #666
  auto world = std::make_unique<World>();
  world->getConstraintSolver()->setCollisionDetector(cd);
  world->addSkeleton(boxSkeleton1);
  world->addSkeleton(boxSkeleton2);
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_FALSE(boxBodyNode1->isColliding());
  EXPECT_FALSE(boxBodyNode2->isColliding());
  DART_SUPPRESS_DEPRECATED_END

  const collision::CollisionResult& result1 = world->getLastCollisionResult();
  EXPECT_FALSE(result1.inCollision(boxBodyNode1));
  EXPECT_FALSE(result1.inCollision(boxBodyNode2));

  world->step();
  DART_SUPPRESS_DEPRECATED_BEGIN
  EXPECT_TRUE(boxBodyNode1->isColliding());
  EXPECT_TRUE(boxBodyNode2->isColliding());
  DART_SUPPRESS_DEPRECATED_END

  const collision::CollisionResult& result2 = world->getLastCollisionResult();
  EXPECT_TRUE(result2.inCollision(boxBodyNode1));
  EXPECT_TRUE(result2.inCollision(boxBodyNode2));
}

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, CreateCollisionGroupFromVariousObject)
{
  auto dart = DARTCollisionDetector::create();
  testCreateCollisionGroups(dart);
}
#endif

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, CollisionOfPrescribedJoints)
{
  // There are one red plate (static skeleton) and 5 pendulums with different
  // actuator types. This test check if the motion prescribed joints are exactly
  // tracking the prescribed motion eventhough there are collision with other
  // objects.

  const double tol = 1e-9;
  const s_t timeStep = 1e-3;
  const std::size_t numFrames = 5e+0; // 5 secs

  // Load world and skeleton
  WorldPtr world = SkelParser::readWorld(
      "dart://sample/skel/test/collision_of_prescribed_joints_test.skel");
  world->setTimeStep(timeStep);
  EXPECT_TRUE(world != nullptr);
  // Gradients don't support prescribed joints
  world->getConstraintSolver()->setGradientEnabled(false);
  EXPECT_NEAR(
      static_cast<double>(world->getTimeStep()),
      static_cast<double>(timeStep),
      tol);

  SkeletonPtr skel1 = world->getSkeleton("skeleton 1");
  SkeletonPtr skel2 = world->getSkeleton("skeleton 2");
  SkeletonPtr skel3 = world->getSkeleton("skeleton 3");
  SkeletonPtr skel4 = world->getSkeleton("skeleton 4");
  SkeletonPtr skel5 = world->getSkeleton("skeleton 5");
  SkeletonPtr skel6 = world->getSkeleton("skeleton 6");
  EXPECT_TRUE(skel1 != nullptr);
  EXPECT_TRUE(skel2 != nullptr);
  EXPECT_TRUE(skel3 != nullptr);
  EXPECT_TRUE(skel4 != nullptr);
  EXPECT_TRUE(skel5 != nullptr);
  EXPECT_TRUE(skel6 != nullptr);

  Joint* joint1 = skel1->getJoint(0);
  Joint* joint2 = skel2->getJoint(0);
  Joint* joint3 = skel3->getJoint(0);
  Joint* joint4 = skel4->getJoint(0);
  Joint* joint5 = skel5->getJoint(0);
  Joint* joint6 = skel6->getJoint(0);
  EXPECT_TRUE(joint1 != nullptr);
  EXPECT_TRUE(joint2 != nullptr);
  EXPECT_TRUE(joint3 != nullptr);
  EXPECT_TRUE(joint4 != nullptr);
  EXPECT_TRUE(joint5 != nullptr);
  EXPECT_TRUE(joint6 != nullptr);
  EXPECT_EQ(joint1->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(joint2->getActuatorType(), Joint::PASSIVE);
  EXPECT_EQ(joint3->getActuatorType(), Joint::SERVO);
  EXPECT_EQ(joint4->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(joint5->getActuatorType(), Joint::VELOCITY);
  EXPECT_EQ(joint6->getActuatorType(), Joint::LOCKED);

  for (std::size_t i = 0; i < numFrames; ++i)
  {
    const s_t time = world->getTime();

    joint1->setCommand(0, -0.5 * constantsd::pi() * cos(time));
    joint2->setCommand(0, -0.5 * constantsd::pi() * cos(time));
    joint3->setCommand(0, -0.5 * constantsd::pi() * cos(time));
    joint4->setCommand(0, -0.5 * constantsd::pi() * cos(time));
    joint5->setCommand(0, -0.5 * constantsd::pi() * sin(time));
    joint6->setCommand(0, -0.5 * constantsd::pi() * sin(time)); // ignored

    world->step(false);

    EXPECT_TRUE(joint1->isDynamic());
    EXPECT_TRUE(joint2->isDynamic());
    EXPECT_TRUE(joint3->isDynamic());

    // Check if the motion prescribed joints are following the prescribed motion
    // eventhough there is a collision with other objects
    EXPECT_TRUE(joint4->isKinematic());
    EXPECT_NEAR(
        static_cast<double>(joint4->getAcceleration(0)),
        static_cast<double>(joint4->getCommand(0)),
        tol);
    EXPECT_TRUE(joint5->isKinematic());
    EXPECT_NEAR(
        static_cast<double>(joint5->getVelocity(0)),
        static_cast<double>(joint5->getCommand(0)),
        tol);

    // The velocity and acceleration of locked joint always must be zero.
    EXPECT_TRUE(joint6->isKinematic());
    EXPECT_NEAR(static_cast<double>(joint6->getVelocity(0)), 0.0, tol);
    EXPECT_NEAR(static_cast<double>(joint6->getAcceleration(0)), 0.0, tol);
  }
}
#endif

//==============================================================================
#ifdef ALL_TESTS
TEST_F(Collision, Factory)
{
  EXPECT_TRUE(collision::CollisionDetector::getFactory()->canCreate("dart"));

#if HAVE_BULLET
  EXPECT_TRUE(collision::CollisionDetector::getFactory()->canCreate("bullet"));
#else
  EXPECT_TRUE(!collision::CollisionDetector::getFactory()->canCreate("bullet"));
#endif

#if HAVE_ODE
  EXPECT_TRUE(collision::CollisionDetector::getFactory()->canCreate("ode"));
#else
  EXPECT_TRUE(!collision::CollisionDetector::getFactory()->canCreate("ode"));
#endif
}
#endif