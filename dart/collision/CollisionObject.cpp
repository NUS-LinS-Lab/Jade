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

#include "dart/collision/CollisionObject.hpp"

#include "dart/collision/CollisionDetector.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

namespace dart {
namespace collision {

//==============================================================================
CollisionDetector* CollisionObject::getCollisionDetector()
{
  return mCollisionDetector;
}

//==============================================================================
const CollisionDetector* CollisionObject::getCollisionDetector() const
{
  return mCollisionDetector;
}

//==============================================================================
const dynamics::ShapeFrame* CollisionObject::getShapeFrame() const
{
  return mShapeFrame;
}

//==============================================================================
dynamics::ConstShapePtr CollisionObject::getShape() const
{
  return mShapeFrame->getShape();
}

//==============================================================================
const Eigen::Isometry3s& CollisionObject::getTransform() const
{
  return mShapeFrame->getWorldTransform();
}

void CollisionObject::saveOldTransform()
{
  mOldTransform = mShapeFrame->getWorldTransform();
}

void CollisionObject::saveNewTransform()
{
  mNewTransform = mShapeFrame->getWorldTransform();
}

Eigen::Isometry3s CollisionObject::getMidTransform(double lambda)
{
  Eigen::Quaternion<double> qOld(mOldTransform.linear());
  Eigen::Quaternion<double> qNew(mNewTransform.linear());
  Eigen::Vector3d tOld(mOldTransform.translation());
  Eigen::Vector3d tNew(mNewTransform.translation());

  Eigen::Quaternion<double> qMid = qOld.slerp(lambda, qNew);
  Eigen::Vector3d tMid = tOld * (1 - lambda) + tNew * lambda;
  Eigen::Isometry3s midTransform;
  midTransform.linear() = qMid.toRotationMatrix();
  midTransform.translation() = tMid;

  return midTransform;
}

//==============================================================================
CollisionObject::CollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : mCollisionDetector(collisionDetector),
    mShapeFrame(shapeFrame)
{
  assert(mCollisionDetector);
  assert(mShapeFrame);
}

}  // namespace collision
}  // namespace dart
