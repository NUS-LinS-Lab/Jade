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

#include "dart/constraint/ContactConstraint.hpp"

#include <iostream>

#include "dart/collision/CollisionObject.hpp"
#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/external/odelcpsolver/lcp.h"
#include "dart/math/Helpers.hpp"

#define DART_EPSILON 1e-6
#define DART_ERROR_ALLOWANCE 0.0
#define DART_ERP 0.01
#define DART_MAX_ERV 1e-3
#define DART_CFM 1e-5
// #define DART_MAX_NUMBER_OF_CONTACTS 32

#define DART_RESTITUTION_COEFF_THRESHOLD 1e-3
#define DART_FRICTION_COEFF_THRESHOLD 1e-3
#define DART_BOUNCING_VELOCITY_THRESHOLD 1e-1
#define DART_MAX_BOUNCING_VELOCITY 1e+2
#define DART_CONTACT_CONSTRAINT_EPSILON_SQUARED 1e-12

namespace dart {
namespace constraint {

s_t ContactConstraint::mErrorAllowance = DART_ERROR_ALLOWANCE;
s_t ContactConstraint::mErrorReductionParameter = DART_ERP;
s_t ContactConstraint::mMaxErrorReductionVelocity = DART_MAX_ERV;
s_t ContactConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
ContactConstraint::ContactConstraint(
    collision::Contact& contact,
    s_t timeStep,
    bool penetrationCorrectionEnabled)
  : ConstraintBase(),
    mTimeStep(timeStep),
    mBodyNodeA(const_cast<dynamics::ShapeFrame*>(
                   contact.collisionObject1->getShapeFrame())
                   ->asShapeNode()
                   ->getBodyNodePtr()),
    mBodyNodeB(const_cast<dynamics::ShapeFrame*>(
                   contact.collisionObject2->getShapeFrame())
                   ->asShapeNode()
                   ->getBodyNodePtr()),
    mContact(contact),
    mFirstFrictionalDirection(Eigen::Vector3s::UnitZ()),
    mIsFrictionOn(true),
    mAppliedImpulseIndex(dynamics::INVALID_INDEX),
    mPenetrationCorrectionEnabled(penetrationCorrectionEnabled),
    mDidBounce(false),
    mIsBounceOn(false),
    mActive(false)
{
  assert(
      contact.normal.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED);

  //----------------------------------------------
  // Bounce
  //----------------------------------------------
  mRestitutionCoeff
      = mBodyNodeA->getRestitutionCoeff() * mBodyNodeB->getRestitutionCoeff();
  if (mRestitutionCoeff > DART_RESTITUTION_COEFF_THRESHOLD)
    mIsBounceOn = true;
  else
    mIsBounceOn = false;

  //----------------------------------------------
  // Friction
  //----------------------------------------------
  // TODO(JS): Assume the frictional coefficient can be changed during
  //           simulation steps.
  // Update mFrictionalCoff
  mFrictionCoeff = std::min(
      mBodyNodeA->getFrictionCoeff(), mBodyNodeB->getFrictionCoeff());
  if (mFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD)
  {
    mIsFrictionOn = true;

    // Update frictional direction
    updateFirstFrictionalDirection();
  }
  else
  {
    mIsFrictionOn = false;
  }

  assert(mBodyNodeA->getSkeleton());
  assert(mBodyNodeB->getSkeleton());
  mIsSelfCollision = (mBodyNodeA->getSkeleton() == mBodyNodeB->getSkeleton());

  const math::Jacobian jacA
      = mBodyNodeA->getSkeleton()->getJacobian(mBodyNodeA);
  const math::Jacobian jacB
      = mBodyNodeB->getSkeleton()->getJacobian(mBodyNodeB);

  // Compute local contact Jacobians expressed in body frame
  if (mIsFrictionOn)
  {
    // Set the dimension of this constraint. 1 is for Normal direction
    // constraint.
    // TODO(JS): Assumed that the number of contact is not static.
    // TODO(JS): Adjust following code once use of mNumFrictionConeBases is
    //           implemented.
    //  mDim = mContacts.size() * (1 + mNumFrictionConeBases);
    mDim = 3;

    mSpatialNormalA.resize(6, 3);
    mSpatialNormalB.resize(6, 3);

    Eigen::Vector3s bodyDirectionA;
    Eigen::Vector3s bodyDirectionB;

    Eigen::Vector3s bodyPointA;
    Eigen::Vector3s bodyPointB;

    collision::Contact& ct = mContact;

    // TODO(JS): Assumed that the number of tangent basis is 2.
    const TangentBasisMatrix D = getTangentBasisMatrixODE(ct.normal);

    // Verify that normal and tangent directions are perpendicular.
    assert(abs(ct.normal.dot(D.col(0))) < DART_EPSILON);
    assert(abs(ct.normal.dot(D.col(1))) < DART_EPSILON);
    assert(abs(D.col(0).dot(D.col(1))) < DART_EPSILON);

    // Jacobian for normal contact
    bodyDirectionA.noalias()
        = mBodyNodeA->getTransform().linear().transpose() * ct.normal;
    bodyDirectionB.noalias()
        = mBodyNodeB->getTransform().linear().transpose() * -ct.normal;
    bodyPointA.noalias() = mBodyNodeA->getTransform().inverse() * ct.point;
    bodyPointB.noalias() = mBodyNodeB->getTransform().inverse() * ct.point;
    mSpatialNormalA.col(0).head<3>().noalias()
        = bodyPointA.cross(bodyDirectionA);
    mSpatialNormalB.col(0).head<3>().noalias()
        = bodyPointB.cross(bodyDirectionB);
    mSpatialNormalA.col(0).tail<3>() = bodyDirectionA;
    mSpatialNormalB.col(0).tail<3>() = bodyDirectionB;

    // Jacobian for directional friction 1
    bodyDirectionA.noalias()
        = mBodyNodeA->getTransform().linear().transpose() * D.col(0);
    bodyDirectionB.noalias()
        = mBodyNodeB->getTransform().linear().transpose() * -D.col(0);
    mSpatialNormalA.col(1).head<3>().noalias()
        = bodyPointA.cross(bodyDirectionA);
    mSpatialNormalB.col(1).head<3>().noalias()
        = bodyPointB.cross(bodyDirectionB);
    mSpatialNormalA.col(1).tail<3>() = bodyDirectionA;
    mSpatialNormalB.col(1).tail<3>() = bodyDirectionB;

    // Jacobian for directional friction 2
    bodyDirectionA.noalias()
        = mBodyNodeA->getTransform().linear().transpose() * D.col(1);
    bodyDirectionB.noalias()
        = mBodyNodeB->getTransform().linear().transpose() * -D.col(1);
    mSpatialNormalA.col(2).head<3>().noalias()
        = bodyPointA.cross(bodyDirectionA);
    mSpatialNormalB.col(2).head<3>().noalias()
        = bodyPointB.cross(bodyDirectionB);
    mSpatialNormalA.col(2).tail<3>() = bodyDirectionA;
    mSpatialNormalB.col(2).tail<3>() = bodyDirectionB;

    assert(!dart::math::isNan(mSpatialNormalA));
    assert(!dart::math::isNan(mSpatialNormalB));
  }
  else
  {
    // Set the dimension of this constraint.
    mDim = 1;

    mSpatialNormalA.resize(6, 1);
    mSpatialNormalB.resize(6, 1);

    collision::Contact& ct = mContact;

    // Contact normal in the local coordinates
    const Eigen::Vector3s bodyDirectionA
        = mBodyNodeA->getTransform().linear().transpose() * ct.normal;
    const Eigen::Vector3s bodyDirectionB
        = mBodyNodeB->getTransform().linear().transpose() * -ct.normal;

    // Contact points in the local coordinates
    const Eigen::Vector3s bodyPointA
        = mBodyNodeA->getTransform().inverse() * ct.point;
    const Eigen::Vector3s bodyPointB
        = mBodyNodeB->getTransform().inverse() * ct.point;
    mSpatialNormalA.col(0).head<3>().noalias()
        = bodyPointA.cross(bodyDirectionA);
    mSpatialNormalB.col(0).head<3>().noalias()
        = bodyPointB.cross(bodyDirectionB);
    mSpatialNormalA.col(0).tail<3>().noalias() = bodyDirectionA;
    mSpatialNormalB.col(0).tail<3>().noalias() = bodyDirectionB;
  }
}

//==============================================================================
const Eigen::MatrixXs ContactConstraint::getSpatialNormalA() const
{
  return mSpatialNormalA;
}

const Eigen::MatrixXs ContactConstraint::getSpatialNormalB() const
{
  return mSpatialNormalB;
}

//==============================================================================
void ContactConstraint::setErrorAllowance(s_t allowance)
{
  // Clamp error reduction parameter if it is out of the range
  if (allowance < 0.0)
  {
    dtwarn << "Error reduction parameter[" << allowance
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorAllowance = 0.0;
  }

  mErrorAllowance = allowance;
}

//==============================================================================
s_t ContactConstraint::getErrorAllowance()
{
  return mErrorAllowance;
}

//==============================================================================
void ContactConstraint::setErrorReductionParameter(s_t erp)
{
  // Clamp error reduction parameter if it is out of the range [0, 1]
  if (erp < 0.0)
  {
    dtwarn << "Error reduction parameter[" << erp << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mErrorReductionParameter = 0.0;
  }
  if (erp > 1.0)
  {
    dtwarn << "Error reduction parameter[" << erp << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mErrorReductionParameter = 1.0;
  }

  mErrorReductionParameter = erp;
}

//==============================================================================
s_t ContactConstraint::getErrorReductionParameter()
{
  return mErrorReductionParameter;
}

//==============================================================================
void ContactConstraint::setMaxErrorReductionVelocity(s_t erv)
{
  // Clamp maximum error reduction velocity if it is out of the range
  if (erv < 0.0)
  {
    dtwarn << "Maximum error reduction velocity[" << erv
           << "] is lower than 0.0. "
           << "It is set to 0.0." << std::endl;
    mMaxErrorReductionVelocity = 0.0;
  }

  mMaxErrorReductionVelocity = erv;
}

//==============================================================================
s_t ContactConstraint::getMaxErrorReductionVelocity()
{
  return mMaxErrorReductionVelocity;
}

//==============================================================================
void ContactConstraint::setConstraintForceMixing(s_t cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9)
  {
    dtwarn << "Constraint force mixing parameter[" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9." << std::endl;
    mConstraintForceMixing = 1e-9;
  }
  if (cfm > 1.0)
  {
    dtwarn << "Constraint force mixing parameter[" << cfm
           << "] is greater than 1.0. "
           << "It is set to 1.0." << std::endl;
    mConstraintForceMixing = 1.0;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
s_t ContactConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void ContactConstraint::setFrictionDirection(const Eigen::Vector3s& dir)
{
  mFirstFrictionalDirection = dir.normalized();
}

//==============================================================================
const Eigen::Vector3s& ContactConstraint::getFrictionDirection1() const
{
  return mFirstFrictionalDirection;
}

//==============================================================================
void ContactConstraint::update()
{
  if (mBodyNodeA->isReactive() || mBodyNodeB->isReactive())
    mActive = true;
  else
    mActive = false;
}

//==============================================================================
void ContactConstraint::getInformation(ConstraintInfo* info)
{
  // Fill w, where the LCP form is Ax = b + w (x >= 0, w >= 0, x^T w = 0)
  getRelVelocity(info->b);

  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn)
  {
    // Bias term, w, should be zero
    assert(info->w[0] == 0.0);
    assert(info->w[1] == 0.0);
    assert(info->w[2] == 0.0);

    // Upper and lower bounds of normal impulsive force
    info->lo[0] = 0.0;
    info->hi[0] = static_cast<s_t>(dInfinity);
    assert(info->findex[0] == -1);

    // Upper and lower bounds of tangential direction-1 impulsive force
    info->lo[1] = -mFrictionCoeff;
    info->hi[1] = mFrictionCoeff;
    info->findex[1] = 0;

    // Upper and lower bounds of tangential direction-2 impulsive force
    info->lo[2] = -mFrictionCoeff;
    info->hi[2] = mFrictionCoeff;
    info->findex[2] = 0;

    //------------------------------------------------------------------------
    // Bouncing
    //------------------------------------------------------------------------
    // A. Penetration correction
    s_t bouncingVelocity = mContact.penetrationDepth - mErrorAllowance;
    if (bouncingVelocity < 0.0)
    {
      bouncingVelocity = 0.0;
    }
    else
    {
      bouncingVelocity *= mErrorReductionParameter * info->invTimeStep;
      if (bouncingVelocity > mMaxErrorReductionVelocity)
        bouncingVelocity = mMaxErrorReductionVelocity;
    }

    // Penetration correction can be disabled here
    if (!mPenetrationCorrectionEnabled)
    {
      bouncingVelocity = 0;
    }

    // At this point, the bouncing velocity is exactly due to the penetration
    // correction hack saying "bouncing" should be non-zero.
    mPenetrationCorrectionVelocity = bouncingVelocity;

    // B. Restitution
    if (mIsBounceOn)
    {
      s_t& negativeRelativeVel = info->b[0];
      s_t restitutionVel = negativeRelativeVel * mRestitutionCoeff;

      mDidBounce = restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD;
      if (mDidBounce)
      {
        if (restitutionVel > bouncingVelocity)
        {
          bouncingVelocity = restitutionVel;

          if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY)
          {
            bouncingVelocity = DART_MAX_BOUNCING_VELOCITY;
          }

          // If we are actually bouncing, then that overwrites the penetration
          // correction hack.
          mPenetrationCorrectionVelocity = 0.0;
        }
      }
    }

    info->b[0] += bouncingVelocity;

    // We deliberately don't disturb info->x, because it contains last LCP's
    // solution, which is close to what we want already
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else
  {
    // Bias term, w, should be zero
    info->w[0] = 0.0;

    // Upper and lower bounds of normal impulsive force
    info->lo[0] = 0.0;
    info->hi[0] = static_cast<s_t>(dInfinity);
    assert(info->findex[0] == -1);

    //------------------------------------------------------------------------
    // Bouncing
    //------------------------------------------------------------------------
    // A. Penetration correction
    s_t bouncingVelocity = mContact.penetrationDepth - DART_ERROR_ALLOWANCE;
    if (bouncingVelocity < 0.0)
    {
      bouncingVelocity = 0.0;
    }
    else
    {
      bouncingVelocity *= mErrorReductionParameter * info->invTimeStep;
      if (bouncingVelocity > mMaxErrorReductionVelocity)
        bouncingVelocity = mMaxErrorReductionVelocity;
    }

    // Penetration correction can be disabled here
    if (!mPenetrationCorrectionEnabled)
    {
      bouncingVelocity = 0;
    }

    // At this point, the bouncing velocity is exactly due to the penetration
    // correction hack saying "bouncing" should be non-zero.
    mPenetrationCorrectionVelocity = bouncingVelocity;

    // B. Restitution
    if (mIsBounceOn)
    {
      s_t& negativeRelativeVel = info->b[0];
      s_t restitutionVel = negativeRelativeVel * mRestitutionCoeff;

      mDidBounce = restitutionVel > DART_BOUNCING_VELOCITY_THRESHOLD;
      if (mDidBounce)
      {
        if (restitutionVel > bouncingVelocity)
        {
          bouncingVelocity = restitutionVel;

          if (bouncingVelocity > DART_MAX_BOUNCING_VELOCITY)
            bouncingVelocity = DART_MAX_BOUNCING_VELOCITY;

          // If we are actually bouncing, then that overwrites the penetration
          // correction hack.
          mPenetrationCorrectionVelocity = 0.0;
        }
      }
    }

    info->b[0] += bouncingVelocity;

    // We deliberately don't disturb info->x, because it contains last LCP's
    // solution, which is close to what we want already
  }
}

//==============================================================================
void ContactConstraint::applyUnitImpulse(std::size_t index)
{
  assert(index < mDim && "Invalid Index.");
  // assert(isActive());
  assert(mBodyNodeA->isReactive() || mBodyNodeB->isReactive());

  dynamics::Skeleton* skelA = mBodyNodeA->getSkeleton().get();
  dynamics::Skeleton* skelB = mBodyNodeB->getSkeleton().get();

  // Self collision case
  if (mIsSelfCollision)
  {
    skelA->clearConstraintImpulses();

    if (mBodyNodeA->isReactive())
    {
      // Both bodies are reactive
      if (mBodyNodeB->isReactive())
      {
        skelA->updateBiasImpulse(
            mBodyNodeA,
            mSpatialNormalA.col(index),
            mBodyNodeB,
            mSpatialNormalB.col(index));
      }
      // Only body1 is reactive
      else
      {
        skelA->updateBiasImpulse(mBodyNodeA, mSpatialNormalA.col(index));
      }
    }
    else
    {
      // Only body2 is reactive
      if (mBodyNodeB->isReactive())
      {
        skelB->updateBiasImpulse(mBodyNodeB, mSpatialNormalB.col(index));
      }
      // Both bodies are not reactive
      else
      {
        // This case should not be happed
        assert(0);
      }
    }

    skelA->updateVelocityChange();
  }
  // Colliding two distinct skeletons
  else
  {
    if (mBodyNodeA->isReactive())
    {
      skelA->clearConstraintImpulses();
      skelA->updateBiasImpulse(mBodyNodeA, mSpatialNormalA.col(index));
      skelA->updateVelocityChange();
    }

    if (mBodyNodeB->isReactive())
    {
      skelB->clearConstraintImpulses();
      skelB->updateBiasImpulse(mBodyNodeB, mSpatialNormalB.col(index));
      skelB->updateVelocityChange();
    }
  }

  mAppliedImpulseIndex = index;
}

//==============================================================================
void ContactConstraint::getVelocityChange(s_t* vel, bool withCfm)
{
  assert(vel != nullptr && "Null pointer is not allowed.");

  Eigen::Map<Eigen::VectorXs> velMap(vel, static_cast<int>(mDim));
  velMap.setZero();

  if (mBodyNodeA->getSkeleton()->isImpulseApplied() && mBodyNodeA->isReactive())
    velMap += mSpatialNormalA.transpose() * mBodyNodeA->getBodyVelocityChange();

  if (mBodyNodeB->getSkeleton()->isImpulseApplied() && mBodyNodeB->isReactive())
    velMap += mSpatialNormalB.transpose() * mBodyNodeB->getBodyVelocityChange();

  // Add small values to the diagnal to keep it away from singular, similar to
  // cfm variable in ODE
  if (withCfm)
  {
    vel[mAppliedImpulseIndex]
        += vel[mAppliedImpulseIndex] * mConstraintForceMixing;
  }
}

//==============================================================================
void ContactConstraint::excite()
{
  if (mBodyNodeA->isReactive())
    mBodyNodeA->getSkeleton()->setImpulseApplied(true);

  if (mBodyNodeB->isReactive())
    mBodyNodeB->getSkeleton()->setImpulseApplied(true);
}

//==============================================================================
void ContactConstraint::unexcite()
{
  if (mBodyNodeA->isReactive())
    mBodyNodeA->getSkeleton()->setImpulseApplied(false);

  if (mBodyNodeB->isReactive())
    mBodyNodeB->getSkeleton()->setImpulseApplied(false);
}

//==============================================================================
void ContactConstraint::applyImpulse(s_t* lambda)
{
  //----------------------------------------------------------------------------
  // Friction case
  //----------------------------------------------------------------------------
  if (mIsFrictionOn)
  {
    assert(!math::isNan(lambda[0]));
    assert(!math::isNan(lambda[1]));
    assert(!math::isNan(lambda[2]));

    // Store contact impulse (force) toward the normal w.r.t. world frame
    mContact.force = mContact.normal * lambda[0] / mTimeStep;

    // Normal impulsive force
    if (mBodyNodeA->isReactive())
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA.col(0) * lambda[0]);
    if (mBodyNodeB->isReactive())
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB.col(0) * lambda[0]);

    // Add contact impulse (force) toward the tangential w.r.t. world frame
    const Eigen::MatrixXs D = getTangentBasisMatrixODE(mContact.normal);
    mContact.force += D.col(0) * lambda[1] / mTimeStep;

    // Tangential direction-1 impulsive force
    if (mBodyNodeA->isReactive())
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA.col(1) * lambda[1]);
    if (mBodyNodeB->isReactive())
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB.col(1) * lambda[1]);

    // Add contact impulse (force) toward the tangential w.r.t. world frame
    mContact.force += D.col(1) * lambda[2] / mTimeStep;

    // Tangential direction-2 impulsive force
    if (mBodyNodeA->isReactive())
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA.col(2) * lambda[2]);
    if (mBodyNodeB->isReactive())
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB.col(2) * lambda[2]);
  }
  //----------------------------------------------------------------------------
  // Frictionless case
  //----------------------------------------------------------------------------
  else
  {
    // Normal impulsive force
    if (mBodyNodeA->isReactive())
      mBodyNodeA->addConstraintImpulse(mSpatialNormalA * lambda[0]);

    if (mBodyNodeB->isReactive())
      mBodyNodeB->addConstraintImpulse(mSpatialNormalB * lambda[0]);

    // Store contact impulse (force) toward the normal w.r.t. world frame
    mContact.force = mContact.normal * lambda[0] / mTimeStep;
  }
}

//==============================================================================
void ContactConstraint::getRelVelocity(s_t* relVel)
{
  assert(relVel != nullptr && "Null pointer is not allowed.");

  Eigen::Map<Eigen::VectorXs> relVelMap(relVel, static_cast<int>(mDim));
  relVelMap.setZero();
  relVelMap -= mSpatialNormalA.transpose() * mBodyNodeA->getSpatialVelocity();
  relVelMap -= mSpatialNormalB.transpose() * mBodyNodeB->getSpatialVelocity();
}

//==============================================================================
bool ContactConstraint::isActive() const
{
  return mActive;
}

//==============================================================================
bool ContactConstraint::isFrictionOn() const
{
  return mIsFrictionOn;
}

//==============================================================================
dynamics::SkeletonPtr ContactConstraint::getRootSkeleton() const
{
  assert(isActive());

  if (mBodyNodeA->isReactive())
    return mBodyNodeA->getSkeleton()->mUnionRootSkeleton.lock();
  else
    return mBodyNodeB->getSkeleton()->mUnionRootSkeleton.lock();
}

//==============================================================================
void ContactConstraint::updateFirstFrictionalDirection()
{
  //  std::cout << "ContactConstraintTEST::_updateFirstFrictionalDirection(): "
  //            << "Not finished implementation."
  //            << std::endl;

  // TODO(JS): Not implemented
  // Refer to:
  // https://github.com/erwincoumans/bullet3/blob/master/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp#L910
  //  mFirstFrictionalDirection;
}

//==============================================================================
ContactConstraint::TangentBasisMatrix
ContactConstraint::getTangentBasisMatrixODE(const Eigen::Vector3s& n)
{
  using namespace math::suffixes;

  // TODO(keenon): find a smoother way to handle the tangent basis. Will need to
  // be path dependent, because this is in general impossible without path
  // dependence: https://scicomp.stackexchange.com/a/27843

  // TODO(JS): Use mNumFrictionConeBases
  // Check if the number of bases is even number.
  //  bool isEvenNumBases = mNumFrictionConeBases % 2 ? true : false;
  assert(n.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED);

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3s tangent = mFirstFrictionalDirection.cross(n);

  // TODO(JS): Modify following lines once _updateFirstFrictionalDirection() is
  //           implemented.
  // If they're too close (or opposing directions, or one of the vectors 0),
  // pick another tangent (use X-axis as arbitrary vector)
  if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
  {
    tangent = Eigen::Vector3s::UnitX().cross(n);

    // Make sure this is not zero length, otherwise normalization will lead to
    // NaN values.
    if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    {
      tangent = Eigen::Vector3s::UnitY().cross(n);
      if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
      {
        tangent = Eigen::Vector3s::UnitZ().cross(n);

        // Now tangent shouldn't be zero-length unless the normal is
        // zero-length, which shouldn't the case because ConstraintSolver
        // shouldn't create a ContactConstraint for a contact with zero-length
        // normal.
        assert(
            tangent.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED);
      }
    }
  }

  assert(tangent.norm() > 1e-06);
  tangent.normalize();

  assert(!dart::math::isNan(tangent));

  TangentBasisMatrix T;
  T.setZero();

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times
  T.col(0) = tangent;
  T.col(1) = n.cross(
      tangent); // Eigen::Quaternion_s(Eigen::AngleAxis_s(0.5_pi, n)) * tangent;
  return T;
}

//==============================================================================
/// This returns the gradient of each element of the Tangent basis matrix, if
/// `g` is the gradient of `n` with respect to whatever scalar we care about.
ContactConstraint::TangentBasisMatrix
ContactConstraint::getTangentBasisMatrixODEGradient(
    const Eigen::Vector3s& n, const Eigen::Vector3s& g)
{
  using namespace math::suffixes;

  // TODO(JS): Use mNumFrictionConeBases
  // Check if the number of bases is even number.
  //  bool isEvenNumBases = mNumFrictionConeBases % 2 ? true : false;

  // Pick an arbitrary vector to take the cross product of (in this case,
  // Z-axis)
  Eigen::Vector3s cross = mFirstFrictionalDirection;
  Eigen::Vector3s tangent = cross.cross(n);

  // TODO(JS): Modify following lines once _updateFirstFrictionalDirection() is
  //           implemented.
  // If they're too close (or opposing directions, or one of the vectors 0),
  // pick another tangent (use X-axis as arbitrary vector)
  if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
  {
    cross = Eigen::Vector3s::UnitX();
    tangent = cross.cross(n);

    // Make sure this is not zero length, otherwise normalization will lead to
    // NaN values.
    if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    {
      cross = Eigen::Vector3s::UnitY();
      tangent = cross.cross(n);
      if (tangent.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
      {
        cross = Eigen::Vector3s::UnitZ();
        tangent = cross.cross(n);

        // Now tangent shouldn't be zero-length unless the normal is
        // zero-length, which shouldn't the case because ConstraintSolver
        // shouldn't create a ContactConstraint for a contact with zero-length
        // normal.
        assert(
            tangent.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED);
      }
    }
  }

  s_t tangentNorm = tangent.norm();
  assert(tangentNorm > 1e-06);
  tangent.normalize();

  assert(!dart::math::isNan(tangent));

  TangentBasisMatrix T;
  T.setZero();

  // Rotate the tangent around the normal to compute bases.
  // Note: a possible speedup is in place for mNumDir % 2 = 0
  // Each basis and its opposite belong in the matrix, so we iterate half as
  // many times
  Eigen::Vector3s gradOfFrictionalDirectionCrossNormal = cross.cross(g);
  gradOfFrictionalDirectionCrossNormal /= tangentNorm;
  // Account for the normalization
  Eigen::Vector3s gradOfTangent;
  if (abs(tangentNorm - 1.0) > DART_EPSILON)
  {
    gradOfTangent
        = gradOfFrictionalDirectionCrossNormal
          - ((gradOfFrictionalDirectionCrossNormal.dot(tangent)) * tangent);
  }
  else
  {
    gradOfTangent = gradOfFrictionalDirectionCrossNormal;
  }

  T.col(0) = gradOfTangent;
  T.col(1) = g.cross(tangent) + n.cross(gradOfTangent);
  return T;
}

//==============================================================================
void ContactConstraint::uniteSkeletons()
{
  if (!mBodyNodeA->isReactive() || !mBodyNodeB->isReactive())
    return;

  if (mBodyNodeA->getSkeleton() == mBodyNodeB->getSkeleton())
    return;

  const dynamics::SkeletonPtr& unionIdA
      = ConstraintBase::compressPath(mBodyNodeA->getSkeleton());
  const dynamics::SkeletonPtr& unionIdB
      = ConstraintBase::compressPath(mBodyNodeB->getSkeleton());

  if (unionIdA == unionIdB)
    return;

  if (unionIdA->mUnionSize < unionIdB->mUnionSize)
  {
    // Merge root1 --> root2
    unionIdA->mUnionRootSkeleton = unionIdB;
    unionIdB->mUnionSize += unionIdA->mUnionSize;
  }
  else
  {
    // Merge root2 --> root1
    unionIdB->mUnionRootSkeleton = unionIdA;
    unionIdA->mUnionSize += unionIdB->mUnionSize;
  }
}

//==============================================================================
std::vector<dynamics::SkeletonPtr> ContactConstraint::getSkeletons() const
{
  std::vector<dynamics::SkeletonPtr> skeletons;

  if (mBodyNodeA->isReactive())
    skeletons.push_back(mBodyNodeA->getSkeleton());
  if (mBodyNodeB->isReactive())
    skeletons.push_back(mBodyNodeB->getSkeleton());

  return skeletons;
}

//==============================================================================
const collision::Contact& ContactConstraint::getContact() const
{
  return mContact;
}

//==============================================================================
const dynamics::BodyNode* ContactConstraint::getBodyNodeA() const
{
  return mBodyNodeA;
}

//==============================================================================
const dynamics::BodyNode* ContactConstraint::getBodyNodeB() const
{
  return mBodyNodeB;
}

//==============================================================================
bool ContactConstraint::isContactConstraint() const
{
  return true;
}

//==============================================================================
s_t ContactConstraint::getCoefficientOfRestitution()
{
  if (!mIsBounceOn || !mDidBounce)
    return 0;
  else
    return mRestitutionCoeff;
}

//==============================================================================
s_t ContactConstraint::getPenetrationCorrectionVelocity()
{
  return mPenetrationCorrectionVelocity;
}

} // namespace constraint
} // namespace dart