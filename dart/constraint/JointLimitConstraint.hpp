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

#ifndef DART_CONSTRAINT_JOINTLIMITCONSTRAINT_HPP_
#define DART_CONSTRAINT_JOINTLIMITCONSTRAINT_HPP_

#include "dart/constraint/ConstraintBase.hpp"

namespace dart {

namespace dynamics {
class BodyNode;
class Joint;
}  // namespace dynamics

namespace constraint {

/// JointLimitConstraint handles joint position or velocity limits
// TOOD: better naming
class JointLimitConstraint : public ConstraintBase
{
public:
  /// Constructor
  explicit JointLimitConstraint(dynamics::Joint* _joint);

  /// Destructor
  virtual ~JointLimitConstraint();

  //----------------------------------------------------------------------------
  // Property settings
  //----------------------------------------------------------------------------

  /// Set global error reduction parameter
  static void setErrorAllowance(s_t _allowance);

  /// Get global error reduction parameter
  static s_t getErrorAllowance();

  /// Set global error reduction parameter
  static void setErrorReductionParameter(s_t _erp);

  /// Get global error reduction parameter
  static s_t getErrorReductionParameter();

  /// Set global error reduction parameter
  static void setMaxErrorReductionVelocity(s_t _erv);

  /// Get global error reduction parameter
  static s_t getMaxErrorReductionVelocity();

  /// Set global constraint force mixing parameter
  static void setConstraintForceMixing(s_t _cfm);

  /// Get global constraint force mixing parameter
  static s_t getConstraintForceMixing();

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;
  friend class ConstrainedGroup;

protected:
  //----------------------------------------------------------------------------
  // Constraint virtual functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(ConstraintInfo* _lcp) override;

  // Documentation inherited
  void applyUnitImpulse(std::size_t _index) override;

  // Documentation inherited
  void getVelocityChange(s_t* _delVel, bool _withCfm) override;

  // Documentation inherited
  void excite() override;

  // Documentation inherited
  void unexcite() override;

  // Documentation inherited
  void applyImpulse(s_t* _lambda) override;

  // Documentation inherited
  dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  bool isActive() const override;

private:
  ///
  dynamics::Joint* mJoint;

  ///
  dynamics::BodyNode* mBodyNode;

  /// Index of applied impulse
  std::size_t mAppliedImpulseIndex;

  ///
  std::size_t mLifeTime[6];

  ///
  bool mActive[6];

  ///
  s_t mViolation[6];

  ///
  s_t mNegativeVel[6];

  ///
  s_t mOldX[6];

  ///
  s_t mUpperBound[6];

  ///
  s_t mLowerBound[6];

  /// Global constraint error allowance
  static s_t mErrorAllowance;

  /// Global constraint error redection parameter in the range of [0, 1]. The
  /// default is 0.01.
  static s_t mErrorReductionParameter;

  /// Maximum error reduction velocity
  static s_t mMaxErrorReductionVelocity;

  /// Global constraint force mixing parameter in the range of [1e-9, 1]. The
  /// default is 1e-5
  /// \sa http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
  static s_t mConstraintForceMixing;
};

}  // namespace constraint
}  // namespace dart

#endif  // DART_CONSTRAINT_JOINTLIMITCONSTRAINT_HPP_

