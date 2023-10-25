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

#ifndef DART_DYNAMICS_DETAIL_JOINTASPECT_HPP_
#define DART_DYNAMICS_DETAIL_JOINTASPECT_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dart/math/MathTypes.hpp"

namespace dart {
namespace dynamics {
namespace detail {

/// Actuator type
///
/// The command is taken by setCommand() or setCommands(), and the meaning of
/// command is different depending on the actuator type. The default actuator
/// type is FORCE. (TODO: FreeJoint should be PASSIVE?)
///
/// FORCE/PASSIVE/SERVO/MIMIC joints are dynamic joints while
/// ACCELERATION/VELOCITY/LOCKED joints are kinematic joints.
///
/// Note the presence of joint damping force and joint spring force for all
/// the actuator types if the coefficients are non-zero. The default
/// coefficients are zero.
///
/// \sa setActuatorType(), getActuatorType(),
/// setSpringStiffness(), setDampingCoefficient(),
enum ActuatorType
{
  /// Command input is joint force, and the output is joint acceleration.
  ///
  /// If the command is zero, then it's identical to passive joint. The valid
  /// joint constraints are position limit, velocity limit, and Coulomb
  /// friction, and the invalid joint constraint is force limit.
  FORCE,

  /// Passive joint doesn't take any command input, and the output is joint
  /// acceleration.
  ///
  /// The valid joint constraints are position limit, velocity limit, and
  /// Coulomb friction, and the invalid joint constraint is force limit.
  PASSIVE,

  /// Command input is desired velocity, and the output is joint acceleration.
  ///
  /// The constraint solver will try to track the desired velocity within the
  /// joint force limit. All the joint constraints are valid.
  SERVO,

  /// There is no command input. The joint tries to follow the position of
  /// another joint (the mimic joint) by computing desired velocities.
  /// The output is joint acceleration.
  ///
  /// The constraint solver will try to track the desired velocity within the
  /// joint force limit. All the joint constraints are valid.
  MIMIC,

  /// Command input is joint acceleration, and the output is joint force.
  ///
  /// The joint acceleration is always satisfied but it doesn't take the joint
  /// force limit into account. All the joint constraints are invalid.
  ACCELERATION,

  /// Command input is joint velocity, and the output is joint force.
  ///
  /// The joint velocity is always satisfied but it doesn't take the joint
  /// force limit into account. If you want to consider the joint force limit,
  /// should use SERVO instead. All the joint constraints are invalid.
  VELOCITY,

  /// Locked joint always set the velocity and acceleration to zero so that
  /// the joint dosen't move at all (locked), and the output is joint force.
  /// force.
  ///
  /// All the joint constraints are invalid.
  LOCKED
};

const ActuatorType DefaultActuatorType = FORCE;

struct JointProperties
{
  /// Joint name
  std::string mName;

  /// Transformation from parent BodyNode to this Joint
  Eigen::Isometry3s mT_ParentBodyToJoint;

  /// Transformation from child BodyNode to this Joint
  Eigen::Isometry3s mT_ChildBodyToJoint;

  /// This holds the relative scale we're applying to the offsets of the parent
  /// transform. Do not use directly! Use setParentScale() and getParentScale()
  /// instead
  Eigen::Vector3s mParentScale;

  /// This holds the original translation from the parent, so that we can
  /// reconstruct exactly what the translation is as we change the scale of
  /// objects. This avoids numerical issues with roundoff during scaling messing
  /// with finite differencing.
  Eigen::Vector3s mOriginalParentTranslation;

  /// This holds the relative scale we're applying to the offsets of the child
  /// transform. Do not use directly! Use setChildScale() and getChildScale()
  /// instead
  Eigen::Vector3s mChildScale;

  /// This holds the original translation from the child, so that we can
  /// reconstruct exactly what the translation is as we change the scale of
  /// objects. This avoids numerical issues with roundoff during scaling messing
  /// with finite differencing.
  Eigen::Vector3s mOriginalChildTranslation;

  /// True if the joint limits should be enforced in dynamic simulation
  bool mIsPositionLimitEnforced;

  /// Actuator type
  ActuatorType mActuatorType;

  /// Mimic joint
  const Joint* mMimicJoint;

  /// Mimic joint properties
  s_t mMimicMultiplier, mMimicOffset;

  /// Constructor
  JointProperties(
      const std::string& _name = "Joint",
      const Eigen::Isometry3s& _T_ParentBodyToJoint
      = Eigen::Isometry3s::Identity(),
      const Eigen::Isometry3s& _T_ChildBodyToJoint
      = Eigen::Isometry3s::Identity(),
      bool _isPositionLimitEnforced = false,
      ActuatorType _actuatorType = DefaultActuatorType,
      const Joint* _mimicJoint = nullptr,
      s_t _mimicMultiplier = 1.0,
      s_t _mimicOffset = 0.0);

  virtual ~JointProperties() = default;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_JOINTASPECT_HPP_
