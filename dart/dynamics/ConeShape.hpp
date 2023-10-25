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

#ifndef DART_DYNAMICS_CONESHAPE_HPP_
#define DART_DYNAMICS_CONESHAPE_HPP_

#include "dart/dynamics/Shape.hpp"
#include "dart/math/MathTypes.hpp"

namespace dart {
namespace dynamics {

/// ConeShape represents a three-dimensional geometric shape that tapers
/// smoothly from a flat circular base to a point called the apex or vertex.
class ConeShape : public Shape
{
public:
  /// Constructor.
  /// \param[in] radius Radius of the circular base.
  /// \param[in] height Lateral height of the cone.
  ConeShape(s_t radius, s_t height);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Get shape type string for this shape.
  static const std::string& getStaticType();

  /// Get the radius of the circular base.
  s_t getRadius() const;

  /// Set the radius of the circular base.
  void setRadius(s_t radius);

  /// Get the lateral height of the cone.
  s_t getHeight() const;

  /// Set the lateral height of the cone.
  void setHeight(s_t height);

  /// Compute volume from given properties.
  /// \param[in] radius Radius of the circular base.
  /// \param[in] height Lateral height of the cone.
  static s_t computeVolume(s_t radius, s_t height);

  /// Compute moments of inertia of a cone at the center of geometric center
  /// (half of the z-axis segment between the tip and the center of the base
  /// disk).
  /// \param[in] radius Radius of the circular base.
  /// \param[in] height Lateral height of the cone.
  /// \param[in] mass The mass of the cone.
  static Eigen::Matrix3s computeInertia(s_t radius, s_t height, s_t mass);

  // Documentation inherited.
  Eigen::Matrix3s computeInertia(s_t mass) const override;

  /// Allow us to clone shapes, to avoid race conditions when scaling shapes
  /// belonging to different skeletons
  ShapePtr clone() const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

private:
  /// Radius of the circular base.
  s_t mRadius;

  /// Height of the cylindrical part.
  s_t mHeight;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_CONESHAPE_HPP_
