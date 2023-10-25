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

#include "dart/dynamics/BoxShape.hpp"

#include <cmath>

namespace dart {
namespace dynamics {

//==============================================================================
BoxShape::BoxShape(const Eigen::Vector3s& _size) : Shape(BOX), mSize(_size)
{
  assert(_size[0] > 0.0);
  assert(_size[1] > 0.0);
  assert(_size[2] > 0.0);
}

//==============================================================================
BoxShape::~BoxShape()
{
  // Do nothing
}

//==============================================================================
const std::string& BoxShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& BoxShape::getStaticType()
{
  static const std::string type("BoxShape");
  return type;
}

//==============================================================================
s_t BoxShape::computeVolume(const Eigen::Vector3s& size)
{
  return size[0] * size[1] * size[2];
}

//==============================================================================
Eigen::Matrix3s BoxShape::computeInertia(const Eigen::Vector3s& size, s_t mass)
{
  Eigen::Matrix3s inertia = Eigen::Matrix3s::Identity();

  inertia(0, 0) = mass / 12.0 * (pow(size[1], 2) + pow(size[2], 2));
  inertia(1, 1) = mass / 12.0 * (pow(size[0], 2) + pow(size[2], 2));
  inertia(2, 2) = mass / 12.0 * (pow(size[0], 2) + pow(size[1], 2));

  return inertia;
}

//==============================================================================
void BoxShape::setSize(const Eigen::Vector3s& _size)
{
  assert(_size[0] > 0.0);
  assert(_size[1] > 0.0);
  assert(_size[2] > 0.0);
  mSize = _size;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
const Eigen::Vector3s& BoxShape::getSize() const
{
  return mSize;
}

//==============================================================================
Eigen::Matrix3s BoxShape::computeInertia(s_t mass) const
{
  return computeInertia(mSize, mass);
}

//==============================================================================
/// Allow us to clone shapes, to avoid race conditions when scaling shapes
/// belonging to different skeletons
ShapePtr BoxShape::clone() const
{
  return std::make_shared<BoxShape>(mSize);
}

//==============================================================================
void BoxShape::updateBoundingBox() const
{
  mBoundingBox.setMin(-mSize * 0.5);
  mBoundingBox.setMax(mSize * 0.5);
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void BoxShape::updateVolume() const
{
  mVolume = computeVolume(mSize);
  mIsVolumeDirty = false;
}

} // namespace dynamics
} // namespace dart