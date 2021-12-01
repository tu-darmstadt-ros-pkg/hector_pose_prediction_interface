/*
 * Copyright (C) 2020  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H
#define HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H

#include <hector_math/types.h>
#include <Eigen/Geometry>

namespace hector_pose_prediction_interface
{
namespace math
{
using hector_math::Vector2;
using hector_math::Vector2f;
using hector_math::Vector2d;
using hector_math::Vector2List;

using hector_math::Vector3;
using hector_math::Vector3f;
using hector_math::Vector3d;
using hector_math::Vector3List;

using hector_math::Isometry3;
using hector_math::Isometry3f;
using hector_math::Isometry3d;

template<typename Scalar>
struct SupportPolygon
{
  //! The convex hull of contact points making up the support polygon.
  Vector3List<Scalar> contact_hull_points;
  //! The stability for each edge of the support polygon formed by the contact_hull_points.
  //! First value is the stability of the edge from the first point at index 0 to the second point at index 1.
  std::vector<Scalar> edge_stabilities;
};

using hector_math::Pose;
}
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H