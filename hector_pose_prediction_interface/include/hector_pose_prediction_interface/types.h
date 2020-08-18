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

#include <hector_stability_metrics/math/types.h>
#include <hector_stability_metrics/support_polygon.h>
#include <Eigen/Geometry>

namespace hector_pose_prediction_interface
{
template <typename Scalar>
using SupportPolygon = hector_stability_metrics::SupportPolygon<Scalar>;

using SupportPolygonf = hector_stability_metrics::SupportPolygonf;
using SupportPolygond = hector_stability_metrics::SupportPolygond;

template <typename Scalar>
using Vector3 = hector_stability_metrics::Vector3<Scalar>;
using Vector3f = hector_stability_metrics::Vector3f;
using Vector3d = hector_stability_metrics::Vector3d;
template <typename Scalar>
using Vector3List = hector_stability_metrics::Vector3List<Scalar>;

template <typename Scalar>
using Isometry3 = hector_stability_metrics::Isometry3<Scalar>;
using Isometry3f = hector_stability_metrics::Isometry3f;
using Isometry3d = hector_stability_metrics::Isometry3d;

}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H