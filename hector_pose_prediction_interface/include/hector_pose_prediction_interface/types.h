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

#include <hector_stability_metrics/support_polygon.h>
#include <hector_stability_metrics/types.h>
#include <Eigen/Geometry>

namespace hector_pose_prediction_interface
{

using hector_stability_metrics::SupportPolygon;
using hector_stability_metrics::SupportPolygonf;
using hector_stability_metrics::SupportPolygond;

using hector_stability_metrics::Vector3;
using hector_stability_metrics::Vector3f;
using hector_stability_metrics::Vector3d;
using hector_stability_metrics::Vector3List;

using hector_stability_metrics::Isometry3;
using hector_stability_metrics::Isometry3f;
using hector_stability_metrics::Isometry3d;

}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H