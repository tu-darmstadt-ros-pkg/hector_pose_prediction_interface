/*
 * Copyright (C) 2020  Stefan Fabian, Martin Oehler
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

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_POSE_PREDICTOR_H
#define HECTOR_POSE_PREDICTION_INTERFACE_POSE_PREDICTOR_H

#include "hector_pose_prediction_interface/types.h"

namespace hector_pose_prediction_interface
{

/*!
 * Interface for a pose predictor implementation which based on an input pose, e.g., a position and identity rotation,
 * estimates the pose of the robot at the given position in the world.
 * @tparam Scalar The floating point type used for calculations usually float or double.
 */
template<typename Scalar>
class PosePredictor
{
public:
  using Ptr = std::shared_ptr<PosePredictor<Scalar> >;
  using ConstPtr = std::shared_ptr<const PosePredictor<Scalar> >;

  /*!
   * Predict the pose of the robot and the support polygon with the passed pose as initial guess.
   * For example, by tipping the robot over unstable axes until a stable pose was found.
   * @param pose Used as both input and output. An isometric transform consisting of a translation and a rotation.
   * @param support_polygon The resulting support polygon if a stable pose was found.
   * @return A value indicating the stability where greater values mean higher stability and negative values or NaN
   *   indicate that there was no stable pose found.
   */
  virtual Scalar predictPoseAndSupportPolygon( Isometry3<Scalar> &pose,
                                               SupportPolygon<Scalar> &support_polygon ) = 0;

  /*!
   * Predict the pose of the robot using the passed pose as initial guess.
   * @param pose Used as both input and output. An isometric transform consisting of a translation and a rotation.
   * @return A value indicating the stability where greater values mean higher stability and negative values or NaN
   *   indicate that there was no stable pose found.
   */
  virtual Scalar predictPose( Isometry3<Scalar> &pose ) = 0;

  /*!
   * Estimate the support polygon for a given pose.
   * @param pose The pose of the robot.
   * @param support_polygon The estimated support polygon.
   * @return True if a valid support polygon was estimated, false, otherwise, e.g., because there were only one or two
   *   contact points.
   */
  virtual bool estimateSupportPolygon( const Isometry3<Scalar> &pose,
                                       SupportPolygon<Scalar> &support_polygon ) = 0;
};
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_POSE_PREDICTOR_H
