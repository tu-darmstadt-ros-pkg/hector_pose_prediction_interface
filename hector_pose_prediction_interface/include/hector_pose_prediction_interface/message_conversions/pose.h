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

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_MESSAGE_CONVERSIONS_POSE_H
#define HECTOR_POSE_PREDICTION_INTERFACE_MESSAGE_CONVERSIONS_POSE_H

#include "hector_pose_prediction_interface/types.h"

#include <hector_stability_metrics/message_conversions/eigen.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

namespace hector_pose_prediction_interface
{
/*!
 * IMPORTANT: These conversion methods are provided as headers but the library does not depend on the message types.
 *   Hence, make sure that your project depends on them if you use them.
 */
namespace message_conversions
{
template<typename Scalar>
Pose<Scalar> msgToPose( const geometry_msgs::Pose &msg )
{
  using namespace hector_stability_metrics::message_conversions;
  return { msgToVector<Scalar>( msg.position ), msgToQuaternion<Scalar>( msg.orientation ) };
}

template<typename Scalar>
Pose<Scalar> msgToPose( const geometry_msgs::Transform &msg )
{
  using namespace hector_stability_metrics::message_conversions;
  return { msgToVector<Scalar>( msg.translation ), msgToQuaternion<Scalar>( msg.rotation ) };
}

template<typename Scalar>
geometry_msgs::Pose poseToPoseMsg( const Pose<Scalar> &pose )
{
  using namespace hector_stability_metrics::message_conversions;
  geometry_msgs::Pose msg;
  msg.position = vectorToPointMsg( pose.translation());
  msg.orientation = quaternionToMsg( pose.orientation());
  return msg;
}

template<typename Scalar>
geometry_msgs::Transform poseToTransformMsg( const Pose<Scalar> &pose )
{
  using namespace hector_stability_metrics::message_conversions;
  geometry_msgs::Transform msg;
  msg.translation = vectorToVectorMsg( pose.translation());
  msg.rotation = quaternionToMsg( pose.orientation());
  return msg;
}
}  // namespace message_conversions
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_MESSAGE_CONVERSIONS_POSE_H