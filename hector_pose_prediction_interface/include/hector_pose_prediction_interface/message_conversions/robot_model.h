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

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_MESSAGE_CONVERSIONS_ROBOT_MODEL_H
#define HECTOR_POSE_PREDICTION_INTERFACE_MESSAGE_CONVERSIONS_ROBOT_MODEL_H

#include "hector_pose_prediction_interface/robot_model.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayRobotState.h>

namespace hector_pose_prediction_interface
{
/*!
 * IMPORTANT: These conversion methods are provided as headers but the library does not depend on the message types.
 *   Hence, make sure that your project depends on them if you use them.
 */
namespace message_conversions
{

template<typename Scalar>
moveit::core::RobotState robotModelToRobotState( const RobotModel <Scalar> &model, const urdf::ModelSharedPtr &urdf,
                                                 const srdf::ModelSharedPtr &srdf = std::make_shared<srdf::Model>())
{
  moveit::core::RobotModelPtr robot_model = std::make_shared<moveit::core::RobotModel>( urdf, srdf );
  moveit::core::RobotState state( robot_model );
  const auto &names = model.getJointNames();
  const auto &positions = model.getJointPositions();
  const auto &variable_names = state.getVariableNames();
  for ( int i = 0; i < names.size(); ++i )
  {
    if ( std::find( variable_names.begin(), variable_names.end(), names[i] ) == variable_names.end()) continue;
    state.setVariablePosition( names[i], positions[i] );
  }
  state.update();
  return state;
}
}
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_MESSAGE_CONVERSIONS_ROBOT_MODEL_H