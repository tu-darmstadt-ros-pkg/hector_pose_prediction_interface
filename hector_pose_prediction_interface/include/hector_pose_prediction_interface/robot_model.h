//
// Created by Stefan Fabian on 16.09.2020.
//

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_ROBOT_MODEL_H
#define HECTOR_POSE_PREDICTION_INTERFACE_ROBOT_MODEL_H

#include "hector_pose_prediction_interface/types.h"
#include <memory>
#include <unordered_map>

namespace hector_pose_prediction_interface
{

template<typename Scalar>
class RobotModel
{
public:
  using Ptr = std::shared_ptr<RobotModel<Scalar> >;
  using ConstPtr = std::shared_ptr<const RobotModel<Scalar> >;

  //! Updates the joint positions of the robot model with the given values for the corresponding joint name.
  virtual void updateJointPositions( const std::unordered_map<std::string, double> &joint_positions ) = 0;

  //! Updates the joint positions of the robot model with the given values.
  //! Indices have to correspond to names in getJointNames().
  virtual void updateJointPositions( const std::vector<double> &positions ) = 0;

  //! The names of the joints represented in this robot model.
  virtual std::vector<std::string> getJointNames() = 0;

  //! The position of the center of mass in the robot coordinate frame.
  virtual Vector3<Scalar> centerOfMass() = 0;
};
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_ROBOT_MODEL_H