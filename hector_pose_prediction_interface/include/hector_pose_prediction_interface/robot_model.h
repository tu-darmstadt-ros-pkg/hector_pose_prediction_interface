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
  void updateJointPositions( const std::unordered_map<std::string, Scalar> &positions )
  {
    doUpdateJointPositions( positions );
  }

  //! Updates the joint positions of the robot model with the given values.
  //! Indices have to correspond to names in getJointNames().
  void updateJointPositions( const std::vector<Scalar> &positions ) { doUpdateJointPositions( positions ); }

  //! The names of the joints represented in this robot model.
  std::vector<std::string> getJointNames() { return doGetJointNames(); }

  //! The position of the center of mass in the robot coordinate frame.
  Vector3<Scalar> centerOfMass() { return computeCenterOfMass(); }

private:
  virtual void doUpdateJointPositions( const std::unordered_map<std::string, Scalar> &joint_positions ) = 0;

  virtual void doUpdateJointPositions( const std::vector<Scalar> &positions ) = 0;

  virtual std::vector<std::string> doGetJointNames() = 0;

  virtual Vector3<Scalar> computeCenterOfMass() = 0;
};
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_ROBOT_MODEL_H