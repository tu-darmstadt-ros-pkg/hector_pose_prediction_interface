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

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_CONTACT_INFORMATION_H
#define HECTOR_POSE_PREDICTION_INTERFACE_CONTACT_INFORMATION_H

#include "hector_pose_prediction_interface/types.h"

namespace hector_pose_prediction_interface
{

namespace joint_types
{
enum JointType : int
{
  Undefined = 0,
  Tracks = 1,
  Chassis = 2,
  Fragile = 4,
  //! If you want to define custom types use values between 1024 and 2047
  Custom = 1024
};
}
using JointType = joint_types::JointType;

template<typename Scalar>
struct ContactPointInformation
{
  Vector3<Scalar> point;
  Vector3<Scalar> surface_normal;
  int joint_type;
};

template<typename Scalar>
struct ContactInformation
{
  std::vector<ContactPointInformation<Scalar> > contact_points;
};
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_CONTACT_INFORMATION_H