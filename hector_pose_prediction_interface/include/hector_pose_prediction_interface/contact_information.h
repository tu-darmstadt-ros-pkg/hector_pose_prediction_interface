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

namespace contact_information_flags
{
enum ContactInformationFlags : unsigned short
{
  None = 0,
  JointType = 0x1,
  Point = 0x2,
  SurfaceNormal = 0x4,
  All = 0xffff
};
}
using ContactInformationFlags = contact_information_flags::ContactInformationFlags;

namespace joint_types
{
enum JointType : unsigned int
{
  Undefined = 0,
  Tracks = 1,
  Chassis = 2, // Note: Despite the pattern this is not a flags enum
  Fragile = 8,
  //! If you want to define custom types use values between 1024 and 2047 so checking for the Custom bit is possible
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

// Define operators for contact flags
ContactInformationFlags operator&( ContactInformationFlags a, ContactInformationFlags b )
{
  return static_cast<ContactInformationFlags>(static_cast<unsigned short>(a) & static_cast<unsigned short>(b));
}

ContactInformationFlags operator|( ContactInformationFlags a, ContactInformationFlags b )
{
  return static_cast<ContactInformationFlags>(static_cast<unsigned short>(a) | static_cast<unsigned short>(b));
}

ContactInformationFlags operator^( ContactInformationFlags a, ContactInformationFlags b )
{
  return static_cast<ContactInformationFlags>(static_cast<unsigned short>(a) ^ static_cast<unsigned short>(b));
}

ContactInformationFlags operator~( ContactInformationFlags a )
{
  return static_cast<ContactInformationFlags>(~static_cast<unsigned short>(a));
}

ContactInformationFlags &operator&=( ContactInformationFlags &a, ContactInformationFlags b )
{
  return a = static_cast<ContactInformationFlags>(static_cast<unsigned short>(a) & static_cast<unsigned short>(b));
}

ContactInformationFlags &operator|=( ContactInformationFlags &a, ContactInformationFlags b )
{
  return a = static_cast<ContactInformationFlags>(static_cast<unsigned short>(a) | static_cast<unsigned short>(b));
}

ContactInformationFlags &operator^=( ContactInformationFlags &a, ContactInformationFlags b )
{
  return a = static_cast<ContactInformationFlags>(static_cast<unsigned short>(a) ^ static_cast<unsigned short>(b));
}
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_CONTACT_INFORMATION_H