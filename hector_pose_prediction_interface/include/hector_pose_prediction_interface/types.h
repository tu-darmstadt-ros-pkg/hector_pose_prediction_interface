// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H
#define HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H

#include <Eigen/Geometry>
#include <hector_math/types.h>

namespace hector_pose_prediction_interface
{

template<typename Scalar>
struct SupportPolygon {
  //! The convex hull of contact points making up the support polygon.
  hector_math::Vector3List<Scalar> contact_hull_points;
  //! The stability for each edge of the support polygon formed by the contact_hull_points.
  //! First value is the stability of the edge from the first point at index 0 to the second point at index 1.
  std::vector<Scalar> edge_stabilities;
};

namespace contact_information_flags
{
enum ContactInformationFlags : unsigned short {
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
enum JointType : unsigned int {
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
struct ContactPointInformation {
  hector_math::Vector3<Scalar> point;
  hector_math::Vector3<Scalar> surface_normal;
  JointType joint_type;
};

template<typename Scalar>
struct ContactInformation {
  std::vector<ContactPointInformation<Scalar>> contact_points;
};

// Define operators for contact flags
inline ContactInformationFlags operator&( ContactInformationFlags a, ContactInformationFlags b )
{
  return static_cast<ContactInformationFlags>( static_cast<unsigned short>( a ) &
                                               static_cast<unsigned short>( b ) );
}

inline ContactInformationFlags operator|( ContactInformationFlags a, ContactInformationFlags b )
{
  return static_cast<ContactInformationFlags>( static_cast<unsigned short>( a ) |
                                               static_cast<unsigned short>( b ) );
}

inline ContactInformationFlags operator^( ContactInformationFlags a, ContactInformationFlags b )
{
  return static_cast<ContactInformationFlags>( static_cast<unsigned short>( a ) ^
                                               static_cast<unsigned short>( b ) );
}

inline ContactInformationFlags operator~( ContactInformationFlags a )
{
  return static_cast<ContactInformationFlags>( ~static_cast<unsigned short>( a ) );
}

inline ContactInformationFlags &operator&=( ContactInformationFlags &a, ContactInformationFlags b )
{
  return a = static_cast<ContactInformationFlags>( static_cast<unsigned short>( a ) &
                                                   static_cast<unsigned short>( b ) );
}

inline ContactInformationFlags &operator|=( ContactInformationFlags &a, ContactInformationFlags b )
{
  return a = static_cast<ContactInformationFlags>( static_cast<unsigned short>( a ) |
                                                   static_cast<unsigned short>( b ) );
}

inline ContactInformationFlags &operator^=( ContactInformationFlags &a, ContactInformationFlags b )
{
  return a = static_cast<ContactInformationFlags>( static_cast<unsigned short>( a ) ^
                                                   static_cast<unsigned short>( b ) );
}
} // namespace hector_pose_prediction_interface

#endif // HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H