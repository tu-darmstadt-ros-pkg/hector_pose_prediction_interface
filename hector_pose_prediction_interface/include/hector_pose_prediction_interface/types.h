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

//! Represents the pose of a robot as a combination of orientation quaternion and translation vector.
template<typename Scalar>
class Pose
{
public:
  static Pose<Scalar> Identity()
  {
    Pose<Scalar> result;
    result.translation_ = Vector3<Scalar>::Zero();
    result.orientation_ = Eigen::Quaternion<Scalar>::Identity();
    return result;
  }

  static Pose<Scalar> Origin() { return Identity(); }

  static Pose<Scalar> Translation( Scalar x, Scalar y, Scalar z )
  {
    return Translation( Vector3<Scalar>( x, y, z ));
  }

  static Pose<Scalar> Translation( const Vector3<Scalar> &translation )
  {
    Pose<Scalar> result;
    result.translation_ = translation;
    result.orientation_ = Eigen::Quaternion<Scalar>::Identity();
    return result;
  }

  Pose() = default;

  Pose( Scalar tx, Scalar ty, Scalar tz, Scalar qw, Scalar qx, Scalar qy, Scalar qz )
  {
    translation_ = { tx, ty, tz };
    orientation_ = Eigen::Quaternion<Scalar>( qw, qx, qy, qz ).normalized();
  }

  explicit Pose( const Isometry3<Scalar> &isometry )
  {
    translation_ = isometry.translation();
    orientation_ = Eigen::Quaternion<Scalar>( isometry.linear()).normalized();
  }

  explicit Pose( const Eigen::Translation<Scalar, 3> &translation )
  {
    orientation_ = Eigen::Quaternion<Scalar>::Identity();
    translation_ = translation.vector();
  }

  template<typename Derived>
  explicit Pose( const Eigen::RotationBase<Derived, 3> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived()).normalized();
    translation_ = Vector3<Scalar>::Zero();
  }

  template<typename Derived>
  Pose( const Eigen::Translation<Scalar, 3> &translation, const Eigen::RotationBase<Derived, 3> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived()).normalized();
    translation_ = translation.vector();
  }

  template<typename Derived>
  Pose( const Eigen::Translation<Scalar, 3> &translation, const Eigen::MatrixBase<Derived> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived()).normalized();
    translation_ = translation.vector();
  }

  template<typename Derived>
  Pose( const Vector3<Scalar> &translation, const Eigen::RotationBase<Derived, 3> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived()).normalized();
    translation_ = translation;
  }

  template<typename Derived>
  Pose( const Vector3<Scalar> &translation, const Eigen::MatrixBase<Derived> &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation.derived()).normalized();
    translation_ = translation;
  }

  Vector3<Scalar> &translation() { return translation_; }

  const Vector3<Scalar> &translation() const { return translation_; }

  Eigen::Quaternion<Scalar> &orientation() { return orientation_; }

  const Eigen::Quaternion<Scalar> &orientation() const { return orientation_; }

  Eigen::Matrix<Scalar, 3, 3> rotation() const
  {
    return orientation_.toRotationMatrix();
  }

  // ================= Mathematical Operations =================

  void normalize() { orientation_.normalize(); }

  Pose<Scalar> normalized()
  {
    Pose<Scalar> result = *this;
    result.normalize();
    return result;
  }

  Isometry3<Scalar> asTransform() const
  {
    Isometry3<Scalar> result = Isometry3<Scalar>::Identity();
    result.linear() = rotation();
    result.translation() = translation_;
    return result;
  }

  Pose<Scalar> inverse() const
  {
    Pose<Scalar> result;
    // We can use the conjugate as the inverse here since the quaternion should be normalized
    result.orientation_ = orientation_.conjugate();
    result.translation_ = result.orientation_ * -translation_;
    return result;
  }

  void setIdentity()
  {
    translation_ = Vector3<Scalar>::Zero();
    orientation_ = Eigen::Quaternion<Scalar>::Identity();
  }

  Pose<Scalar> &operator*=( const Pose<Scalar> &rh ) { return *this = *this * rh; }

  Pose<Scalar> operator*( const Pose<Scalar> &rh )
  {
    Pose<Scalar> result;
    result.orientation_ = orientation_ * rh.orientation_;
    result.translation_ = translation_ + orientation_ * rh.translation_;
    return result;
  }

  Pose<Scalar> &operator*=( const Eigen::Translation<Scalar, 3> &translation )
  {
    return translate( translation.vector());
  }

  Pose<Scalar> operator*( const Eigen::Translation<Scalar, 3> &translation )
  {
    Pose<Scalar> result = *this;
    result.translate( translation.vector());
    return result;
  }

  Vector3<Scalar> operator*( const Vector3<Scalar> &rh )
  {
    return translation_ + orientation_ * rh;
  }

  template<typename Derived>
  Pose<Scalar> operator*=( const Eigen::RotationBase<Derived, 3> &rotation ) { return rotate( rotation.derived()); }

  template<typename Derived>
  Pose<Scalar> operator*( const Eigen::RotationBase<Derived, 3> &rotation )
  {
    Pose<Scalar> result = *this;
    result.rotate( rotation.derived());
    return result;
  }

  template<int N>
  Eigen::Matrix<Scalar, 3, N> operator*( const Eigen::Matrix<Scalar, 3, N> &rh )
  {
    return (orientation_.toRotationMatrix() * rh).colwise() + translation_;
  }

  template<typename OtherDerived>
  Pose &translate( const Eigen::MatrixBase<OtherDerived> &translation )
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE( OtherDerived, 3 )
    translation_ += orientation_ * translation.derived();
    return *this;
  }

  template<typename OtherDerived>
  Pose &pretranslate( const Eigen::MatrixBase<OtherDerived> &translation )
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE( OtherDerived, 3 )
    translation_ += translation.derived();
    return *this;
  }

  template<typename RotationType>
  Pose &rotate( const RotationType &rotation )
  {
    orientation_ *= rotation;
    return *this;
  }

  template<typename RotationType>
  Pose &prerotate( const RotationType &rotation )
  {
    orientation_ = Eigen::Quaternion<Scalar>( rotation ) * orientation_;
    translation_ = rotation * translation_;
    return *this;
  }

private:
  Eigen::Quaternion<Scalar> orientation_;
  Vector3<Scalar> translation_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename Derived>
Pose<typename Eigen::RotationBase<Derived, 3>::Scalar> operator*( const Eigen::RotationBase<Derived, 3> &rotation,
                                                                  const Pose<typename Eigen::RotationBase<Derived, 3>::Scalar> &pose )
{
  Pose<typename Eigen::RotationBase<Derived, 3>::Scalar> result = pose;
  result.prerotate( rotation.derived());
  return result;
}

template<typename Scalar>
Pose<Scalar> operator*( const Eigen::Translation<Scalar, 3> &translation, const Pose<Scalar> &pose )
{
  Pose<Scalar> result = pose;
  result.pretranslate( translation.vector());
  return result;
}

template<typename Scalar>
Pose<Scalar> operator*( const Isometry3<Scalar> &transform, const Pose<Scalar> &pose )
{
  return Pose<Scalar>( transform ) * pose;
}
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_TYPES_H