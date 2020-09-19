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

#include "hector_pose_prediction_interface/contact_information.h"
#include "hector_pose_prediction_interface/robot_model.h"

namespace hector_pose_prediction_interface
{

template<typename Scalar>
struct PosePredictorSettings
{
  PosePredictorSettings( int maximum_iterations, Scalar contact_threshold, Scalar tip_over_threshold )
    : maximum_iterations( maximum_iterations ), contact_threshold( contact_threshold )
      , tip_over_threshold( tip_over_threshold ) { }

  virtual ~PosePredictorSettings() = default;

  //! The maximum number of iterations the pose predictor will run before giving up.
  int maximum_iterations;
  //! The maximum distance in m for a potential contact point to be considered as in contact.
  Scalar contact_threshold;
  //! The length in z-direction of the projected z-axis below which the robot is considered tipped over.
  //! If the length in z-direction of the projected z-axis is smaller than this value
  Scalar tip_over_threshold;

  //! Sets the tip over threshold from the given angle in radians between the z-axis and the projected z-axis of the robot orientation.
  void setTipOverThresholdFromAngle( Scalar angle )
  {
    tip_over_threshold = ::std::cos( angle );
  }

  PosePredictorSettings<Scalar> &operator=( const PosePredictorSettings<Scalar> &other )
  {
    maximum_iterations = other.maximum_iterations;
    contact_threshold = other.contact_threshold;
    tip_over_threshold = other.tip_over_threshold;
    return *this;
  }
};

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

  virtual ~PosePredictor<Scalar>() = default;

  /*!
   * Predict the pose of the robot, the support polygon and contact information (see ContactInformation) with the
   * passed pose as initial guess.
   * For example, by tipping the robot over unstable axes until a stable pose was found.
   * @param pose Used as both input and output. An isometric transform consisting of a translation and a rotation.
   * @param support_polygon The resulting support polygon if a stable pose was found.
   * @param contact_information Contact information for the pose if a stable pose was found.
   *   This includes information about the joint type that is in contact and surface normals if available.
   * @param requested_contact_information Flags indicating the contact information that should be computed.
   *   See RequestedContactInformation and ContactInformation.
   * @return A value indicating the stability where greater values mean higher stability and negative values or NaN
   *   indicate that there was no stable pose found.
   */
  Scalar predictPoseAndContactInformation( Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon,
                                           ContactInformation<Scalar> &contact_information,
                                           ContactInformationFlags requested_contact_information = contact_information_flags::All )
  {
    return doPredictPoseAndContactInformation( pose, support_polygon, contact_information,
                                               requested_contact_information );
  }

  /*!
   * Predict the pose of the robot and the support polygon with the passed pose as initial guess.
   * For example, by tipping the robot over unstable axes until a stable pose was found.
   * @param pose Used as both input and output. An isometric transform consisting of a translation and a rotation.
   * @param support_polygon The resulting support polygon if a stable pose was found.
   * @return A value indicating the stability where greater values mean higher stability and negative values or NaN
   *   indicate that there was no stable pose found.
   */
  Scalar predictPoseAndSupportPolygon( Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon )
  {
    return doPredictPoseAndSupportPolygon( pose, support_polygon );
  }

  /*!
   * Predict the pose of the robot using the passed pose as initial guess.
   * @param pose Used as both input and output. An isometric transform consisting of a translation and a rotation.
   * @return A value indicating the stability where greater values mean higher stability and negative values or NaN
   *   indicate that there was no stable pose found.
   */
  Scalar predictPose( Pose<Scalar> &pose ) { return doPredictPose( pose ); }

  /*!
   * Estimate the support polygon for a given pose.
   * @param pose The pose of the robot.
   * @param support_polygon The estimated support polygon.
   * @return True if a valid support polygon was estimated, false, otherwise, e.g., because there were only one or two
   *   contact points.
   */
  bool estimateSupportPolygon( const Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon )
  {
    return doEstimateSupportPolygon( pose, support_polygon );
  }

  /*!
   * Estimate the support polygon and contact information (see ContactInformation) for a given pose.
   * @param pose The pose of the robot.
   * @param support_polygon The estimated support polygon.
   * @param contact_information Contact information for the pose if a stable pose was found.
   *   This includes information about the joint type that is in contact and surface normals if available.
   * @param requested_contact_information Flags indicating the contact information that should be computed.
   *   See RequestedContactInformation and ContactInformation.
   * @return True if a valid support polygon was estimated, false, otherwise, e.g., because there were only one or two
   *   contact points.
   */
  bool estimateContactInformation( const Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon,
                                   ContactInformation<Scalar> &contact_information,
                                   ContactInformationFlags requested_contact_information = contact_information_flags::All )
  {
    return doEstimateContactInformation( pose, support_polygon, contact_information, requested_contact_information );
  }

  virtual typename RobotModel<Scalar>::Ptr robotModel() = 0;

  virtual typename RobotModel<Scalar>::ConstPtr robotModel() const = 0;

  virtual void updateSettings( const PosePredictorSettings<Scalar> &settings ) = 0;

  virtual const PosePredictorSettings<Scalar> &settings() const = 0;

private:
  virtual Scalar doPredictPoseAndContactInformation( Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon,
                                                     ContactInformation<Scalar> &contact_information,
                                                     ContactInformationFlags requested_contact_information ) = 0;

  virtual Scalar doPredictPoseAndSupportPolygon( Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon ) = 0;

  virtual Scalar doPredictPose( Pose<Scalar> &pose ) = 0;

  virtual bool doEstimateSupportPolygon( const Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon ) = 0;

  virtual bool doEstimateContactInformation( const Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon,
                                             ContactInformation<Scalar> &contact_information,
                                             ContactInformationFlags requested_contact_information ) = 0;
};
}  // namespace hector_pose_prediction_interface

#endif  // HECTOR_POSE_PREDICTION_INTERFACE_POSE_PREDICTOR_H
