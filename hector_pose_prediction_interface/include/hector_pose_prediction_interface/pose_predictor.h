// Copyright (c) 2020 Stefan Fabian, Martin Oehler. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POSE_PREDICTION_INTERFACE_POSE_PREDICTOR_H
#define HECTOR_POSE_PREDICTION_INTERFACE_POSE_PREDICTOR_H

#include "hector_math/robot/robot_model.h"
#include "hector_pose_prediction_interface/types.h"

namespace hector_pose_prediction_interface
{

template<typename Scalar>
struct PosePredictorSettings {
  PosePredictorSettings( int maximum_iterations, Scalar contact_threshold,
                         Scalar tip_over_threshold, bool fix_xy_coordinates = false )
      : maximum_iterations( maximum_iterations ), contact_threshold( contact_threshold ),
        tip_over_threshold( tip_over_threshold ), fix_xy_coordinates( fix_xy_coordinates )
  {
  }

  virtual ~PosePredictorSettings() = default;

  //! The maximum number of iterations the pose predictor will run before giving up.
  int maximum_iterations;
  //! The maximum distance in m for a potential contact point to be considered as in contact.
  Scalar contact_threshold;
  //! The length in z-direction of the projected z-axis below which the robot is considered tipped
  //! over. If the length in z-direction of the projected z-axis is smaller than this value
  Scalar tip_over_threshold;

  //! If true the x-y position is kept fixed rather than being updated to reflect rotation induced translations
  //! of the robots origin. This may lead to slower convergence and even non-converging poses.
  bool fix_xy_coordinates;

  //! Sets the tip over threshold from the given angle in radians between the z-axis and the
  //! projected z-axis of the robot orientation.
  void setTipOverThresholdFromAngle( Scalar angle ) { tip_over_threshold = ::std::cos( angle ); }

  PosePredictorSettings<Scalar> &operator=( const PosePredictorSettings<Scalar> &other )
  {
    maximum_iterations = other.maximum_iterations;
    contact_threshold = other.contact_threshold;
    tip_over_threshold = other.tip_over_threshold;
    fix_xy_coordinates = other.fix_xy_coordinates;
    return *this;
  }
};

template<typename Scalar>
struct Wrench {
  //! External force acting on the COM. Default: Gravity of 9.81 in -Z direction
  hector_math::Vector3<Scalar> force = hector_math::Vector3<Scalar>( 0, 0, -9.81 );
  //! Torque acting on the COM
  hector_math::Vector3<Scalar> torque = hector_math::Vector3<Scalar>( 0, 0, 0 );
};

/*!
 * Interface for a pose predictor implementation which based on an input pose, e.g., a position and identity rotation,
 * estimates the pose of the robot at the given position in the world.
 *
 * @b Note:
 * The members of this class may be called by different threads concurrently and are REQUIRED to be threadsafe.
 *
 * @tparam Scalar The floating point type used for calculations usually float or double.
 */
template<typename Scalar>
class PosePredictor
{
public:
  using Ptr = std::shared_ptr<PosePredictor<Scalar>>;
  using ConstPtr = std::shared_ptr<const PosePredictor<Scalar>>;

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
  Scalar predictPoseAndContactInformation(
      hector_math::Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon,
      ContactInformation<Scalar> &contact_information,
      ContactInformationFlags requested_contact_information = contact_information_flags::All,
      const Wrench<Scalar> &wrench = {} ) const
  {
    return doPredictPoseAndContactInformation( pose, support_polygon, contact_information,
                                               requested_contact_information, wrench );
  }

  /*!
   * Predict the pose of the robot and the support polygon with the passed pose as initial guess.
   * For example, by tipping the robot over unstable axes until a stable pose was found.
   * @param pose Used as both input and output. An isometric transform consisting of a translation and a rotation.
   * @param support_polygon The resulting support polygon if a stable pose was found.
   * @return A value indicating the stability where greater values mean higher stability and negative values or NaN
   *   indicate that there was no stable pose found.
   */
  Scalar predictPoseAndSupportPolygon( hector_math::Pose<Scalar> &pose,
                                       SupportPolygon<Scalar> &support_polygon,
                                       const Wrench<Scalar> &wrench = {} ) const
  {
    return doPredictPoseAndSupportPolygon( pose, support_polygon, wrench );
  }

  /*!
   * Predict the pose of the robot using the passed pose as initial guess.
   * @param pose Used as both input and output. An isometric transform consisting of a translation and a rotation.
   * @return A value indicating the stability where greater values mean higher stability and negative values or NaN
   *   indicate that there was no stable pose found.
   */
  Scalar predictPose( hector_math::Pose<Scalar> &pose, const Wrench<Scalar> &wrench = {} ) const
  {
    return doPredictPose( pose, wrench );
  }

  Scalar tipOverAxis( hector_math::Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon,
                      size_t axis, const Wrench<Scalar> &wrench = {} ) const
  {
    return doTipOverAxis( pose, support_polygon, axis, wrench );
  }

  /*!
   * Estimate the support polygon for a given pose.
   * @param pose The pose of the robot.
   * @param support_polygon The estimated support polygon.
   * @return True if a valid support polygon was estimated, false, otherwise, e.g., because there
   * were only one or two contact points.
   */
  bool estimateSupportPolygon( const hector_math::Pose<Scalar> &pose,
                               SupportPolygon<Scalar> &support_polygon ) const
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
  bool estimateContactInformation(
      const hector_math::Pose<Scalar> &pose, SupportPolygon<Scalar> &support_polygon,
      ContactInformation<Scalar> &contact_information,
      ContactInformationFlags requested_contact_information = contact_information_flags::All ) const
  {
    return doEstimateContactInformation( pose, support_polygon, contact_information,
                                         requested_contact_information );
  }

  virtual typename hector_math::RobotModel<Scalar>::Ptr robotModel() = 0;

  virtual typename hector_math::RobotModel<Scalar>::ConstPtr robotModel() const = 0;

  virtual void updateSettings( const PosePredictorSettings<Scalar> &settings ) = 0;

  virtual const PosePredictorSettings<Scalar> &settings() const = 0;

private:
  virtual Scalar
  doPredictPoseAndContactInformation( hector_math::Pose<Scalar> &pose,
                                      SupportPolygon<Scalar> &support_polygon,
                                      ContactInformation<Scalar> &contact_information,
                                      ContactInformationFlags requested_contact_information,
                                      const Wrench<Scalar> &wrench ) const = 0;

  virtual Scalar doPredictPoseAndSupportPolygon( hector_math::Pose<Scalar> &pose,
                                                 SupportPolygon<Scalar> &support_polygon,
                                                 const Wrench<Scalar> &wrench ) const = 0;

  virtual Scalar doTipOverAxis( hector_math::Pose<Scalar> &pose,
                                SupportPolygon<Scalar> &support_polygon, size_t axis,
                                const Wrench<Scalar> &wrench = {} ) const
  {
    throw std::runtime_error( "Not implemented" );
  }

  virtual Scalar doPredictPose( hector_math::Pose<Scalar> &pose,
                                const Wrench<Scalar> &wrench ) const = 0;

  virtual bool doEstimateSupportPolygon( const hector_math::Pose<Scalar> &pose,
                                         SupportPolygon<Scalar> &support_polygon ) const = 0;

  virtual bool
  doEstimateContactInformation( const hector_math::Pose<Scalar> &pose,
                                SupportPolygon<Scalar> &support_polygon,
                                ContactInformation<Scalar> &contact_information,
                                ContactInformationFlags requested_contact_information ) const = 0;
};
} // namespace hector_pose_prediction_interface

#endif // HECTOR_POSE_PREDICTION_INTERFACE_POSE_PREDICTOR_H
