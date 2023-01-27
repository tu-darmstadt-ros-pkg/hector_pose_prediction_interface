#ifndef HECTOR_POSE_PREDICTION_ROS_VISUALIZATION_H
#define HECTOR_POSE_PREDICTION_ROS_VISUALIZATION_H

#include <eigen_conversions/eigen_msg.h>
#include <hector_math/types.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace hector_pose_prediction_ros
{
namespace visualization
{
template<typename Scalar>
void drawSupportPolygonImpl( const hector_math::Vector3List<Scalar> &contact_hull_points,
                             const std::vector<std_msgs::ColorRGBA> &colors,
                             const std::string &frame_id,
                             visualization_msgs::MarkerArray &marker_array )
{
  marker_array.markers.reserve( marker_array.markers.size() + contact_hull_points.size() );
  for ( size_t i = 0; i < contact_hull_points.size(); i++ ) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.00001;
    marker.scale.z = 0.00001;

    marker.color = colors[i];
    marker.header.frame_id = frame_id;
    marker.ns = "support_polygon";
    marker.id = static_cast<int32_t>( i );

    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point point_start;
    tf::pointEigenToMsg( contact_hull_points[i], point_start );
    marker.points.push_back( point_start );
    int ip1 = ( i + 1 ) % contact_hull_points.size(); // wrap around
    geometry_msgs::Point point_end;
    tf::pointEigenToMsg( contact_hull_points[ip1], point_end );
    marker.points.push_back( point_end );
    marker_array.markers.push_back( marker );
  }
}

template<typename Scalar>
void drawSupportPolygonWithStability( const hector_math::Vector3List<Scalar> &contact_hull_points,
                                      const hector_math::Vector3List<Scalar> &edge_stabilities,
                                      Scalar max_stability, const std::string &frame_id,
                                      visualization_msgs::MarkerArray &marker_array )
{
  if ( contact_hull_points.size() != edge_stabilities.size() ) {
    return;
  }
  marker_array.markers.reserve( marker_array.markers.size() + contact_hull_points.size() );
  std::vector<std_msgs::ColorRGBA> colors;
  colors.reserve( contact_hull_points.size() );
  for ( const auto &stability : edge_stabilities ) {
    Scalar clamped_stability =
        std::max( 0.0, std::min( stability, max_stability ) ); // Clamp to [0, max_stability]
    float stability_pct = clamped_stability / max_stability;
    std_msgs::ColorRGBA color;
    color.a = 1.0f;
    color.r = 1.0f - stability_pct;
    color.g = stability_pct;
    color.b = 0.0f;
    colors.push_back( color );
  }
  drawSupportPolygonImpl( contact_hull_points, colors, frame_id, marker_array );
}

template<typename Scalar>
void drawSupportPolygon( const hector_math::Vector3List<Scalar> &contact_hull_points,
                         Eigen::Vector4f color, const std::string &frame_id,
                         visualization_msgs::MarkerArray &marker_array )
{
  std_msgs::ColorRGBA color_msg;
  color_msg.r = color( 0 );
  color_msg.g = color( 1 );
  color_msg.b = color( 2 );
  color_msg.a = color( 3 );
  drawSupportPolygonImpl( contact_hull_points,
                          std::vector<std_msgs::ColorRGBA>( contact_hull_points.size(), color_msg ),
                          frame_id, marker_array );
}

template<typename Scalar>
void drawContactPoints( const hector_math::Vector3List<Scalar> &contact_points,
                        Eigen::Vector4f color, const std::string &frame_id, const std::string &ns,
                        visualization_msgs::MarkerArray &marker_array )
{
  marker_array.markers.reserve( marker_array.markers.size() + contact_points.size() );
  for ( size_t i = 0; i < contact_points.size(); i++ ) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker.color.r = color( 0 );
    marker.color.g = color( 1 );
    marker.color.b = color( 2 );
    marker.color.a = color( 3 );

    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = static_cast<int32_t>( i );

    Eigen::Isometry3d marker_pose = Eigen::Isometry3d::Identity();
    marker_pose.translation() = contact_points[i];
    tf::poseEigenToMsg( marker_pose, marker.pose );
    marker_array.markers.push_back( marker );
  }
}

template<typename Scalar>
void drawContactNormals( const hector_math::Vector3List<Scalar> &contact_points,
                         const hector_math::Vector3List<Scalar> &normals, const std::string &frame_id,
                         const std::string &ns, visualization_msgs::MarkerArray &marker_array )
{
  if ( contact_points.size() != normals.size() ) {
    return;
  }
  marker_array.markers.reserve( marker_array.markers.size() + contact_points.size() );
  for ( size_t i = 0; i < contact_points.size(); ++i ) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01; // shaft diameter
    marker.scale.y = 0.02; // head diameter
    marker.scale.z = 0.01; // if not 0, head length

    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = static_cast<int32_t>( i );

    // Identity
    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point point_start_msg;
    tf::pointEigenToMsg( contact_points[i], point_start_msg );
    marker.points.push_back( point_start_msg );

    double normals_scale = 0.1;
    Eigen::Vector3d point_end = contact_points[i] + normals_scale * normals[i];
    geometry_msgs::Point point_end_msg;
    tf::pointEigenToMsg( point_end, point_end_msg );
    marker.points.push_back( point_end_msg );

    // Determine color based on angle between normal and (inverted) gravity vector
    hector_math::Vector3<Scalar> gravity_vector = hector_math::Vector3<Scalar>::UnitZ();
    Scalar angle = std::acos( gravity_vector.dot( normals[i] ) / ( normals[i].norm() ) );
    auto v = static_cast<float>( std::max( -2 / M_PI * angle + 1, 0.0 ) ); // Value between 1 and 0
    marker.color.a = 1.0f;
    marker.color.r = 1.0f - v;
    marker.color.g = v;
    marker.color.b = 0.0f;

    marker_array.markers.push_back( marker );
  }
}

} // namespace visualization
} // namespace hector_pose_prediction_ros

#endif