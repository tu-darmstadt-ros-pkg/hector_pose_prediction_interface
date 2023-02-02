#ifndef HECTOR_POSE_PREDICTION_ROS_VISUALIZATION_H
#define HECTOR_POSE_PREDICTION_ROS_VISUALIZATION_H

#include <eigen_conversions/eigen_msg.h>
#include <hector_math/types.h>
#include <hector_pose_prediction_interface/types.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace hector_pose_prediction_ros
{
namespace visualization
{
template<typename Scalar>
void drawSupportPolygonImpl( visualization_msgs::MarkerArray &marker_array,
                             const hector_math::Vector3List<Scalar> &contact_hull_points,
                             const std::vector<std_msgs::ColorRGBA> &colors,
                             const std::string &frame_id, const std::string &ns )
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
    marker.ns = ns;
    marker.id = static_cast<int32_t>( i );

    marker.pose.orientation.w = 1.0;

    geometry_msgs::Point point_start;
    tf::pointEigenToMsg( contact_hull_points[i].template cast<double>(), point_start );
    marker.points.push_back( point_start );
    int ip1 = ( i + 1 ) % contact_hull_points.size(); // wrap around
    geometry_msgs::Point point_end;
    tf::pointEigenToMsg( contact_hull_points[ip1].template cast<double>(), point_end );
    marker.points.push_back( point_end );
    marker_array.markers.push_back( marker );
  }
}

template<typename Scalar>
void drawSupportPolygon( visualization_msgs::MarkerArray &marker_array,
                         const hector_math::Vector3List<Scalar> &contact_hull_points,
                         Eigen::Vector4f color, const std::string &frame_id, const std::string &ns )
{
  std_msgs::ColorRGBA color_msg;
  color_msg.r = color( 0 );
  color_msg.g = color( 1 );
  color_msg.b = color( 2 );
  color_msg.a = color( 3 );
  drawSupportPolygonImpl( marker_array, contact_hull_points,
                          std::vector<std_msgs::ColorRGBA>( contact_hull_points.size(), color_msg ),
                          frame_id, ns );
}

template<typename Scalar>
void drawSupportPolygonWithStability( visualization_msgs::MarkerArray &marker_array,
                                      const hector_math::Vector3List<Scalar> &contact_hull_points,
                                      const std::vector<Scalar> &edge_stabilities,
                                      Scalar max_stability, const std::string &frame_id,
                                      const std::string &ns )
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
    auto stability_pct = static_cast<float>( clamped_stability / max_stability );
    std_msgs::ColorRGBA color;
    color.a = 1.0f;
    color.r = 1.0f - stability_pct;
    color.g = stability_pct;
    color.b = 0.0f;
    colors.push_back( color );
  }
  drawSupportPolygonImpl( marker_array, contact_hull_points, colors, frame_id, ns );
}

template<typename Scalar>
void drawContactPoints( visualization_msgs::MarkerArray &marker_array,
                        const hector_math::Vector3List<Scalar> &contact_points, Eigen::Vector4f color,
                        const std::string &frame_id, const std::string &ns, double size = 0.04 )
{
  marker_array.markers.reserve( marker_array.markers.size() + contact_points.size() );
  for ( size_t i = 0; i < contact_points.size(); i++ ) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.r = color( 0 );
    marker.color.g = color( 1 );
    marker.color.b = color( 2 );
    marker.color.a = color( 3 );

    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = static_cast<int32_t>( i );

    Eigen::Isometry3d marker_pose = Eigen::Isometry3d::Identity();
    marker_pose.translation() = contact_points[i].template cast<double>();
    tf::poseEigenToMsg( marker_pose, marker.pose );
    marker_array.markers.push_back( marker );
  }
}

template<typename Scalar>
void drawContactNormals( visualization_msgs::MarkerArray &marker_array,
                         const hector_math::Vector3List<Scalar> &contact_points,
                         const hector_math::Vector3List<Scalar> &normals, const std::string &frame_id,
                         const std::string &ns, double normals_scale = 0.05 )
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
    tf::pointEigenToMsg( contact_points[i].template cast<double>(), point_start_msg );
    marker.points.push_back( point_start_msg );

    Eigen::Vector3d point_end = contact_points[i].template cast<double>() +
                                normals_scale * normals[i].template cast<double>();
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

// Convenience functions

template<typename Scalar>
void drawFullSupportPolygon(
    visualization_msgs::MarkerArray &marker_array,
    const hector_pose_prediction_interface::SupportPolygon<Scalar> &support_polygon,
    const std::string &frame_id )
{
  // Draw Polygon with stability if available
  if ( support_polygon.edge_stabilities.empty() ) {
    drawSupportPolygon( marker_array, support_polygon.contact_hull_points,
                        Eigen::Vector4f( 0.0f, 1.0f, 0.0f, 1.0f ), frame_id, "support_polygon" );
  } else {
    drawSupportPolygonWithStability( marker_array, support_polygon.contact_hull_points,
                                     support_polygon.edge_stabilities, 6.0, frame_id,
                                     "support_polygon" );
  }
  // Draw convex hull points
  drawContactPoints( marker_array, support_polygon.contact_hull_points,
                     Eigen::Vector4f( 0, 1, 0, 1 ), frame_id, "support_polygon_points", 0.04 );
}

template<typename Scalar>
void drawFullContactInformation(
    visualization_msgs::MarkerArray &marker_array,
    const hector_pose_prediction_interface::ContactInformation<Scalar> &contact_information,
    const std::string &frame_id )
{
  // Collect data
  hector_math::Vector3List<Scalar> contact_points;
  contact_points.reserve( contact_information.contact_points.size() );
  hector_math::Vector3List<Scalar> normals;
  normals.reserve( contact_information.contact_points.size() );
  for ( const auto &contact_point : contact_information.contact_points ) {
    contact_points.push_back( contact_point.point );
    normals.push_back( contact_point.surface_normal );
  }
  // Draw contact points
  drawContactPoints( marker_array, contact_points, Eigen::Vector4f( 1.0f, 0.5f, 0, 1.0f ), frame_id,
                     "contact_points", 0.03 );
  // Draw contact normals
  drawContactNormals( marker_array, contact_points, normals, frame_id, "surface_normals");
}

template<typename Scalar>
void drawFullSupportPolygonWithContactInformation(
    visualization_msgs::MarkerArray &marker_array,
    const hector_pose_prediction_interface::SupportPolygon<Scalar> &support_polygon,
    const hector_pose_prediction_interface::ContactInformation<Scalar> &contact_information,
    const std::string &frame_id )
{
  // Draw support polygon
  drawFullSupportPolygon( marker_array, support_polygon, frame_id );
  // Draw contact information
  drawFullContactInformation( marker_array, contact_information, frame_id );
}

} // namespace visualization
} // namespace hector_pose_prediction_ros

#endif