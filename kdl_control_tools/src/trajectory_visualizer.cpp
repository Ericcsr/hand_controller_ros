// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include <kdl_control_tools/trajectory_visualizer.h>
#include <kdl_conversions/kdl_msg.h>

using namespace kdl_control_tools;


TrajectoryVisualizer::TrajectoryVisualizer(CartesianTrajectory traj, string ns, string frame_id):
  traj_(traj), step_(0)
{
  // create the marker prototype
  marker_sample_.header.frame_id = frame_id;
  marker_sample_.ns = ns;

  marker_sample_.scale.x = 0.01;
  marker_sample_.scale.y = 0.01;
  marker_sample_.scale.z = 0.01;
  marker_sample_.color.r = 0.4;
  marker_sample_.color.g = 0.4;
  marker_sample_.color.b = 0.8;
  marker_sample_.color.a = 0.6;

  marker_sample_.type = visualization_msgs::Marker::CUBE;
  marker_sample_.action = visualization_msgs::Marker::MODIFY;
  marker_sample_.frame_locked = true;

  // short duration
  marker_sample_.lifetime = ros::Duration(0.15);

}
/*********************************************************************
* returns the marker for current frame and increments the step
*********************************************************************/
Marker TrajectoryVisualizer::nextPoint()
{
  KDL::Frame point = traj_.getPoint(step_);
  step_++;
  return createMarker_(point);
}
/*********************************************************************
* returns the marker at the given relative time
*********************************************************************/
Marker TrajectoryVisualizer::pointAt(double t) const
{
  int step_cur = traj_.getIndex(t);
  KDL::Frame point = traj_.getPoint(step_cur);
  return createMarker_(point);
}
/*********************************************************************
* sets the trajectory step corresponding to the given time
* t: time since the beginning of the execution
*********************************************************************/
void TrajectoryVisualizer::setTime(double t)
{
  step_ = traj_.getIndex(t);
}
/*********************************************************************
* returns last executed time
*********************************************************************/
double TrajectoryVisualizer::getTime()
{
  return traj_.getTime(step_);
}
/*********************************************************************
* returns the marker at the given relative time
*********************************************************************/
Marker TrajectoryVisualizer::createMarker_(KDL::Frame point) const
{
  // copy the prototype properties
  visualization_msgs::Marker marker = marker_sample_;
  marker.header.stamp = ros::Time();
  marker.id = step_;

  // position from arg
  tf::poseKDLToMsg(point, marker.pose);

  return marker;
}

/*********************************************************************
* GETTERS & SETTERS
*********************************************************************/
// will copy the shape, size, color, lifetime properties from the given marker
void TrajectoryVisualizer::setMarkerProperties(Marker marker)
{
  // update the marker prototype
  marker_sample_.scale = marker.scale;
  marker_sample_.color = marker.color;
  marker_sample_.type = marker.type;
  marker_sample_.lifetime = marker.lifetime;
}
void TrajectoryVisualizer::setMarkerType(int val){
  marker_sample_.type = val;
}
void TrajectoryVisualizer::setMarkerScale(double x, double y, double z){
  marker_sample_.scale.x = x;
  marker_sample_.scale.y = y;
  marker_sample_.scale.z = z;
}
void TrajectoryVisualizer::setMarkerScale(double w){
  setMarkerScale(w, w, w);
}
void TrajectoryVisualizer::setMarkerColor(float r, float g, float b, float a){
  marker_sample_.color.r = r;
  marker_sample_.color.g = g;
  marker_sample_.color.b = b;
  marker_sample_.color.a = a;
}
void TrajectoryVisualizer::setMarkerLifetime(float d){
  marker_sample_.lifetime = ros::Duration(d);
}
