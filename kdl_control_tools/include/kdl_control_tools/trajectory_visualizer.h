// Copyright (C) 2020  CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef TRAJECTORY_VISUALIZER_H
#define TRAJECTORY_VISUALIZER_H

#include <ros/ros.h>
#include <kdl_control_tools/trajectory.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace visualization_msgs;

namespace kdl_control_tools
{

/**
* Returns the markers to visualize the points of a trajectory, step by step.
* It maintains an internal counter to achieve continuity
**/
class TrajectoryVisualizer
{
  protected:

    CartesianTrajectory traj_;
    int step_;
    // contains all info about a marker
    Marker marker_sample_;

    Marker createMarker_(KDL::Frame point) const;

  public:

    TrajectoryVisualizer(CartesianTrajectory traj, string ns="traj",
                          string frame_id="world");
    ~TrajectoryVisualizer(){};

    // returns the marker for current frame and increments the step
    Marker nextPoint();

    // returns the marker at the given relative time
    Marker pointAt(double t) const;

    // t: time since the beginning of the execution
    void setTime(double t);
    // returns last executed time
    double getTime();

    // will copy the shape, size, color, lifetime properties from the given marker
    void setMarkerProperties(Marker marker);
    // can also set individual property
    void setMarkerType(int val);
    void setMarkerScale(double x, double y, double z);
    void setMarkerScale(double w);
    void setMarkerColor(float r, float g, float b, float a=0.6);
    void setMarkerLifetime(float d);

};

} // end namespace kdl_control_tools

#endif // TRAJECTORY_VISUALIZER_H
