/*
 * SurfaceVisualization.hpp
 *
 *  Created on: June 5th, 2015
 *      Author: Joachim Ott
 *   Institute: University of Zurich, RPG LAB
 */

#pragma once

// ROS
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"

namespace grid_map_visualization {

/*!
 * Visualization of the elevation as pillars.
 */
class SurfaceVisualization
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  SurfaceVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~SurfaceVisualization();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   * @param marker the pointer to the marker that is populated with visualization
   * information.
   */
  bool initialize();

  void visualize(
      const grid_map::GridMap& map,
      const std::string& typeNameSurface,
      const std::string& typeNameColor,
      const float lowerValueBound,
      const float upperValueBound);

 private:

  ros::NodeHandle& nodeHandle_;
  ros::Publisher markerPublisher_;
  visualization_msgs::Marker marker_;
  double markerHeight_; //!< Height of the cube markers [m].

  int lowerColorValue_;
  int upperColorValue_;
  std_msgs::ColorRGBA lowerColor_;
  std_msgs::ColorRGBA upperColor_;

};

} /* namespace grid_map_visualization */
