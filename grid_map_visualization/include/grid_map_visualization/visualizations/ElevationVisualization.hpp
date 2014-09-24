/*
 * ElevationVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
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
class ElevationVisualization
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ElevationVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationVisualization();

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
      const std::string& typeNameElevation,
      const std::string& typeNameColor,
      const float colorScale = 1.0f,
      const float colorOffset = 0.0f);

 private:

  ros::NodeHandle& nodeHandle_;
  ros::Publisher markerPublisher_;
  visualization_msgs::Marker marker_;
  double markerHeight_; //!< Height of the cube markers [m].

};

} /* namespace grid_map_visualization */
