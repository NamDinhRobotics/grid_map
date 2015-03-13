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

namespace color_utils {

Eigen::Vector3f colorValueToRGB(
    const unsigned long& colorValue);

std_msgs::ColorRGBA colorFromColorValue(
    const unsigned long& colorValue);

float computeLinearMapping(
    const float& sourceValue,
    const float& sourceLowerValue,
    const float& sourceUpperValue,
    const float& mapLowerValue,
    const float& mapUpperValue);

void setColorChannelFromValue(
    float& colorChannel,
    const float value,
    const float lowerValueBound,
    const float upperValueBound,
    const bool invert,
    const float colorChannelLowerValue,
    const float colorChannelUpperValue);

std_msgs::ColorRGBA interpolateBetweenColors(
    const float value,
    const float lowerValueBound,
    const float upperValueBound,
    const std_msgs::ColorRGBA& colorForLowerValue,
    const std_msgs::ColorRGBA& colorForUpperValue);

} /* namespace color_utils */
} /* namespace grid_map_visualization */
