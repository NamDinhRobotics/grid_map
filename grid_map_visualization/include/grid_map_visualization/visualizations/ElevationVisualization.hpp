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
    const unsigned long& colorValue)
{
  Eigen::Vector3i rgb;
  rgb(0) = (colorValue >> 16) & 0x0000ff;
  rgb(1) = (colorValue >> 8) & 0x0000ff;
  rgb(2) = (colorValue) & 0x0000ff;
  return rgb.cast<float>()/255.0;
}

std_msgs::ColorRGBA colorFromColorValue(
    const unsigned long& colorValue)
{
  Eigen::Vector3f colorVector = colorValueToRGB(colorValue);
  std_msgs::ColorRGBA color;
  color.r = colorVector(0);
  color.g = colorVector(1);
  color.b = colorVector(2);
  color.a = 1.0;
  return color;
}

float computeLinearMapping(
    const float& sourceValue,
    const float& sourceLowerValue,
    const float& sourceUpperValue,
    const float& mapLowerValue,
    const float& mapUpperValue)
{
  float m = (mapLowerValue - mapUpperValue) / (sourceLowerValue - sourceUpperValue);
  float b = mapUpperValue - m * sourceUpperValue;
  float mapValue = m * sourceValue + b;
  if (mapLowerValue < mapUpperValue)
  {
    mapValue = std::max(mapValue, mapLowerValue);
    mapValue = std::min(mapValue, mapUpperValue);
  }
  else
  {
    mapValue = std::min(mapValue, mapLowerValue);
    mapValue = std::max(mapValue, mapUpperValue);
  }
  return mapValue;
}

void setColorChannelFromValue(
    float& colorChannel,
    const float value,
    const float lowerValueBound,
    const float upperValueBound,
    const bool invert,
    const float colorChannelLowerValue,
    const float colorChannelUpperValue)
{
  float tempColorChannelLowerValue = colorChannelLowerValue;
  float tempColorChannelUpperValue = colorChannelUpperValue;
  if (invert)
  {
    tempColorChannelLowerValue = colorChannelUpperValue;
    tempColorChannelUpperValue = colorChannelLowerValue;
  }
  colorChannel =
      computeLinearMapping(
        value, lowerValueBound, upperValueBound, tempColorChannelLowerValue,
        tempColorChannelUpperValue);
}



std_msgs::ColorRGBA interpolateBetweenColors(
    const float value,
    const float lowerValueBound,
    const float upperValueBound,
    const std_msgs::ColorRGBA& colorForLowerValue,
    const std_msgs::ColorRGBA& colorForUpperValue)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  setColorChannelFromValue(
        color.r, value, lowerValueBound, upperValueBound,
        false, colorForLowerValue.r, colorForUpperValue.r);
  setColorChannelFromValue(
        color.g, value, lowerValueBound, upperValueBound,
        false, colorForLowerValue.g, colorForUpperValue.g);
  setColorChannelFromValue(
        color.b, value, lowerValueBound, upperValueBound,
        false, colorForLowerValue.b, colorForUpperValue.b);
  return color;
}

} /* namespace color_utils */


} /* namespace grid_map_visualization */
