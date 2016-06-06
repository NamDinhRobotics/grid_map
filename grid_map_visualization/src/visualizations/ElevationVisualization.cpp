/*
 * ElevationVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser, Christian Forster
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMap.hpp"
#include "grid_map_visualization/visualizations/ElevationVisualization.hpp"
#include "grid_map_core/GridMapMath.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map_visualization {

ElevationVisualization::ElevationVisualization(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
  , markerHeight_(1.0)
  , lowerColorValue_(255)
  , upperColorValue_(16711680)
  , lowerColor_(grid_map_visualization::color_utils::colorFromColorValue(lowerColorValue_))
  , upperColor_(grid_map_visualization::color_utils::colorFromColorValue(upperColorValue_))
{
  markerPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("elevation", 1, true);
}

ElevationVisualization::~ElevationVisualization()
{}

bool ElevationVisualization::readParameters()
{
  nodeHandle_.param("elevation/marker_height", markerHeight_, markerHeight_);
  return true;
}

bool ElevationVisualization::initialize()
{
  marker_.ns = "elevation";
  marker_.lifetime = ros::Duration();
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::CUBE_LIST;
  marker_.scale.z = markerHeight_;
  return true;
}

void ElevationVisualization::visualize(
    const grid_map::GridMap& map,
    const std::string& typeNameElevation,
    const std::string& typeNameColor,
    const float lowerValueBound,
    const float upperValueBound)
{
  // Set marker info.
  marker_.header.frame_id = map.getFrameId();
  marker_.header.stamp.fromNSec(map.getTimestamp());
  marker_.scale.x = map.getResolution();
  marker_.scale.y = map.getResolution();

  // Clear points.
  marker_.points.clear();
  marker_.colors.clear();

  float markerHeightOffset = static_cast<float>(markerHeight_/2.0);

  const Eigen::Array2i buffSize = map.getBufferSize();
  const Eigen::Array2i buffStartIndex = map.getBufferStartIndex();
  const bool haveColor = map.isType("color");
  for (unsigned int i = 0; i < buffSize(0); ++i)
  {
    for (unsigned int j = 0; j < buffSize(1); ++j)
    {
      // Getting map data.
      const Eigen::Array2i cellIndex(i, j);
      const Eigen::Array2i buffIndex =
          grid_map_lib::getBufferIndexFromIndex(cellIndex, buffSize, buffStartIndex);
      const float& elevation = map.at(typeNameElevation, buffIndex);
      if(std::isnan(elevation))
        continue;
      const float color = haveColor ? map.at(typeNameColor, buffIndex) : lowerValueBound;

      // Add marker point.
      Eigen::Vector2d position;
      map.getPosition(buffIndex, position);
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = elevation - markerHeightOffset;
      marker_.points.push_back(point);

      // Add marker color.
      if(haveColor)
      {
        std_msgs::ColorRGBA markerColor =
            grid_map_visualization::color_utils::interpolateBetweenColors(
              color, lowerValueBound, upperValueBound, lowerColor_, upperColor_);
        marker_.colors.push_back(markerColor);
      }
    }
  }

  markerPublisher_.publish(marker_);
}

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
