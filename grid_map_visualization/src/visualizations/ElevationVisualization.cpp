/*
 * ElevationVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser, Christian Forster
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMap.hpp"
#include "grid_map_visualization/visualizations/ElevationVisualization.hpp"
#include "grid_map_lib/GridMapMath.hpp"

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

} /* namespace grid_map_visualization */
