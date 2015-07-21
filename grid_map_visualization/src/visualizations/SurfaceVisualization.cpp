/*
 * SurfaceVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser, Christian Forster
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map/GridMap.hpp"
#include "grid_map_visualization/visualizations/SurfaceVisualization.hpp"
#include "grid_map_lib/GridMapMath.hpp"
#include "grid_map_visualization/visualizations/ElevationVisualization.hpp"

// Eigen
#include <Eigen/Core>

namespace grid_map_visualization {

SurfaceVisualization::SurfaceVisualization(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
  , markerHeight_(1.0)
  , lowerColorValue_(255)
  , upperColorValue_(16711680)
  , lowerColor_(grid_map_visualization::color_utils::colorFromColorValue(lowerColorValue_))
  , upperColor_(grid_map_visualization::color_utils::colorFromColorValue(upperColorValue_))
{
  markerPublisher_ =
      nodeHandle_.advertise<visualization_msgs::Marker>("surface", 1, true);
}

SurfaceVisualization::~SurfaceVisualization()
{}

bool SurfaceVisualization::readParameters()
{
  nodeHandle_.param("surface/marker_height", markerHeight_, markerHeight_);
  return true;
}

bool SurfaceVisualization::initialize()
{
  marker_.ns = "surface";
  marker_.lifetime = ros::Duration();
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::CUBE_LIST;
  marker_.scale.z = markerHeight_;
  return true;
}

void SurfaceVisualization::visualize(
    const grid_map::GridMap& map,
    const std::string& typeNameSurface,
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
  const bool haveColor = map.isType("class");
  for (unsigned int i = 0; i < buffSize(0); ++i)
  {
    for (unsigned int j = 0; j < buffSize(1); ++j)
    {
      // Getting map data.
      const Eigen::Array2i cellIndex(i, j);
      const Eigen::Array2i buffIndex =
          grid_map_lib::getBufferIndexFromIndex(cellIndex, buffSize, buffStartIndex);
      const float& elevation = map.at("mu", buffIndex);
      const float& class_pred = map.at("class", buffIndex);
      int class_number=0;

      if(!std::isnan(class_pred))
      {
        class_number=(int)class_pred;
      }
      if(std::isnan(elevation))
        {

          continue;
        }


      //const float color = haveColor ? 16711680/class_pred : lowerValueBound;
      //ROS_INFO_STREAM("class_pred: "<<class_pred <<" int: "<<(int)class_pred);

      std_msgs::ColorRGBA surface_color;
      surface_color.a = 1.0;
      switch (class_number) {
        case 1:
          //long_grass
          surface_color.r = 0.0;
          surface_color.g = 1.0;
          surface_color.b = 0.0;
          break;
        case 3:
          //cobblestone
          surface_color.r = 0.62;
          surface_color.g = 0.32;
          surface_color.b = 0.17;
          break;
        case 4:
          //gravel
          surface_color.r = 0.2;
          surface_color.g = 0.2;
          surface_color.b = 0.2;
          break;
        default:
          surface_color.r = 1.0;
          surface_color.g = 1.0;
          surface_color.b = 1.0;          
          break;
        }





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
//        std_msgs::ColorRGBA markerColor =
//            grid_map_visualization::color_utils::interpolateBetweenColors(
//              color, lowerValueBound, upperValueBound, lowerColor_, upperColor_);
        //marker_.colors.push_back(markerColor);
        marker_.colors.push_back(surface_color);
      }
    }
  }

  markerPublisher_.publish(marker_);
}


} /* namespace grid_map_visualization */
