/*
 * TerrainMapVisualization.cpp
 *
 *  Created on: Dec 18, 2015
 *      Author: Jeff Delmerico
 *   Institute: Uni Zurich, Robotics and Perception Group
 */

#include <grid_map/grid_map.hpp>
#include <grid_map_visualization/visualizations/TerrainMapVisualization.hpp>
#include <grid_map/GridMapRosConverter.hpp>
#include <grid_map/GridMapMsgHelpers.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace grid_map_visualization {

TerrainMapVisualization::TerrainMapVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
    : VisualizationBase(nodeHandle, name)
{
  // this code is to convert RGB to unsigned long values
  unsigned long result;
  grid_map::colorVectorToValue(Eigen::Vector3i(255, 255, 0), result);
  ROS_INFO("your color: %u", result);
}

TerrainMapVisualization::~TerrainMapVisualization()
{
}

bool TerrainMapVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);
  if (!getParam("elevation_layer", elevationLayer_)) {
    ROS_ERROR("TerrainMapVisualization with name '%s' did not find an 'elevation_layer' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("elevation_offset", elevationOffset_)) {
    ROS_ERROR("TerrainMapVisualization with name '%s' did not find an 'elevation_offset' parameter.", name_.c_str());
    return false;
  }

  // this parameter is not required, and by default set to false
  if (!getParam("only_show_elevation", onlyShowElevation_)) {
    onlyShowElevation_ = false;
  }

  if (!getParam("path_layer", pathLayer_)) {
    ROS_ERROR("TerrainMapVisualization with name '%s' did not find a 'label_layer' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("path_offset", pathOffset_)) {
    ROS_ERROR("TerrainMapVisualization with name '%s' did not find an 'path_offset' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("label_names", labelNames_)) {
    ROS_ERROR("TerrainMapVisualization with name '%s' did not find a 'label_names' parameter.", name_.c_str());
    return false;
  }

  std::vector<int> colorInts;
  if (!getParam("label_colors", colorInts)) {
    ROS_ERROR("TerrainMapVisualization with name '%s' did not find a 'label_colors' parameter.", name_.c_str());
    return false;
  }
  if (labelNames_.size() != colorInts.size())
  {
    ROS_ERROR("TerrainMapVisualization with name '%s' received %zu class names and %zu label color values.", name_.c_str(), labelNames_.size(), colorInts.size());
  }
  for(auto c : colorInts)
  {
    Eigen::Vector3f rgb;
    grid_map::colorValueToVector(static_cast<unsigned long>(c), rgb);
    labelColors_.push_back(rgb);
  }

  return true;
}

bool TerrainMapVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(name_, 1, true);
  return true;
}

Eigen::Vector3i TerrainMapVisualization::colorVectorFromFloat(const float color)
{
  unsigned long colorInt = *((unsigned long *)&color);
  Eigen::Vector3i colorVector;
  grid_map::colorValueToVector(colorInt, colorVector);
  return colorVector;
}

void TerrainMapVisualization::multiplyColors(Eigen::Vector3i& rgb, Eigen::Vector3f label)
{
  rgb = (rgb.cast<float>().array() * label.array()).cast<int>().matrix();
}

bool TerrainMapVisualization::visualize(const grid_map::GridMap& mapMsg)
{
  if (!isActive()) return true;

  grid_map::GridMap map = mapMsg;

  if (!map.exists(elevationLayer_)) {
    ROS_WARN_STREAM("TerrainMapVisualization::visualize: No grid map layer with name '" << elevationLayer_ << "' found.");
    return false;
  }
  if (!map.exists(pathLayer_)) {
    ROS_WARN_STREAM("TerrainMapVisualization::visualize: No grid map layer with name '" << pathLayer_ << "' found.");
    return false;
  }

  float path_color;
  grid_map::colorVectorToValue(Eigen::Vector3i(255, 0, 0), path_color);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    // skip points without elevation
    if (!map.isValid(*it, elevationLayer_) && onlyShowElevation_)
      continue;

    // find label with highest probability
    int max_p_index = -1;
    float max_p = 0.0;
    for (int i=0; i<labelNames_.size(); i++)
    {
      auto this_p = map.at(labelNames_[i], *it);
      if(this_p > max_p)
      {
        max_p_index = i;
        max_p = this_p;
      }
    }


    // add elevation offset
    if (!map.isValid(*it, elevationLayer_) && max_p > 1./(double) labelColors_.size() + 0.01)
    {
      map.at(elevationLayer_, *it) = 0.0;

      // add elevation with color
      auto& colorVal = map.at("color", *it);
      Eigen::Vector3i rgbVec = Eigen::Vector3i(255, 255, 255);
      if(max_p_index >= 0 && max_p_index < labelColors_.size())
      {
        multiplyColors(rgbVec, labelColors_.at(max_p_index));
        grid_map::colorVectorToValue(rgbVec, colorVal);
      }
    }
    else
    {
      map.at(elevationLayer_, *it) += elevationOffset_;

      // add elevation with color
      auto& colorVal = map.at("color", *it);
      Eigen::Vector3i rgbVec = colorVectorFromFloat(colorVal);
      if(max_p_index >= 0 && max_p_index < labelColors_.size())
      {
        multiplyColors(rgbVec, labelColors_.at(max_p_index));
        grid_map::colorVectorToValue(rgbVec, colorVal);
      }
    }

    // add path in red above terrain
    if (map.isValid(*it, "path"))
    {
      map.at("color", *it) = path_color;
      if (!map.isValid(*it, elevationLayer_))
        map.at(elevationLayer_, *it) = 0.0;
    }
  }

  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map, elevationLayer_, pointCloud);
  publisher_.publish(pointCloud);
  return true;
}

} /* namespace */
