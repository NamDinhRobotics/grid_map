/*
 * SemanticLabelsVisualization.cpp
 *
 *  Created on: Dec 18, 2015
 *      Author: Jeff Delmerico 
 *   Institute: Uni Zurich, Robotics and Perception Group
 */

#include <grid_map/grid_map.hpp>
#include <grid_map_visualization/visualizations/SemanticLabelsVisualization.hpp>
#include <grid_map/GridMapRosConverter.hpp>
#include <grid_map/GridMapMsgHelpers.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace grid_map_visualization {

SemanticLabelsVisualization::SemanticLabelsVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
    : VisualizationBase(nodeHandle, name)
{
}

SemanticLabelsVisualization::~SemanticLabelsVisualization()
{
}

bool SemanticLabelsVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);
  if (!getParam("elevation_layer", elevationLayer_)) {
    ROS_ERROR("SemanticLabelsVisualization with name '%s' did not find an 'elevation_layer' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("elevation_offset", elevationOffset_)) {
    ROS_ERROR("SemanticLabelsVisualization with name '%s' did not find an 'elevation_offset' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("path_layer", pathLayer_)) {
    ROS_ERROR("SemanticLabelsVisualization with name '%s' did not find a 'label_layer' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("label_names", labelNames_)) {
    ROS_ERROR("SemanticLabelsVisualization with name '%s' did not find a 'label_names' parameter.", name_.c_str());
    return false;
  }

  std::vector<int> colorInts;
  if (!getParam("label_colors", colorInts)) {
    ROS_ERROR("SemanticLabelsVisualization with name '%s' did not find a 'label_colors' parameter.", name_.c_str());
    return false;
  }
  if (labelNames_.size() != colorInts.size())
  {
    ROS_ERROR("SemanticLabelsVisualization with name '%s' received %zu class names and %zu label color values.", name_.c_str(), labelNames_.size(), colorInts.size());
  }
  for(auto c : colorInts)
  {
    Eigen::Vector3i rgb;
    grid_map::colorValueToVector(static_cast<unsigned long>(c), rgb);
    labelColors_.push_back(rgb);
  }

  return true;
}

bool SemanticLabelsVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(name_, 1, true);
  return true;
}

Eigen::Vector3i SemanticLabelsVisualization::colorVectorFromFloat(const float color)
{
  unsigned long colorInt = *((unsigned long *)&color);
  Eigen::Vector3i colorVector;
  grid_map::colorValueToVector(colorInt, colorVector);
  return colorVector;
}

void SemanticLabelsVisualization::multiplyColors(Eigen::Vector3i& rgb, Eigen::Vector3f label)
{
  rgb = (rgb.cast<float>().array() * label.array()).cast<int>().matrix(); 
}

bool SemanticLabelsVisualization::visualize(const grid_map::GridMap& mapMsg)
{
  if (!isActive()) return true;
 
  grid_map::GridMap map = mapMsg;

  if (!map.exists(elevationLayer_)) {
    ROS_WARN_STREAM("SemanticLabelsVisualization::visualize: No grid map layer with name '" << elevationLayer_ << "' found.");
    return false;
  }
  if (!map.exists(pathLayer_)) {
    ROS_WARN_STREAM("SemanticLabelsVisualization::visualize: No grid map layer with name '" << pathLayer_ << "' found.");
    return false;
  }
 

  float path_color;
  grid_map::colorVectorToValue(Eigen::Vector3i(255, 0, 0), path_color);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    // find label with highest probability
    int max_p_index;
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

    // add elevation with color
    float color_value;
    grid_map::colorVectorToValue(labelColors_[max_p_index], color_value);
    map.at("color", *it) = color_value;

    // add elevation offset
    map.at(elevationLayer_, *it) += elevationOffset_;

    // add path in red above terrain
    if (map.at("path", *it) > 0)
    {
      map.at(elevationLayer_, *it) += 0.5;
      map.at("color", *it) = path_color;
    }
  }

  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map, elevationLayer_, pointCloud);
  publisher_.publish(pointCloud);
  return true;
}

} /* namespace */
