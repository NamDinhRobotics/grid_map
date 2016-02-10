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
  if (!getParam("label_layer", labelLayer_)) {
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
    Eigen::Vector3f rgb;
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
  if (!map.exists(labelLayer_)) {
    ROS_WARN_STREAM("SemanticLabelsVisualization::visualize: No grid map layer with name '" << labelLayer_ << "' found.");
    return false;
  }

  //sensor_msgs::PointCloud2Iterator<float> rgb_it(pointCloud, "rgb");
  //sensor_msgs::PointCloud2Iterator<float> label_it(pointCloud, labelLayer_);

  //for (; rgb_it != rgb_it.end(); ++rgb_it, ++label_it)
  //{

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  {
    auto& colorVal = map.at("color", *iterator);
    Eigen::Vector3i rgbVec = colorVectorFromFloat(colorVal);
    auto labelIndex = static_cast<int>(map.at(labelLayer_, *iterator));
    if(labelIndex >= 0 && labelIndex < labelColors_.size())
    {
      multiplyColors(rgbVec, labelColors_.at(labelIndex));
      grid_map::colorVectorToValue(rgbVec, colorVal);
    }
  }

  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map, elevationLayer_, pointCloud);
  publisher_.publish(pointCloud);
  return true;
}

} /* namespace */
