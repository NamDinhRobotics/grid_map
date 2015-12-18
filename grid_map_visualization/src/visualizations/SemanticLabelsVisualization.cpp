/*
 * SemanticLabelsVisualization.cpp
 *
 *  Created on: Dec 18, 2015
 *      Author: Jeff Delmerico 
 *   Institute: Uni Zurich, Robotics and Perception Group
 */

#include <grid_map_visualization/visualizations/SemanticLabelsVisualization.hpp>
#include <grid_map/GridMapRosConverter.hpp>

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
  return true;
}

bool SemanticLabelsVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(name_, 1, true);
  return true;
}

bool SemanticLabelsVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;
  if (!map.exists(elevationLayer_)) {
    ROS_WARN_STREAM("SemanticLabelsVisualization::visualize: No grid map layer with name '" << elevationLayer_ << "' found.");
    return false;
  }
  if (!map.exists(labelLayer_)) {
    ROS_WARN_STREAM("SemanticLabelsVisualization::visualize: No grid map layer with name '" << labelLayer_ << "' found.");
    return false;
  }

  sensor_msgs::PointCloud2 pointCloud;
  grid_map::GridMapRosConverter::toPointCloud(map, elevationLayer_, pointCloud);

  sensor_msgs::PointCloud2Iterator<float> rgb_it(pointCloud, "rgb");
  sensor_msgs::PointCloud2Iterator<float> label_it(pointCloud, labelLayer_);

  for (; rgb_it != rgb_it.end(); ++rgb_it, ++label_it)
  {
    if(*rgb_it == *label_it)
      ROS_WARN_STREAM("rgb=label");
  }

  publisher_.publish(pointCloud);
  return true;
}

} /* namespace */
