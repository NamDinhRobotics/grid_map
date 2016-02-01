/*
 * TerrainMapVisualization.hpp
 *
 *  Created on: Dec 18, 2015
 *      Author: Jeff Delmerico
 *   Institute: Uni Zurich, Robotics and Perception Group
 */

#pragma once

#include <grid_map_visualization/visualizations/VisualizationBase.hpp>
#include <grid_map_core/GridMap.hpp>

// ROS
#include <ros/ros.h>

namespace grid_map_visualization {

/*!
 * Visualization of the grid map as a point cloud with class labels.
 */
class TerrainMapVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param name the name of the visualization.
   */
  TerrainMapVisualization(ros::NodeHandle& nodeHandle, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~TerrainMapVisualization();

  /*!
   * Read parameters from ROS.
   * @param config the parameters as XML.
   * @return true if successful.
   */
  bool readParameters(XmlRpc::XmlRpcValue& config);

  /*!
   * Initialization.
   */
  bool initialize();

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap& map);

 private:

  Eigen::Vector3i colorVectorFromFloat(const float color);
  void multiplyColors(Eigen::Vector3i& rgb, Eigen::Vector3f label);

  //! Layer that is transformed to points.
  std::string elevationLayer_;
  //! Offset of the elevation
  float elevationOffset_;
  //! Layer that is used for labels.
  std::string pathLayer_;
  //! Offset of the path
  float pathOffset_;
  //! Class names for label
  std::vector<std::string> labelNames_;
  //! RGB triples for label colors
  std::vector<Eigen::Vector3f> labelColors_;
};

} /* namespace */
