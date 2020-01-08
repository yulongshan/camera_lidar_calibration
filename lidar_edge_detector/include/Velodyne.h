/*
 * Velodyne.h
 *
 *  Created on: 26.11.2013
 *      Author: ivelas
 */

#ifndef VELODYNE_H_
#define VELODYNE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <velodyne_pointcloud/point_types.h>

#include "opencv2/opencv.hpp"
#include <pcl/io/pcd_io.h>
#include <boost/smart_ptr/shared_ptr.hpp>
//#include <pcl/visualization/pcl_visualizer.h>



namespace Velodyne
{

typedef enum
{
  DISTORTIONS, INTENSITY_EDGES, NONE
} Processing;

// Euclidean Velodyne coordinate, including intensity, ring number and range information
struct Point
{
  PCL_ADD_POINT4D
  ; // quad-word XYZ
  float intensity; ///< laser intensity reading
  uint16_t ring; ///< laser ring number
  float range;EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
}EIGEN_ALIGN16;

class Velodyne
{
public:
  Velodyne()
  {
  }
  Velodyne(::pcl::PointCloud<Point> point_cloud);
  Velodyne transform(float x, float y, float z, float rot_x, float rot_y, float rot_z);
  Velodyne transform(std::vector<float> DoF);

  void intensityByDiff(Processing processing);
  void intensityByRangeDiff();
  void intensityByIntensityDiff();

  bool isEmpty(){
    return point_cloud.empty();
  }

  size_t size(){
    return point_cloud.size();
  }

  bool empty(){
    return point_cloud.empty();
  }

  void push_back(Point pt){
    point_cloud.push_back(pt);
  }

  ::pcl::PointCloud<Point>::iterator begin()
  {
    return point_cloud.begin();
  }

  ::pcl::PointCloud<Point>::iterator end()
  {
    return point_cloud.end();
  }

  ::pcl::PointCloud<Point> getPointCloud()
  {
    return point_cloud;
  }

  Velodyne threshold(float thresh);
  void normalizeIntensity(float min = 0.0, float max = 1.0);
  ::pcl::PointCloud<pcl::PointXYZ> *toPointsXYZ();

  ::pcl::PointCloud<pcl::PointXYZI> *toPointsXYZI();

    static const unsigned RINGS_COUNT = 64;
  std::vector<std::vector<Point*> > getRings();

protected:
  ::pcl::PointCloud<Point> point_cloud;
};

} /* NAMESPACE Velodyne */



POINT_CLOUD_REGISTER_POINT_STRUCT(
    Velodyne::Point, (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (uint16_t, ring, ring))

#endif /* VELODYNE_H_ */
