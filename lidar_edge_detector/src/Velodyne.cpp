/*
 * Velodyne.cpp
 *
 *  Created on: 26.11.2013
 *      Author: ivelas
 */

#include <vector>
#include <cmath>

#include "Velodyne.h"

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <ros/assert.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace ros;



Velodyne::Velodyne::Velodyne(PointCloud<Point> _point_cloud) :
    point_cloud(_point_cloud)
{
  getRings(); // range computation
}

Velodyne::Velodyne Velodyne::Velodyne::transform(float x, float y, float z, float rot_x, float rot_y, float rot_z)
{
  Eigen::Affine3f transf = getTransformation(x, y, z, rot_x, rot_y, rot_z);
  PointCloud<Point> new_cloud;
  transformPointCloud(point_cloud, new_cloud, transf);
  return Velodyne(new_cloud);
}

Velodyne::Velodyne Velodyne::Velodyne::transform(vector<float> DoF)
{
  ROS_ASSERT(DoF.size() == 6);
  return transform(DoF[0], DoF[1], DoF[2], DoF[3], DoF[4], DoF[5]);
}


vector<vector<Velodyne::Point*> > Velodyne::Velodyne::getRings(){
  vector<vector<Point*> > rings(Velodyne::Velodyne::RINGS_COUNT);
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++)
  {
    ROS_ASSERT(pt->ring < RINGS_COUNT);
    pt->range = sqrt(pt->x * pt->x + pt->y * pt->y + pt->z * pt->z);

    rings[pt->ring].push_back(&(*pt));
  }
  return rings;
}

void Velodyne::Velodyne::intensityByRangeDiff(){
  intensityByDiff(Processing::DISTORTIONS);
}
void Velodyne::Velodyne::intensityByIntensityDiff(){
  intensityByDiff(Processing::INTENSITY_EDGES);
}

void Velodyne::Velodyne::intensityByDiff(Processing processing){
  vector<vector<Point*> > rings = this->getRings();

  for (vector<vector<Point*> >::iterator ring = rings.begin(); ring < rings.end(); ring++){
    Point* prev, *succ;
    if (ring->empty()){
      continue;
    }
    float last_intensity = (*ring->begin())->intensity;
    float new_intensity;
    (*ring->begin())->intensity = 0;
    (*(ring->end() - 1))->intensity = 0;
    for (vector<Point*>::iterator pt = ring->begin() + 1; pt < ring->end() - 1; pt++){
      prev = *(pt - 1);
      succ = *(pt + 1);

      switch (processing){
        case Processing::DISTORTIONS:
          (*pt)->intensity = MAX( MAX( prev->range-(*pt)->range, succ->range-(*pt)->range), 0) * 10;
          break;
        case Processing::INTENSITY_EDGES:
          new_intensity = MAX( MAX( last_intensity-(*pt)->intensity, succ->intensity-(*pt)->intensity), 0) * 10;
          last_intensity = (*pt)->intensity;
          (*pt)->intensity = new_intensity;
          break;
        case Processing::NONE:
          break;
        default:
          break;
      }
    }
  }
  normalizeIntensity(0.0, 1.0);
}

PointCloud<PointXYZ> *Velodyne::Velodyne::toPointsXYZ(){
  PointCloud<PointXYZ> *new_cloud = new PointCloud<PointXYZ>();
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
    new_cloud->push_back(PointXYZ(pt->x, pt->y, pt->z));
  }
  return new_cloud;
}

PointCloud<PointXYZI> *Velodyne::Velodyne::toPointsXYZI(){
  PointCloud<PointXYZI> *new_cloud = new PointCloud<PointXYZI>();
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
      PointXYZI point;
      point.x = pt->x;
      point.y = pt->y;
      point.z = pt->z;
      point.intensity = pt->intensity;
      new_cloud->push_back(point);
  }
  return new_cloud;
}

// all intensities to range min-max
void Velodyne::Velodyne::normalizeIntensity(float min, float max){
  float min_found = INFINITY;
  float max_found = -INFINITY;

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
    max_found = MAX(max_found, pt->intensity);
    min_found = MIN(min_found, pt->intensity);
  }

  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
    pt->intensity = (pt->intensity - min_found) / (max_found - min_found) * (max - min) + min;
  }
}

Velodyne::Velodyne Velodyne::Velodyne::threshold(float thresh){
  PointCloud<Point> new_cloud;
  for (PointCloud<Point>::iterator pt = point_cloud.points.begin(); pt < point_cloud.points.end(); pt++){
    if (pt->intensity > thresh){
      new_cloud.push_back(*pt);
    }
  }
  return Velodyne(new_cloud);
}





