/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 */

#ifndef COTTON_BOLLS_H
#define COTTON_BOLLS_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Geometry>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/io.h>

#include <vector>
#include <iostream>
#include <string>

#include "headers/basic_data.h"

using pcl::PointCloud;
using pcl::PointXYZ;

namespace CottonBolls {
class GetBolls {
    int BollPointDensity;
    float radius;
 public:
    explicit GetBolls(float inRadius);
    PointCloud<PointXYZ>::Ptr removeOutliers(PointCloud<PointXYZ>::Ptr cloud);
    int findBolls(PointCloud<PointXYZ>::Ptr cloud);
};

}  // namespace CottonBolls

#endif  // COTTON_BOLLS_H
