/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 */

#ifndef BASIC_DATA_H
#define BASIC_DATA_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <Eigen/Geometry>

#include <iostream>
#include <string>
#include <vector>

using pcl::PointCloud;
using pcl::PointXYZ;
using std::string;
using std::vector;

// typedef a aligned PCL::PointCloud vector to CloudVector
typedef vector < PointCloud<PointXYZ>::Ptr,
                    Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr> >

                    CloudVector;

namespace basic_data {

class PCD_data {
    PointCloud<PointXYZ>::Ptr cloud;
    float highestPoint;
    CloudVector centroid_clouds;
    PointCloud<PointXYZ> flatCloud;  // new instance of a cloud to copy data to
                                     // as to not change current cloud.

 public:
    explicit PCD_data(PointCloud<PointXYZ>::Ptr inCloud);
    PCD_data();
    void KmeansSplit(int centroids, int numOfRuns);
    int get_data(PointCloud<PointXYZ>::Ptr cloud);
    int get_data();

    PointCloud<PointXYZ>::PointType find_highest_value(
        PointCloud<PointXYZ>::Ptr cloud);

    void flattenData();
    std::vector<size_t> localMaxima(std::vector<float> allPoints);
    float AvgHeight();
    float AvgHeight(PointCloud<PointXYZ>::Ptr inCloud);
    CloudVector  segment();
};  // class PCD_data
}  // namespace basic_data

#endif  // BASIC_DATA_H
