/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 */
#ifndef CLOUDLOCATIONS_H
#define CLOUDLOCATIONS_H

#include <pcl/point_types.h>

#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using pcl::PointCloud;
using pcl::PointXYZ;
using std::string;
using std::vector;

typedef vector < PointCloud<PointXYZ>::Ptr,
                    Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr> >

                    CloudVector;

class cloud_locations {
    std::ifstream infile;
    std::ofstream outfile;

 public:
    explicit cloud_locations(std::string fileDir);
    CloudVector processFiles(PointCloud<PointXYZ>::Ptr cloud);
};

#endif // CLOUDLOCATIONS_H
