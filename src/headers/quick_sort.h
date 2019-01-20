/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 *
 */

#ifndef QUICK_SORT_H
#define QUICK_SORT_H

#include <pcl/io/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <string>
#include <algorithm>

using pcl::PointCloud;
using pcl::PointXYZ;

namespace quick_sort {
void sort(PointCloud<PointXYZ>::Ptr cloud, int left, int right, char k);
}

#endif  // QUICK_SORT_H
