/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 *
 *
 */

#include "headers/quick_sort.h"
#include <algorithm>

using pcl::PointCloud;
using pcl::PointXYZ;

namespace quick_sort {
void sort(const PointCloud<PointXYZ>::Ptr cloud, int left, int right, char k) {
    int i = left, j = right;
    float tempX, tempY, tempZ;


    PointCloud<PointXYZ>::PointType pivot = cloud->points[(left + right) / 2];
    if (k == 'x') {
        /* partition */
        while (i <= j) {
            while (cloud->points[i].x < pivot.x)
                i++;

            while (cloud->points[j].x > pivot.x)
                j--;

            if (i <= j) {
                    tempX = cloud->points[i].x;
                    tempY = cloud->points[i].y;
                    tempZ = cloud->points[i].z;

                    cloud->points[i].x = cloud->points[j].x;
                    cloud->points[i].y = cloud->points[j].y;
                    cloud->points[i].z = cloud->points[j].z;
                    cloud->points[j].x = tempX;
                    cloud->points[j].y = tempY;
                    cloud->points[j].z = tempZ;
                    i++;
                    j--;
                }
            }

            /* recursion */
            if (left < j)
                sort(cloud, left, j, 'x');
            if (i < right)
                sort(cloud, i, right, 'x');
        } else if (k == 'y') {
            /* partition */
            while (i <= j) {
                while (cloud->points[i].y < pivot.y)
                    i++;

                while (cloud->points[j].y > pivot.y)
                    j--;
                if (i <= j) {
                    tempX = cloud->points[i].x;
                    tempY = cloud->points[i].y;
                    tempZ = cloud->points[i].z;


                    cloud->points[i].x = cloud->points[j].x;
                    cloud->points[i].y = cloud->points[j].y;
                    cloud->points[i].z = cloud->points[j].z;
                    cloud->points[j].x = tempX;
                    cloud->points[j].y = tempY;
                    cloud->points[j].z = tempZ;
                    i++;
                    j--;
                }
            }

            /* recursion */
            if (left < j)
                sort(cloud, left, j, 'y');
            if (i < right)
                sort(cloud, i, right, 'y');
        } else {
            /* partition */
            while (i <= j) {
                while (cloud->points[i].z < pivot.z)
                    i++;

                while (cloud->points[j].z > pivot.z)
                    j--;
                if (i <= j) {
                    tempX = cloud->points[i].x;
                    tempY = cloud->points[i].y;
                    tempZ = cloud->points[i].z;

                    cloud->points[i].x = cloud->points[j].x;
                    cloud->points[i].y = cloud->points[j].y;
                    cloud->points[i].z = cloud->points[j].z;
                    cloud->points[j].x = tempX;
                    cloud->points[j].y = tempY;
                    cloud->points[j].z = tempZ;
                    i++;
                    j--;
                }
            }

            /* recursion */
            if (left < j)
                sort(cloud, left, j, 'z');
            if (i < right)
                sort(cloud, i, right, 'z');
        }
}
}  // namespace quick_sort
