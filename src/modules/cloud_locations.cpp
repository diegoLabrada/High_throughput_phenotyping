/*******************************************************************************************
* Copyright (c) 2015 Copyright Holder All Rights Reserved.
* Author: Diego Labrada Sanchez
* Algorithm:
*    1. extract first set of corners from csv file.
*    2. get pointcloud file.
*    3. undo the multiplier and offset from all the points.
*    4. iterate through the points in the pointcloud checking if they are within the
*       boundaries of the corners.
*    5. if they are add them to a new point cloud.
*    6. write the new cloud to a pcd file. naming scheme plot_cloud_id_(id of the csv file corners)
*    7. go to step 1 uneless the csv file has no more members.
*    8. end.
********************************************************************************************/

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <algorithm>
#include <fstream>
#include <vector>

#include "headers/quick_sort.h"
#include "headers/cloud_locations.h"
#include "headers/split.h"

using pcl::PointCloud;
using pcl::PointXYZ;
using std::vector;
using std::stod;

// typedef an aligned pointcloud vector to CloudVector
typedef std::vector<
            PointCloud<PointXYZ>::Ptr,
            Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr> >

            CloudVector;

cloud_locations::cloud_locations(std::string fileDir) {
    infile.open(fileDir);
}

CloudVector cloud_locations::processFiles(PointCloud<PointXYZ>::Ptr cloud) {
    std::vector<double> currCorners;
    std::vector<std::vector<std::string>> sections_of_corners;
    std::string currLine;
    // extracts the corners and puts them into the currCorners vector.

    // Loads file into a 2D array
    while (!infile.eof()) {
        std::vector<std::string> currBlock;

        // only four corners so you add 4 lines into a vector of string.
        for (size_t i = 0; i < 4; ++i) {
            std::getline(infile, currLine);
            currBlock.push_back(currLine);
        }
        sections_of_corners.push_back(currBlock);
    }

    CloudVector all_corners;

    for (size_t i = 0; i < sections_of_corners.size() - 1; ++i) {
        // holds the four points for one plot.
        // corners_points[0] has the first corner point for a plot.

        PointCloud<PointXYZ>::Ptr corners_points(new PointCloud<PointXYZ>);
        for (size_t k = 0; k < 4; ++k) {
            PointXYZ currPoint;
            // FID(0),ID(1),BARCODE(2),RANGE(3),BORDER(4),ENTRY(5),PLOT(6),
            // Treatment(7),subPlot(8),Stand(9),mEast(10),mNorth(11)
            std::vector<std::string> currLine;
            currLine = split(sections_of_corners[i][k], ',');
            currPoint.x = ((std::stod(currLine[9], NULL)  - 409000.0)*1000.0);
            currPoint.y = ((std::stod(currLine[10], NULL) - 3659000.0)*1000.0);
            currPoint.z = 0.0;

            /*
            if (k == 1 && currPoint.y != corners_points->points[0].y) {
                currPoint.y = corners_points->points[0].y;
            }
            if (k == 3 && currPoint.x != corners_points->points[0].x) {
                currPoint.x = corners_points->points[0].x;
            }*/

            corners_points->points.push_back(currPoint);
        }

        quick_sort::sort(corners_points, 0, 3, 'x');
        if (corners_points->points[0].y > corners_points->points[1].y) {
            PointXYZ temp;
            temp.x = corners_points->points[0].x;
            temp.y = corners_points->points[0].y;

            corners_points->points[0].x = corners_points->points[1].x;
            corners_points->points[0].y = corners_points->points[1].y;

            corners_points->points[1].x = temp.x;
            corners_points->points[1].y = temp.y;
        }
        if (corners_points->points[2].y < corners_points->points[3].y) {
            PointXYZ temp;
            temp.x = corners_points->points[2].x;
            temp.y = corners_points->points[2].y;

            corners_points->points[2].x = corners_points->points[3].x;
            corners_points->points[2].y = corners_points->points[3].y;

            corners_points->points[3].x = temp.x;
            corners_points->points[3].y = temp.y;
        }
        // make it a perfect rectangle
        // wouldnt have to do this if the angles lined up for the plots.
        // i.e if the width edge is like a degree slanted you would expect for
        // the length edge to be too but thats not true.
        if (corners_points->points[0].x != corners_points->points[1].x) {
            corners_points->points[0].x = corners_points->points[1].x;
        }
        if (corners_points->points[0].y != corners_points->points[3].y) {
            corners_points->points[0].y = corners_points->points[3].y;
        }
        corners_points->width = corners_points->points.size();
        corners_points->height = 1;
        std::stringstream ssf;
        ssf << "C:\\Users\\diego_000\\Documents\\PCD_FILES\\test_runs\\corners\\plot_corners_" << i << ".pcd";
        pcl::io::savePCDFileASCII(ssf.str(), *corners_points);
        all_corners.push_back(corners_points);  // has all the points for
                                                // each plot all_corners[n]
                                                // should be the nth plot.
    }

    CloudVector  all_plot_clouds;

    /// TODO: optimize with parallelization.
    for (size_t i = 0; i < all_corners.size(); ++i) {
        PointCloud<PointXYZ>::Ptr plot_cloud(new PointCloud<PointXYZ>);

        // only need three corners don't need all four.
        // iterate through the points to see where they lie.
        // assume first point of the points is the origin.

        // ofset/mult for x  / 1000.0) + 409000.0
        // offset/mult for y  / 1000.0) + 3659000.0)

        // vector one
        double vector_one_x = (all_corners[i]->points[1].x)
                                    - (all_corners[i]->points[0].x);
        double vector_one_y = (all_corners[i]->points[1].y)
                                    - (all_corners[i]->points[0].y);

        // vector two
        double vector_two_x = (all_corners[i]->points[3].x)
                                    - (all_corners[i]->points[0].x);
        double vector_two_y = (all_corners[i]->points[3].y )
                                    - (all_corners[i]->points[0].y);

        // dot product of vector_one with itself.
        double width = vector_one_x*vector_one_x
                                    + vector_one_y*vector_one_y;
        double height = vector_two_x*vector_two_x
                                    + vector_two_y*vector_two_y;

        for (size_t j = 0; j < cloud->points.size(); ++j) {
            // undo the offset and multiplier applied to the cloud data.
            double vector_three_x = (cloud->points[j].x)
                                        - (all_corners[i]->points[0].x);
            double vector_three_y = (cloud->points[j].y)
                                        - (all_corners[i]->points[0].y);

            // vector projections.
            double dot_product = (vector_three_x*vector_one_x)
                                        + (vector_three_y*vector_one_y);

            if (dot_product >= 0 && dot_product <= width) {
                double dot_product_two = (vector_three_x*vector_two_x)
                                        + (vector_three_y*vector_two_y);

                if (dot_product_two >= 0 && dot_product_two <= height) {
                    plot_cloud->points.push_back(cloud->points[j]);
                    cloud->points.erase(cloud->points.begin() + j);
                    --j;
                }
            }
        }

        all_plot_clouds.push_back(plot_cloud);

    }



    // write all the plot clouds into their own file for debugging.
    for (size_t i = 0; i < all_plot_clouds.size(); ++i) {
        std::stringstream sstm;
        std::string dir = "C:\\Users\\diego_000\\Documents\\PCD_FILES\\test_runs\\Plot_runs\\plot_cloud_id_";
        std::string fFormat = ".pcd";

        sstm << dir << i << fFormat;
        dir = sstm.str();
        all_plot_clouds[i]->height = 1;
        all_plot_clouds[i]->width = all_plot_clouds[i]->points.size();
        if (!all_plot_clouds[i]->points.empty()) {
            pcl::io::savePCDFileASCII(dir, *all_plot_clouds[i]);
        }
    }
    return all_plot_clouds;
}
