/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 *
 */


#include <math.h>

#include <string>
#include <vector>
#include <algorithm>

#include "headers/quick_sort.h"
#include "headers/basic_data.h"

basic_data::PCD_data::PCD_data(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) {
    cloud = inCloud;
}

int basic_data::PCD_data::get_data() {
    return 0;
}


/// pretty useless too
PointCloud<PointXYZ>::PointType basic_data::PCD_data::find_highest_value(
    PointCloud<PointXYZ>::Ptr cloud) {

    // will be modified to find the average height of the plants.
    // as of right now it only finds the highest Y value of the cloud.
    pcl::PointCloud<pcl::PointXYZ>::PointType tallestPoint = cloud->points[0];

    for (size_t i = 0; i< cloud->points.size(); ++i) {
        if (tallestPoint.z < cloud->points[i].z) {
            tallestPoint = cloud->points[i];
        }
    }
    return tallestPoint;
}

/**
 * Removes y dimesion from cloud by setting it to zero,
 * then writes it's own file
 *
 * TODO: out dir was harcoded, replaced with const empty string,
 *       change this to more robust solution
 *
 * @returns void
 */
void basic_data::PCD_data::flattenData() {
    const string OUT_DIR = "";
    // header of file obtined from original cloud.
    flatCloud.width = cloud->width;
    flatCloud.height = cloud->height;
    flatCloud.is_dense = cloud->is_dense;
    flatCloud.points.resize(flatCloud.width * flatCloud.height);

    // points being copied over with the exception of y wich is set now to 0.
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        flatCloud.points[i].x = cloud->points[i].x;

        // changes cloud from 3d to 2d by making y = 0.
        flatCloud.points[i].y = 0;
        flatCloud.points[i].z = cloud->points[i].z;
    }

    // write flatCloud to a .pcd file.
    pcl::io::savePCDFileASCII(OUT_DIR, flatCloud);

    std::cout
        << "finished flatteing saved as file flat_sorted_pcd_two.pcd"
        << std::endl;
}

// breaks up the cloud into smaller clouds and returns them in a vector.
// currently breaks them into 5000 smaller clouds.
CloudVector basic_data::PCD_data::segment() {
    size_t sizeOfCloud = cloud->points.size();
    size_t stepsThruCloud = sizeOfCloud / 5000;
    CloudVector segmentedClouds;

    for (size_t i = 0; i < sizeOfCloud - stepsThruCloud; i += stepsThruCloud) {
        PointCloud<PointXYZ>::Ptr segCloud(new PointCloud<PointXYZ>);

        for (size_t k = i; k < (i + stepsThruCloud); ++k)
            segCloud->points.push_back(cloud->points[k]);

        segmentedClouds.push_back(segCloud);
    }

    return segmentedClouds;
}

// finds local maxima for an vector of floats.
std::vector<size_t> basic_data::PCD_data::localMaxima(vector<float> allPoints) {
    vector<size_t> allMaxima_indices;
    size_t currMaxima_index = 0;
    size_t start_index;
    size_t end_index;

    for (size_t k = 750; k < allPoints.size() -750; k+=1500) {
        start_index = k - 750;
        end_index = k + 750;
        for (size_t i = start_index; i < end_index; ++i) {
            if (allPoints[currMaxima_index] < allPoints[i]) {
                currMaxima_index = i;
            }
        }
        allMaxima_indices.push_back(currMaxima_index);
        currMaxima_index = k;
    }

    return allMaxima_indices;
}

// tries to find average height of a row of plants not yet finished.
float basic_data::PCD_data::AvgHeight() {
    /*
    vector<float> topPoints;
    vector<size_t> lMaxima_indices;
    vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > segmentedClouds;

    segmentedClouds = segment();
    pcl::PointCloud<pcl::PointXYZ>::Ptr highestPointsCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::PointType currPoint;

    for (size_t i = 0; i < segmentedClouds.size(); ++i) {
        currPoint = find_highest_value(segmentedClouds[i]);
        currPoint.y = 0;

        highestPointsCloud->points.push_back(currPoint);
    }

    highestPointsCloud->width = highestPointsCloud->points.size();
    highestPointsCloud->height = 1;

    for (size_t i = 0; i < highestPointsCloud->points.size(); ++i) {
        topPoints.push_back(highestPointsCloud->points[i].z);
    }

    lMaxima_indices = localMaxima(topPoints);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_maximas(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < lMaxima_indices.size(); ++i) {
        cloud_maximas->points.push_back(highestPointsCloud->points[lMaxima_indices[i]]);
    }


    pcl::PCDWriter writer;
    cloud_maximas->height = 1;
    cloud_maximas->width = cloud_maximas->points.size();
    writer.write<pcl::PointXYZ>("C:\\Users\\diego_000\\Documents\\PCD_FILES\\test_runs\\HeightCloud.pcd", *highestPointsCloud, false);
    writer.write<pcl::PointXYZ>("C:\\Users\\diego_000\\Documents\\PCD_FILES\\test_runs\\HeightCloud_maximas.pcd", *cloud_maximas, false);


    std::vector<int> vector_of_maximums;
    pcl::LocalMaximum<pcl::PointXYZ> maximum_extractor(true);
    maximum_extractor.setInputCloud(cloud);
    maximum_extractor.setRadius(2);
    maximum_extractor.filter(vector_of_maximums);

    pcl::PointCloud<pcl::PointXYZ> cloud_of_maximums;
    for (size_t i = 0; i < vector_of_maximums.size(); ++i) {
        cloud_of_maximums.points.push_back(cloud->points[vector_of_maximums[i]]);
    }


    cloud_of_maximums.width = cloud_of_maximums.points.size();
    cloud_of_maximums.height = 1;

    pcl::io::savePCDFile<pcl::PointXYZ>("C:\\Users\\diego_000\\Documents\\PCD_FILES\\test_runs\\HeightCloud_maximas.pcd", cloud_of_maximums);*/

    quick_sort::sort(cloud, 0, cloud->points.size()-1, 'x');
    std::vector<float> z_vals;
    std::vector<size_t> lMaxima_indices;
    pcl::PointCloud<pcl::PointXYZ> cloud_of_maximas;

    if (cloud->points.size() > 0) {
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            z_vals.push_back(cloud->points[i].z);
        }

        lMaxima_indices = localMaxima(z_vals);

        for (size_t i = 0; i < lMaxima_indices.size(); ++i)
            cloud_of_maximas.points.push_back(
                cloud->points[lMaxima_indices[i]]);

        /*
        cloud_of_maximas.width = cloud_of_maximas.points.size();
        cloud_of_maximas.height = 1;
        pcl::io::savePCDFile<pcl::PointXYZ>("C:\\Users\\diego_000\\Documents\\PCD_FILES\\test_runs\\HeightCloud_maximas.pcd", cloud_of_maximas);
        */
    } else {
        return 0.0;
    }

    return 0.0;
}


float basic_data::PCD_data::AvgHeight(PointCloud<PointXYZ>::Ptr inCloud) {
    vector<float> Y_vals;
    vector<size_t> lMaxima_indices;
    pcl::PointCloud<pcl::PointXYZ> cloud_of_maximas;
    int average_height = 0;  // the average z-value not actually average plant
                             // height need to subtract z-value of the ground.

    if (inCloud->points.size() > 0) {
        quick_sort::sort(inCloud, 0, inCloud->points.size() - 1, 'x');
        for (size_t i = 0; i < inCloud->points.size(); ++i) {
            Y_vals.push_back(inCloud->points[i].z);
        }

        lMaxima_indices = localMaxima(Y_vals);

        for (size_t i = 0; i < lMaxima_indices.size(); ++i)
            cloud_of_maximas.points.push_back(
                inCloud->points[lMaxima_indices[i]]);


        for (size_t i = 0; i < cloud_of_maximas.points.size(); ++i)
            average_height += cloud_of_maximas[i].z;

        return average_height/cloud_of_maximas.size();
    } else {
        return 0;
    }
    /*
    cloud_of_maximas.width = cloud_of_maximas.points.size();
    cloud_of_maximas.height = 1;
    pcl::io::savePCDFile<pcl::PointXYZ>("C:\\Users\\diego_000\\Documents\\PCD_FILES\\test_runs\\HeightCloud_maximas.pcd", cloud_of_maximas);
    */
}
