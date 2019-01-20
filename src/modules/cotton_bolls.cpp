/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 *
 */

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include "headers/cotton_bolls.h"
#include "headers/quick_sort.h"

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointIndices;
using std::vector;

CottonBolls::GetBolls::GetBolls(float inRadius) {
    radius = inRadius * 1000;  // holds the radius the cluster. original file
                               // has a 1000 multiplier applied.

    BollPointDensity = 20;     // holds how dense the cluster needs to be
}

PointCloud<PointXYZ>::Ptr CottonBolls::GetBolls::removeOutliers(PointCloud<PointXYZ>::Ptr cloud) {
    // creating new pointer to hold the filtered cloud that only holds the bolls.
    PointCloud<PointXYZ>::Ptr bollsCloud(new PointCloud<PointXYZ>);

    // creating instance of the outlier removal object.
    pcl::RadiusOutlierRemoval<PointXYZ> RadRemove(false);

    // pass the outlier romoval object the cloud that needs to be filtered.
    RadRemove.setInputCloud(cloud);
    // set the radius of search.
    RadRemove.setRadiusSearch(radius);  // 2 inches.

    // set the minimum point density.
    RadRemove.setMinNeighborsInRadius(BollPointDensity);

    // false to return the cloud of only the points that fit the criteria.
    RadRemove.setNegative(false);


    // fileter and save the filtered cloud to bollsCloud.
    RadRemove.filter(*bollsCloud);
    return bollsCloud;  // return filtered cloud.
}


int CottonBolls::GetBolls::findBolls(PointCloud<PointXYZ>::Ptr cloud) {
    using pcl::EuclideanClusterExtraction;
    using pcl::search::KdTree;

    // create instance of the euclidean cluster extraction object.
    EuclideanClusterExtraction<PointXYZ> bollExtraction;

    // create a vector of point indeces to hold the indices
    // of each cluster that was extracted.
    vector<PointIndices> cluster_indices;

    if (cloud->points.size() > 0) {
        // create pointer to the filtered cloud.
        PointCloud<PointXYZ>::Ptr bollsCloud = removeOutliers(cloud);

        // create an instace of the KdTree as the method of search for the
        // euclidean cluster extraction object to use.
        // pass the filtered cloud to the kdTree.
        KdTree<PointXYZ>::Ptr tree(new KdTree<PointXYZ>);
        tree->setInputCloud(bollsCloud);

        // set paramaters and pass the cloud to the extraction object.
        bollExtraction.setClusterTolerance(22); //
        bollExtraction.setMinClusterSize(20);
        bollExtraction.setMaxClusterSize(150);

        bollExtraction.setSearchMethod(tree);
        bollExtraction.setInputCloud(bollsCloud);
        bollExtraction.extract(cluster_indices);

        // returns the size of the cluster indices (the size of this will be
        // the number of bolls in the filtered cloud).
        // std::cout
        //    << cluster_indices.size()
        //    << " cotton bolls were found "
        //    << std::endl;


        // write each of the extracted cluster clouds into a
        // .pcd file to visualize.
        pcl::PCDWriter writer;
        int j = 0;
        for (auto it = cluster_indices.begin();
                    it != cluster_indices.end(); ++it) {
            PointCloud<PointXYZ>::Ptr cloud_cluster(new PointCloud<PointXYZ>);
            for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(bollsCloud->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::stringstream ss;
            ss << "C:\\Users\\diego_000\\Documents\\PCD_FILES\\Trimmed_Lidar_copy\\Trimmed_Lidar\\single plants\\cotton bolls_" << j << ".pcd";
            writer.write<PointXYZ>(ss.str(), *cloud_cluster, false);
            std::cout << "wrote file: " << j << std::endl;
            j++;
        }
        return cluster_indices.size();
    }
    else return 0.0;
    /*
    else {
        // in case the cloud is empty push back zero point indeces.
        PointIndices random;
        random.indices.push_back(0);
        cluster_indices.push_back(random);
    }
    */
}
