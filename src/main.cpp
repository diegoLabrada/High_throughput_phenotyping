/**
 * Copyright (c) 2015 Copyright Holder All Rights Reserved.
 *
 *
 *
 */



#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>

#include <string>
#include <vector>
#include <iostream>

#include "headers/quick_sort.h"
#include "headers/split.h"
#include "headers/basic_data.h"
#include "headers/cotton_bolls.h"
#include "headers/cloud_locations.h"

// function to remove duplicates from the csv file with plot vertices
void csv_duplicate_removal(std::string dir) {
    std::ifstream infile;
    std::ofstream duplicate_file("duplicates_memebers.txt");
    std::ofstream final_file("deleted_duplicate.txt");

    std::vector<std::string> block_of_lines;

    int number_of_lines = 0;
    std::string line;

    infile.open(dir.c_str());
    std::string id;


    std::getline(infile, line);


    for (size_t i = 0; i < line.size(); ++i) {
        if (line[i] != ',') {
            id.push_back(line[i]);
        }
    }
    while (!infile.eof()) {
        std::getline(infile, line);

        std::string new_id;
        for (size_t i = 0; i < line.size(); ++i) {
            if (line[i] != ',') {
                new_id.push_back(line[i]);
            } else {
                break;
            }
        }


        // this is the part that actually removes duplicates.
        if (id.compare(new_id) != 0) {
            if (block_of_lines.size() > 4) {
                for (size_t i = 0; i < block_of_lines.size() - 1; ++i) {
                    for (size_t k = i + 1; k < block_of_lines.size(); ++k) {
                        if (block_of_lines[i].compare(block_of_lines[k]) == 0) {
                            duplicate_file << block_of_lines[k] << std::endl;
                            block_of_lines.erase(block_of_lines.begin() + k);
                        }
                    }
                }

                for (size_t i = 0; i < block_of_lines.size(); ++i) {
                    final_file << block_of_lines[i] << std::endl;
                }
            } else {
                for (size_t i = 0; i < block_of_lines.size(); ++i) {
                    final_file << block_of_lines[i] << std::endl;
                }
            }
            block_of_lines.clear();
            id = new_id;
        }
        // end

        block_of_lines.push_back(line);
    }

    infile.close();
    duplicate_file.close();
    final_file.close();
}

// function to write the final file with boll count and height of the plants.
void write_csv_data_file(
    std::vector<int> bollcounts,
    std::vector<float> heights,
    std::string dir) {

    std::stringstream ss;
    std::vector<std::string> dir_vector;

    dir_vector = split(dir, '.');
    ss << dir_vector[0] << "_processed"<< ".csv";

    std::ifstream infile;
    std::ofstream csv_file(ss.str().c_str());

    infile.open(dir.c_str());
    std::string line;
    std::getline(infile, line);  // remove the first line which is the header.
    size_t index = 0;


    while (!infile.eof()) {
        std::string pre_line;
        std::string throw_line;
        std::vector<std::string> line_vector;
        for (size_t i = 0; i < 4; ++i) {
            if (i == 0) {
                std::getline(infile, pre_line);
            } else {
                std::getline(infile, throw_line);
            }
        }


        std::stringstream ss_two;

        ss_two << pre_line << "," << heights[index] << "," << bollcounts[index];
        ++index;
        csv_file << ss_two.str() << std::endl;
    }
    infile.close();
    csv_file.close();
}

int main(int argc, char *argv[]) {
    // Read in the cloud data
    std::string dirToFile;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > Plot_clouds;
    std::vector<int> boll_count_of_plots;
    std::vector<float> avg_height_of_plots;
    int currCloudBollCount;
    float currCloudHeight;

    std::cout << "enter the file you want to process: " << std::endl;
    getline(cin, dirToFile);


    std::cout << "reading data" << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(dirToFile, *cloud) == -1) {
        PCL_ERROR("COULDNT READ FILE \n");
        return (-1);
    }

    std::cout << "finished reading" << std::endl;

    std::cout << "processing data" << std::endl;
    cloud_locations locationsObj("C:\\Users\\diego_000\\Documents\\PCD_FILES\\GIS_files\\deleted_duplicate.txt");
    Plot_clouds = locationsObj.processFiles(cloud);


    CottonBolls::GetBolls bollFinder(.025);

    basic_data::PCD_data data_processor(Plot_clouds[0]);
    /*
    for (size_t i = 0; i < Plot_clouds.size(); ++i) {

        currCloudHeight = data_processor.AvgHeight(Plot_clouds[i]);
        avg_height_of_plots.push_back(currCloudHeight);

        //bollFinder.removeOutliers(Plot_clouds[i]);
        currCloudBollCount = bollFinder.findBolls(Plot_clouds[i]);
        boll_count_of_plots.push_back(currCloudBollCount);
    }

    write_csv_data_file(boll_count_of_plots, avg_height_of_plots, "C:\\Users\\diego_000\\Documents\\PCD_FILES\\GIS_files\\deleted_duplicate.txt");
    */
    std::cout << "finished" << std::endl;
    getchar();
    return (0);
}
