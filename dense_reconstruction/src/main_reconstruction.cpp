/*
 * main_reconstruction.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: Karol Hausman
 */

#include <pcl/console/parse.h>
#include <ros/ros.h>
#include "dense_reconstruction/point_type.h"
#include "dense_reconstruction/DenseReconstruction.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "reconstruction");

	if (argc < 3) {
		PCL_INFO("Usage %s -input_file cloud.pcd \n", argv[0]);

		return -1;
	}

	std::string filename;
	pcl::console::parse_argument(argc, argv, "-input_file", filename);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(
                        new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_for_dense(
			new pcl::PointCloud<pcl::PointXYZLRegionF>);
//	pcl::io::loadPCDFile(filename, *cloud_for_dense);
        pcl::io::loadPCDFile(filename, *cloud_input);

        pcl::copyPointCloud(*cloud_input,*cloud_for_dense);


	DenseReconstruction<pcl::PointXYZLRegionF> dense(cloud_for_dense);

        pcl::PointIndices indices_out;
        dense.activeSegmentation(*cloud_for_dense,0.01f,89,100,indices_out);


        pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_segment(new pcl::PointCloud<pcl::PointXYZLRegionF>);
        for (std::vector<int>::const_iterator pit = indices_out.indices.begin();
                        pit != indices_out.indices.end(); pit++) {
                cloud_segment->points.push_back(cloud_for_dense->points[*pit]);
                cloud_for_dense->points[*pit].reg = 1 + 1;
        }

        cloud_segment->width =
                        static_cast<uint32_t>(cloud_segment->points.size());
        cloud_segment->height = 1;
        cloud_segment->is_dense = true;


        pcl::io::savePCDFile("cloud_for_dense.pcd", *cloud_for_dense);

//	dense.reconstructDenseModel(1);

	return 0;
}

