/*
 * DenseReconstruction.h
 *
 *  Created on: Aug 17, 2012
 *      Author: Karol Hausman
 */

#ifndef DENSERECONSTRUCTION_H_
#define DENSERECONSTRUCTION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/boundary.h>

#include "dense_reconstruction/point_type.h"

template<typename PointType>
class DenseReconstruction {
public:
	typedef pcl::PointCloud<PointType> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	DenseReconstruction(typename pcl::PointCloud<PointType>::Ptr saved_cloud) {
		debug_ = 0;
                plane_segmentation_ = 1;
                plane_segmentation_dist_thresh_ = 0.02;//0.006;
		plane_segmentation_max_iter_ = 1000;
		normals_radius_search_ = 0.03;
		boundary_radius_ = 0.02;
		euclidean_tolerance_ = 0.01;
                min_cluster_size_ = 0;//10;
		max_cluster_size_ = 50000;

		cloud_.reset(new pcl::PointCloud<PointType>(*saved_cloud));


                cloud_operational_.reset(new pcl::PointCloud<PointType>);
                pcl::copyPointCloud(*cloud_, *cloud_operational_);

		if (debug_)
			pcl::io::savePCDFile("Aafter_plane.pcd", *cloud_operational_);

		if (debug_)
			pcl::io::savePCDFile("Aafter_normals.pcd", *cloud_operational_);

	}
	;
	void planeSegmentation(const PointCloudPtr &cloud,
			pcl::ModelCoefficients &coefficients, pcl::PointIndices &inliers);
	void planeExtraction(const PointCloudPtr &cloud_input,
			pcl::PointIndices::Ptr &inliers, PointCloud &cloud_output);
	void normalsEstimation(const PointCloudPtr &cloud,
			pcl::PointCloud<pcl::Normal>::Ptr &normals);
	void boundaryEstimation(const PointCloudPtr &cloud_input,
			const pcl::PointCloud<pcl::Normal>::Ptr &normals,
			pcl::PointCloud<pcl::Boundary> &boundaries);
	void activeSegmentation(const PointCloud &cloud_input, float search_radius,
			double eps_angle, int fp_index, pcl::PointIndices &indices_out);
	void reconstructDenseModel(const int feature_number, float search_radius =
			0.01f, double eps_angle = 89);

protected:
	PointCloudPtr cloud_operational_;
	PointCloudPtr cloud_;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
	bool debug_;
	bool plane_segmentation_;
	int plane_segmentation_max_iter_;
	double plane_segmentation_dist_thresh_;
	double normals_radius_search_;
	double boundary_radius_;
	double euclidean_tolerance_;
	int min_cluster_size_;
	int max_cluster_size_;
        int magic_index_;

};
#include "../../src/DenseReconstruction.hpp"

#endif /* DENSERECONSTRUCTION_H_ */
