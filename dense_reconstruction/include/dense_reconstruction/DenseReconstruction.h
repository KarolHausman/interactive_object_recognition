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
		plane_segmentation_dist_thresh_ = 0.006;
		plane_segmentation_max_iter_ = 1000;
		normals_radius_search_ = 0.03;
		boundary_radius_ = 0.02;
		euclidean_tolerance_ = 0.01;
                min_cluster_size_ = 0;//10;
		max_cluster_size_ = 50000;

		cloud_.reset(new pcl::PointCloud<PointType>(*saved_cloud));

		if (plane_segmentation_) {
                    cloud_operational_.reset(new pcl::PointCloud<PointType>);
                    pcl::copyPointCloud(*cloud_, *cloud_operational_);

//			pcl::ModelCoefficients coefficients;
//			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//			if (debug_)
//				pcl::io::savePCDFile("Aafter_beg.pcd", *cloud_);
//			planeSegmentation(cloud_, coefficients, *inliers);
//			planeExtraction(cloud_, inliers, *cloud_operational_);

//                        pcl::io::savePCDFile("after_plane.pcd", *cloud_operational_);

//                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(
//                                        new pcl::PointCloud<pcl::PointXYZRGB>);

////                        pcl::copyPointCloud(*cloud_operational_,*cloud_input);
//                        pcl::copyPointCloud(*cloud_,*cloud_input);

//                        Eigen::Vector4f center;
////                        pcl::compute3DCentroid(*cloud_input,center);
//                        pcl::compute3DCentroid(*cloud_,*inliers,center);


//                        pcl::PointXYZRGB searchPointTemp;
//                        searchPointTemp.x = center(0);
//                        searchPointTemp.y = center(1);
//                        searchPointTemp.z = center(2);

//                        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//                        boost::shared_ptr <std::vector<int> > kd_tree_indices(new std::vector<int>);
//                        *kd_tree_indices=inliers->indices;
//                        kdtree.setInputCloud(cloud_input,kd_tree_indices);

//                        std::vector<int> pointIdxRadiusSearch;
//                        std::vector<float> pointRadiusSquaredDistance;

//                        if (kdtree.nearestKSearch(searchPointTemp, 1, pointIdxRadiusSearch,
//                                        pointRadiusSquaredDistance) > 0)
//                        {
//                            pointIdxRadiusSearch[0];
//                        }

//                        cloud_->points[ /*inliers->indices[*/ pointIdxRadiusSearch[0] /*]*/ ].f=1;
//                        pcl::io::savePCDFile("cloud_with_f.pcd", *cloud_);

//                        magic_index_=/*inliers->indices[*/ pointIdxRadiusSearch[0] /*]*/;


		} else
			pcl::copyPointCloud(*cloud_, *cloud_operational_);

		if (debug_)
			pcl::io::savePCDFile("Aafter_plane.pcd", *cloud_operational_);
//		cloud_normals_.reset(new pcl::PointCloud<pcl::Normal>);
//		normalsEstimation(cloud_operational_, cloud_normals_);

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
