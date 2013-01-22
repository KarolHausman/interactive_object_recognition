#include <template_library/template_library.h>

/*
 * template_library.cpp
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */


TemplateLibrary::TemplateLibrary():
    nh_ ("~/template_library"),
    dense_reconstructor_()
{
    filenames_.push_back("/home/karol/Desktop/template2.pcd");


}

void TemplateLibrary::addLocation(const std::string &location)
{
    filenames_.push_back(location);
}

void TemplateLibrary::loadClouds()
{

    for(uint i=0;i<filenames_.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(
                        new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_for_dense(
                        new pcl::PointCloud<pcl::PointXYZLRegionF>);

        pcl::io::loadPCDFile(filenames_[i], *cloud_input);

        pcl::copyPointCloud(*cloud_input,*cloud_for_dense);

        clouds_dense_.push_back(cloud_for_dense);
        clouds_input_.push_back(cloud_input);
    }
}

void TemplateLibrary::generateTemplateData()
{
    loadClouds();

    for(uint i=0;i<clouds_dense_.size();i++)
    {
        pcl::PointIndices template_inliers;
        generateTemplateDataEuclidean(clouds_dense_[i],template_inliers);
        cv::Mat image=restoreCVMatNoPlaneFromPointCloud(clouds_input_[i],template_inliers);
        cv::imshow("no plane image",image);
        cv::waitKey();


    }
}

cv::Mat TemplateLibrary::restoreCVMatNoPlaneFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr,pcl::PointIndices &inliers)
{
    cv::Mat restored_image = cv::Mat (cloud_in_ptr->height, cloud_in_ptr->width, CV_8UC3);

    for (uint i = 0; i < inliers.indices.size(); i++)
    {
        cloud_in_ptr->points[inliers.indices[i]].b = 0;
        cloud_in_ptr->points[inliers.indices[i]].g = 0;
        cloud_in_ptr->points[inliers.indices[i]].r = 0;
    }

    for (uint rows = 0; rows < cloud_in_ptr->height; rows++)
    {
        for (uint cols = 0; cols < cloud_in_ptr->width; ++cols)
        {

            {
                restored_image.at<cv::Vec3b> (rows, cols)[0] = cloud_in_ptr->at (cols, rows).b;
                restored_image.at<cv::Vec3b> (rows, cols)[1] = cloud_in_ptr->at (cols, rows).g;
                restored_image.at<cv::Vec3b> (rows, cols)[2] = cloud_in_ptr->at (cols, rows).r;
            }
        }
    }
    return restored_image;
}

void TemplateLibrary::generatePlaneOutliers(const pcl::PointIndices& inliers,const uint &cloud_size, pcl::PointIndices &outliers)
{
    std::vector<int> indices=inliers.indices;
    std::sort(indices.begin(),indices.end());
    for(uint i=0; i<cloud_size;i++)
    {
        if (static_cast<int>(i)!=indices[i])
            outliers.indices.push_back(i);
    }

}

void TemplateLibrary::generateTemplateDataEuclidean(pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input, pcl::PointIndices &template_inliers)
{

        dense_reconstructor_.reset(new DenseReconstruction<pcl::PointXYZLRegionF>(cloud_input));

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

        pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_extracted(
                        new pcl::PointCloud<pcl::PointXYZLRegionF>);

        dense_reconstructor_->planeSegmentation(cloud_input,coefficients,*inliers);
        dense_reconstructor_->planeExtraction(cloud_input,inliers,*cloud_extracted);

        generatePlaneOutliers(*inliers,cloud_input->size(),*outliers);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(
                    new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr euclidian_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::copyPointCloud(*cloud_input,*euclidian_cloud);
        ec.setClusterTolerance(0.02);

        ec.setMinClusterSize(1000);

        ec.setSearchMethod(kd_tree);
        ec.setInputCloud(euclidian_cloud);
        ec.setIndices(outliers);
        ec.extract(cluster_indices);

        pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr temp_cloud(
                    new pcl::PointCloud<pcl::PointXYZLRegionF>);
        ROS_INFO_STREAM("size od the clusters: "<<cluster_indices.size());

        if(cluster_indices.size()>0)
        {
            pcl::copyPointCloud(*euclidian_cloud,*temp_cloud);
            for (uint i=0;i<cluster_indices[0].indices.size();i++)
            {
                temp_cloud->points[cluster_indices[0].indices[i]].f=2;
            }

            //        pcl::copyPointCloud(*euclidian_cloud,cluster_indices[0],*temp_cloud);

        }
        pcl::io::savePCDFile("cloud_temp.pcd", *temp_cloud);
        template_inliers=cluster_indices[0];
    }


int TemplateLibrary::getIndex(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr& cloud)
{

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_extracted(
                    new pcl::PointCloud<pcl::PointXYZLRegionF>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(
                    new pcl::PointCloud<pcl::PointXYZRGB>);
    dense_reconstructor_->planeSegmentation(cloud,coefficients,*inliers);
    dense_reconstructor_->planeExtraction(cloud,inliers,*cloud_extracted);


    pcl::copyPointCloud(*cloud,*cloud_input);

    Eigen::Vector4f center;
    pcl::compute3DCentroid(*cloud,*inliers,center);


    pcl::PointXYZRGB searchPointTemp;
    searchPointTemp.x = center(0);
    searchPointTemp.y = center(1);
    searchPointTemp.z = center(2);


    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    boost::shared_ptr <std::vector<int> > kd_tree_indices(new std::vector<int>);
    *kd_tree_indices=inliers->indices;
    kdtree.setInputCloud(cloud_input,kd_tree_indices);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.nearestKSearch(searchPointTemp, 1, pointIdxRadiusSearch,
                    pointRadiusSquaredDistance) > 0)
    {
        pointIdxRadiusSearch[0];
    }

    cloud->points[pointIdxRadiusSearch[0]].f=1;
    pcl::io::savePCDFile("cloud_with_f.pcd", *cloud);

    return pointIdxRadiusSearch[0];


}


void TemplateLibrary::generateTemplateDataActive(pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input,pcl::PointIndices &template_inliers)
{

        dense_reconstructor_.reset(new DenseReconstruction<pcl::PointXYZLRegionF>(cloud_input));

        pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_segment(new pcl::PointCloud<pcl::PointXYZLRegionF>);
        pcl::PointIndices indices_reconstruct;
        int index=getIndex(cloud_input);
        dense_reconstructor_->activeSegmentation(*cloud_input,0.01f,89,index,indices_reconstruct);

        for (std::vector<int>::const_iterator pit = indices_reconstruct.indices.begin();
                        pit != indices_reconstruct.indices.end(); pit++)
        {
                cloud_segment->points.push_back(cloud_input->points[*pit]);
                cloud_input->points[*pit].reg = 1 + 1;
        }

        cloud_segment->width =
                        static_cast<uint32_t>(cloud_segment->points.size());
        cloud_segment->height = 1;
        cloud_segment->is_dense = true;


        pcl::io::savePCDFile("cloud_for_dense.pcd", *cloud_input);
        template_inliers=indices_reconstruct;

 }






