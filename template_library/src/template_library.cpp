#include <template_library/template_library.h>

/*
 * template_library.cpp
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */

#include <pcl/filters/filter.h>

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

    for(uint i=0;i<clouds_input_.size();i++)
    {
        pcl::PointIndices template_inliers;
        generateTemplateDataEuclidean(clouds_input_[i],template_inliers);
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
        //        int rows=static_cast<int>(inliers.indices[i]/cloud_in_ptr->width);
        //        int cols=static_cast<int>(inliers.indices[i]%cloud_in_ptr->width);
        cloud_in_ptr->points[inliers.indices[i]].b = 0;
        cloud_in_ptr->points[inliers.indices[i]].g = 0;
        cloud_in_ptr->points[inliers.indices[i]].r = 0;

    }

    int border_no_nan_width=180;
    int border_no_nan_height=180;

    for (uint rows = 0; rows < cloud_in_ptr->height; rows++)
    {
        for (uint cols = 0; cols < cloud_in_ptr->width; ++cols)
        {                
            if((std::isnan(cloud_in_ptr->at(cols, rows).x))
                    && ((static_cast<int>(cols)<border_no_nan_width)||(static_cast<int>(cols)>cloud_in_ptr->width-border_no_nan_width)
                        || (static_cast<int>(rows)<border_no_nan_height)||(static_cast<int>(rows)>cloud_in_ptr->height-border_no_nan_height)) )
            {
                cloud_in_ptr->at (cols, rows).b = 0;
                cloud_in_ptr->at (cols, rows).g = 0;
                cloud_in_ptr->at (cols, rows).r = 0;
            }



            restored_image.at<cv::Vec3b> (rows, cols)[0] =cloud_in_ptr->at (cols, rows).b;
            restored_image.at<cv::Vec3b> (rows, cols)[1] =cloud_in_ptr->at (cols, rows).g;
            restored_image.at<cv::Vec3b> (rows, cols)[2] =cloud_in_ptr->at (cols, rows).r;

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

        if(std::find(indices.begin(), indices.end(), i) != indices.end()) {
            /* indices contains i */
        } else {
            /* indices does not contain i */
            outliers.indices.push_back(i);

        }

    }

}


void TemplateLibrary::planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::ModelCoefficients &coefficients,
                                        pcl::PointIndices &inliers) {



    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);
}


void TemplateLibrary::generateTemplateDataEuclidean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input, pcl::PointIndices &template_inliers)
{
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr plane_outliers(new pcl::PointIndices);

    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    //        pcl::removeNaNFromPointCloud(*cloud_input,*cloud_temp,*indices);

    planeSegmentation(cloud_input,coefficients,*plane_inliers);
    generatePlaneOutliers(*plane_inliers,cloud_input->size(),*plane_outliers);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(
                new pcl::search::KdTree<pcl::PointXYZRGB>);

    ec.setClusterTolerance(0.02);

    ec.setMinClusterSize(100);
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(cloud_input);
    pcl::PointIndices::Ptr outliers2(new pcl::PointIndices);

    //        for (uint z=0;z<inliers->indices.size();z++)
    //            if((inliers->indices[z]<=euclidian_cloud->points.size())&&(inliers->indices[z]>=0))
    //                outliers2->indices.push_back(inliers->indices[z]);

    //        sort( outliers2->indices.begin(), outliers2->indices.end() );
    //        outliers2->indices.erase( unique( outliers2->indices.begin(), outliers2->indices.end() ), outliers2->indices.end() );
    //            if((inliers->indices[z]>=euclidian_cloud->points.size())||(inliers->indices[z]<0))
    //                ROS_INFO("HERE I AM");

    //        for (uint j=0;j<(euclidian_cloud->points.size());j++)
    //            outliers2->indices.push_back(j);
    ec.setIndices(plane_inliers);
    //        ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr temp_cloud(
                new pcl::PointCloud<pcl::PointXYZLRegionF>);
    ROS_INFO_STREAM("size od the clusters: "<<cluster_indices.size());

    if(cluster_indices.size()>0)
    {
        //            pcl::copyPointCloud(*euclidian_cloud,*temp_cloud);
        //            for (uint i=0;i<cluster_indices[0].indices.size()/*outliers->indices.size()*/;i++)
        //            {
        //                temp_cloud->points[/*outliers->indices[i]*/cluster_indices[0].indices[i]].f=2;
        //            }

        pcl::copyPointCloud(*cloud_input,*plane_outliers,*temp_cloud);

    }

    temp_cloud->width =
            static_cast<uint32_t>(temp_cloud->points.size());
    temp_cloud->height = 1;
    temp_cloud->is_dense = true;

    pcl::io::savePCDFile("cloud_temp.pcd", *cloud_input);

    template_inliers=*plane_inliers;
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






