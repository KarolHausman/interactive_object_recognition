#include <dense_reconstruction/DenseReconstruction.h>

#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>

/*
 * template_library.h
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */



class TemplateLibrary
{
public:
    TemplateLibrary();
    void addLocation(const std::string &location);
    void generateTemplateData();

private:
    void loadClouds();
    int getIndex(const  pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr& cloud);
    void planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::ModelCoefficients &coefficients,
                    pcl::PointIndices &inliers);
    void generateTemplateDataActive(pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input , pcl::PointIndices &template_inliers);
    void generateTemplateDataEuclidean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input , pcl::PointIndices &template_inliers);
    void generatePlaneOutliers(const pcl::PointIndices& inliers,const uint &cloud_size, pcl::PointIndices &outliers);
    cv::Mat restoreCVMatNoPlaneFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr,pcl::PointIndices &inliers);


    ros::NodeHandle nh_;
    std::vector<std::string> filenames_;
    boost::shared_ptr<DenseReconstruction<pcl::PointXYZLRegionF> > dense_reconstructor_;
    std::vector<pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr > clouds_dense_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds_input_;


};
