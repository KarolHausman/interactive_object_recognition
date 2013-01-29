#include <dense_reconstruction/DenseReconstruction.h>

#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>

/*
 * template_library.h
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */

struct Template
{
    Template(cv::Mat image, cv::Mat no_plane_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
            pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_with_inliers_ptr):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        cloud_with_inliers_ptr_.reset(new pcl::PointCloud<pcl::PointXYZLRegionF>(*cloud_with_inliers_ptr));
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
        image_=image;
        no_plane_image_=no_plane_image;
    }

    Template(cv::Mat image, cv::Mat no_plane_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
        image_=image;
        no_plane_image_=no_plane_image;
    }

    Template(cv::Mat image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
        image_=image;
    }

    Template(cv::Mat image):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        image_=image;
    }

    Template(cv::Mat image, cv::Mat no_plane_image):
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        image_=image;
        no_plane_image_=no_plane_image;
    }

    inline void setNoPlaneImage(const cv::Mat &no_plane_image)
    {
        no_plane_image_=no_plane_image;
    }

    inline void setImage(const cv::Mat &image)
    {
        image_=image;
    }

    inline void setCloudPtr(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr)
    {
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
    }

    inline void setCloudWithInliersPtr(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_with_inliers_ptr)
    {
        cloud_with_inliers_ptr_.reset(new pcl::PointCloud<pcl::PointXYZLRegionF>(*cloud_with_inliers_ptr));
    }


    cv::Mat no_plane_image_;
    cv::Mat image_;
    pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_with_inliers_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;

};


class TemplateLibrary
{
public:
    TemplateLibrary();
    void addLocation(const std::string &location);
    void generateTemplateData();
    std::vector<Template> loadTemplates();
    std::vector<Template> getTemplates();
    cv::Mat restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr);



private:
    void loadClouds();
    void saveTemplates();
    int getIndex(const  pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr& cloud);
    void planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::ModelCoefficients &coefficients,
                    pcl::PointIndices &inliers);
    void generateTemplateDataActive(pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input , pcl::PointIndices &template_inliers);
    void generateTemplateDataEuclidean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input ,pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input_with_inliers, pcl::PointIndices &template_inliers);
    void generatePlaneOutliers(const pcl::PointIndices& inliers,const uint &cloud_size, pcl::PointIndices &outliers);
    void generateNames(const int &i,std::stringstream &ss_image,std::stringstream &ss_no_plane_image,std::stringstream &ss_cloud_rgb,std::stringstream &ss_cloud_inliers);
    cv::Mat restoreCVMatNoPlaneFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB> cloud_in,pcl::PointIndices &inliers);


    ros::NodeHandle nh_;
    std::vector<std::string> filenames_;
    boost::shared_ptr<DenseReconstruction<pcl::PointXYZLRegionF> > dense_reconstructor_;
    std::vector<pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr > clouds_dense_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds_input_;
    std::vector<Template> templates_;


};
