
#ifndef TEMPLATELIBRARY_H_
#define TEMPLATELIBRARY_H_

#include <dense_reconstruction/DenseReconstruction.h>

#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/template_library/LibraryConfig.h"

#include <pcd_io/pcd_io.h>

/*
 * template_library.h
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */

struct Template
{
    Template(cv::Mat image, cv::Mat no_plane_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
            pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_with_inliers_ptr, std::string name, int& pose):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        cloud_with_inliers_ptr_.reset(new pcl::PointCloud<pcl::PointXYZLRegionF>(*cloud_with_inliers_ptr));
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
        image_=image;
        no_plane_image_=no_plane_image;
        name_=name;
        pose_=pose;
    }

    Template(cv::Mat image, cv::Mat no_plane_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
            pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_with_inliers_ptr, std::string name):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        cloud_with_inliers_ptr_.reset(new pcl::PointCloud<pcl::PointXYZLRegionF>(*cloud_with_inliers_ptr));
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
        image_=image;
        no_plane_image_=no_plane_image;
        name_=name;
    }

    Template(cv::Mat image, cv::Mat no_plane_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, std::string name):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
        image_=image;
        no_plane_image_=no_plane_image;
        name_=name;
    }

    Template(cv::Mat image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, std::string name):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud_ptr));
        image_=image;
        name_=name;
    }

    Template(cv::Mat image, std::string name):
        cloud_with_inliers_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        image_=image;
        name_=name;
    }

    Template(cv::Mat image, cv::Mat no_plane_image, std::string name):
        cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        image_=image;
        no_plane_image_=no_plane_image;
        name_=name;
    }

    inline void setName(const std::string &name)
    {
        name_=name;
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
    std::string name_;
    uint pose_;

};


class TemplateLibrary
{
public:
    TemplateLibrary();
    virtual ~TemplateLibrary();
    void generateTemplateData(const std::string& source_directory, const std::string& data_directory);
    std::vector<Template> loadTemplates(const std::string& type = "template");
    std::vector<Template> getTemplates();
    cv::Mat restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr);



private:
    void reconfigCallback (template_library::LibraryConfig&config, uint32_t level);
    void loadClouds(const std::string& source_directory);
    void saveTemplates(const std::string& data_directory);
    int getIndex(const  pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr& cloud);
    void planeSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::ModelCoefficients &coefficients,
                    pcl::PointIndices &inliers);
    void generateTemplateDataActive(pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input , pcl::PointIndices &template_inliers);
    void generateTemplateDataEuclidean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input ,pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input_with_inliers, pcl::PointIndices &template_inliers);
    void generatePlaneOutliers(const pcl::PointIndices& inliers,const uint &cloud_size, pcl::PointIndices &outliers);
    void generateNames(const std::string& data_directory,const int &i,std::stringstream &ss_image,std::stringstream &ss_no_plane_image,std::stringstream &ss_cloud_rgb,std::stringstream &ss_cloud_inliers);

    cv::Mat restoreCVMatNoPlaneFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB> cloud_in,pcl::PointIndices &inliers);


    ros::NodeHandle nh_;
    std::vector<std::pair<std::string,std::string> > filenames_;
    std::vector<std::string> names_;
    boost::shared_ptr<DenseReconstruction<pcl::PointXYZLRegionF> > dense_reconstructor_;
    PcdIO pcd_io_;
    std::vector<pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr > clouds_dense_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds_input_;
    std::vector<Template> templates_;
    dynamic_reconfigure::Server<template_library::LibraryConfig> reconfig_srv_;
    dynamic_reconfigure::Server<template_library::LibraryConfig>::CallbackType
    reconfig_callback_;
    std::string data_directory_;
    std::string source_directory_;
    std::string training_directory_;
    std::string training_data_directory_;



};

#endif /* TEMPLATELIBRARY_H_ */

