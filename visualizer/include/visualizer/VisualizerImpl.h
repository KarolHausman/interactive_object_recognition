#ifndef VISUALIZERIMPL_H
#define VISUALIZERIMPL_H

#include <pcl_typedefs/pcl_typedefs.h>
#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>

class VisualizerImpl
{
public:

    VisualizerImpl():
        point_cloud_number_(0)
    {
        viewer_.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer_->initCameraParameters();
        viewer_->setBackgroundColor (0, 0, 0);
    }


    void spinOnce(){
        viewer_->spinOnce (100);
    }

    void removeAllClouds()
    {
      viewer_->removeAllPointClouds();
    }

    void addPointCloudColor(PointCloudPtr& input_cloud){

        std::stringstream convertToString;
        convertToString<<point_cloud_number_;
        point_cloud_name_=convertToString.str();
        pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(input_cloud);
        viewer_->addPointCloud<PointType>(input_cloud,rgb,point_cloud_name_);
        point_cloud_number_++;
        this->spinOnce();

    }
    void addPointCloud(PointCloudPtr& input_cloud){

        std::stringstream convertToString;
        convertToString<<point_cloud_number_;
        point_cloud_name_=convertToString.str();
        viewer_->addPointCloud<PointType>(input_cloud,point_cloud_name_);
        point_cloud_number_++;
        this->spinOnce();

    }

private:

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    std::string point_cloud_name_;
    int point_cloud_number_;
};

#endif // VISUALIZERIMPL_H
