/*
 * main_library.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: Karol Hausman
 */

#include <ros/ros.h>
#include "template_library/template_library.h"

int main(int argc, char **argv)
{

        ros::init(argc, argv, "library");

        TemplateLibrary library;

        library.generateTemplateData();
//        library.loadTemplates();
//        pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_for_dense(
//                    new pcl::PointCloud<pcl::PointXYZLRegionF>);
//        pcl::io::loadPCDFile("/home/karol/Desktop/RESULT_plane.pcd", *cloud_for_dense);
//        for (uint i=0;i<cloud_for_dense->size();i++)
//        {
//            if (cloud_for_dense->points[i].reg==4)
//            {
//                cloud_for_dense->points[i].x=0;
//                cloud_for_dense->points[i].y=0;
//                cloud_for_dense->points[i].z=0;


//            }
//        }
//        pcl::io::savePCDFile("/home/karol/Desktop/final.pcd",*cloud_for_dense);


}

