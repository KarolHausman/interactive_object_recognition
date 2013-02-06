/*
* pcd_io.h
*
* Created on: Feb 5, 2013
* Author: Karol Hausman
*/

#ifndef PCDIO_H_
#define PCDIO_H_

//pcl
#include <pcl/io/pcd_io.h>

#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"

#include <pcl/point_types.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>

class PcdIO
{
  public:
    PcdIO ();
    virtual ~PcdIO ();

    void loadImagesFromDir (std::string directory, std::vector<cv::Mat>& images);

    void loadPointcloudsFromDir (std::string directory, std::vector<pcl::PointCloud<pcl::PointXYZRGB> >& pointclouds);

    void getDirectoryListWithExtension (const std::string& input_dir,
       std::set<std::string>& file_list);

    void getDirectoryListWithExtension (const std::string& input_dir,
       std::set<std::string>& file_list,std::set<std::string>& folder_list);

    void getFileListWithExtension(const std::string& input_dir, const std::string& input_ext,
        std::set<std::string>& file_list);
  private:

};
#endif // PCDIO_H_
