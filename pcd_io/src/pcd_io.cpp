/*
* pcd_io.cpp
*
* Created on: Feb 5, 2013
* Author: Karol Hausman
*/

#include <pcd_io/pcd_io.h>


PcdIO::PcdIO ()
{
  ros::NodeHandle nh ("~");
}

PcdIO::~PcdIO ()
{
}


void PcdIO::getDirectoryListWithExtension (const std::string& input_dir,
   std::set<std::string>& file_list)
{
    std::ifstream input_stream;
    namespace fs = boost::filesystem;

    fs::path path (input_dir);

    fs::directory_iterator itr (path), eod;

    BOOST_FOREACH(fs::path const &file_path, std::make_pair(itr, eod))
    {  if(is_directory(file_path))
        {
            ROS_DEBUG_STREAM("directory:"<<file_path.c_str());
            file_list.insert(file_path.string());

        }
    }

}

void PcdIO::getFileListWithExtension (const std::string& input_dir,
    const std::string& input_ext, std::set<std::string>& file_list)
{
  std::ifstream input_stream;
  namespace fs = boost::filesystem;

  fs::path path (input_dir);

  fs::directory_iterator itr (path), eod;

  BOOST_FOREACH(fs::path const &file_path, std::make_pair(itr, eod))
  { if(is_regular_file(file_path))
      {
          ROS_DEBUG_STREAM("file:" << file_path.c_str() << " ext " << file_path.extension().c_str());
          if(file_path.extension().string() == input_ext)
              file_list.insert(file_path.string());
      }

  }
}

void PcdIO::loadImagesFromDir (std::string directory, std::vector<cv::Mat>& images)
{
  std::set<std::string> file_list;
  getFileListWithExtension (directory, ".png", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    cv::Mat cv_image = cv::imread (*itr, CV_LOAD_IMAGE_COLOR);
    images.push_back(cv_image);
  }
}



void PcdIO::loadPointcloudsFromDir (std::string directory, std::vector<pcl::PointCloud<pcl::PointXYZRGB> >& pointclouds)
{
  std::set<std::string> file_list;
  pcl::PointCloud<pcl::PointXYZRGB> temp_pointcloud;
  getFileListWithExtension (directory, ".pcd", file_list);
  for (std::set<std::string>::iterator itr = file_list.begin (); itr != file_list.end (); itr++)
  {
    pcl::io::loadPCDFile(*itr, temp_pointcloud);

    pointclouds.push_back (temp_pointcloud);
  }
}
