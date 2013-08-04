#include <template_library/template_library.h>

/*
 * template_library.cpp
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */

#include <pcl/filters/filter.h>
#include <ros/package.h>


TemplateLibrary::TemplateLibrary():
    nh_ ("~/template_library"),
    dense_reconstructor_(),
    pcd_io_(),
    reconfig_srv_(nh_)
{
    reconfig_callback_ = boost::bind (&TemplateLibrary::reconfigCallback, this, _1, _2);
    reconfig_srv_.setCallback (reconfig_callback_);
//    filenames_.push_back("/home/karol/Desktop/template2.pcd");

//    data_directory_="/home/karol/ros_workspace/interactive_object_recognition/template_library/data/";
//    source_directory_="/home/karol/ros_workspace/interactive_object_recognition/template_library/source";
    std::string package_path = ros::package::getPath("template_library");
    source_directory_ = package_path + "/source/";
    data_directory_ = package_path + "/data/";
}

TemplateLibrary::~TemplateLibrary ()
{
}

void TemplateLibrary::reconfigCallback (template_library::LibraryConfig&config, uint32_t level)
{
//    data_directory_=config.data_directory;
//    source_directory_=config.source_directory;

}


void TemplateLibrary::loadClouds()
{
    std::set<std::string> file_names;
    std::set<std::pair<std::string,std::string> > directory_names_full_list;
    pcd_io_.getDirectoryListWithExtension(source_directory_,file_names);
    for (std::set<std::string>::iterator itr = file_names.begin (); itr != file_names.end (); itr++)
      {
          std::set<std::string> directory_names;
          std::set<std::pair<std::string,std::string> > directory_names_pair;

          namespace fs = boost::filesystem;

          fs::path path (*itr);

          ROS_DEBUG_STREAM("folder:"<<path.stem());
          pcd_io_.getFileListWithExtension(*itr,".pcd",directory_names);

          for (std::set<std::string>::iterator itr = directory_names.begin (); itr != directory_names.end (); itr++)
          {
              directory_names_pair.insert(std::pair<std::string,std::string>(path.stem().string(),*itr));
          }

          directory_names_full_list.insert(directory_names_pair.begin(), directory_names_pair.end());

      }

    std::copy(directory_names_full_list.begin(), directory_names_full_list.end(), std::back_inserter(filenames_));

    for (std::set<std::pair<std::string,std::string> >::iterator itr = directory_names_full_list.begin (); itr != directory_names_full_list.end (); itr++)
    {
        ROS_INFO_STREAM("Loading : directory: "<<itr->second<<", object name: "<<itr->first);
    }




    for(uint i=0;i<filenames_.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(
                    new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_for_dense(
                    new pcl::PointCloud<pcl::PointXYZLRegionF>);

        pcl::io::loadPCDFile(filenames_[i].second, *cloud_input);

        pcl::copyPointCloud(*cloud_input,*cloud_for_dense);

        clouds_dense_.push_back(cloud_for_dense);
        clouds_input_.push_back(cloud_input);
        names_.push_back(filenames_[i].first);
    }
}

void TemplateLibrary::generateTemplateData()
{
    loadClouds();

    for(uint i=0;i<clouds_input_.size();i++)
    {

        pcl::PointIndices template_inliers;
        generateTemplateDataEuclidean(clouds_input_[i],clouds_dense_[i],template_inliers);
        cv::Mat image_no_plane=restoreCVMatNoPlaneFromPointCloud(*clouds_input_[i],template_inliers);
        cv::Mat image=restoreCVMatFromPointCloud(clouds_input_[i]);

        Template temp(image,image_no_plane,clouds_input_[i],clouds_dense_[i],names_[i]);
        templates_.push_back(temp);
    }
    saveTemplates();

}

void TemplateLibrary::saveTemplates()
{
    for(uint i=0;i<templates_.size();i++)
    {
        Template temp=templates_[i];

        std::stringstream ss_cloud_rgb;
        std::stringstream ss_cloud_inliers;
        std::stringstream ss_image;
        std::stringstream ss_no_plane_image;

        generateNames(i,ss_image,ss_no_plane_image,ss_cloud_rgb,ss_cloud_inliers);

        ROS_DEBUG_STREAM(ss_cloud_rgb.str());
        pcl::io::savePCDFile(ss_cloud_inliers.str(),*temp.cloud_with_inliers_ptr_);
        pcl::io::savePCDFile(ss_cloud_rgb.str(),*temp.cloud_ptr_);
        cv::imwrite( ss_image.str(), temp.image_);
        cv::imwrite( ss_no_plane_image.str(), temp.no_plane_image_);

    }

}

std::vector<Template> TemplateLibrary::loadTemplates()
{
    std::set<std::string> file_names;
    int i=0;
    pcd_io_.getDirectoryListWithExtension(source_directory_,file_names);
    for (std::set<std::string>::iterator itr = file_names.begin (); itr != file_names.end (); itr++)
      {
          std::set<std::string> directory_names;

          namespace fs = boost::filesystem;

          fs::path path (*itr);

          ROS_DEBUG_STREAM("folder:"<<path.stem());
          pcd_io_.getFileListWithExtension(*itr,".pcd",directory_names);

          for (std::set<std::string>::iterator itr = directory_names.begin (); itr != directory_names.end (); itr++)
          {

              std::stringstream ss_cloud_rgb_;
              std::stringstream ss_cloud_inliers_;
              std::stringstream ss_image_;
              std::stringstream ss_no_plane_image_;

              std::string directory=data_directory_+path.stem().string()+ "_template";
              ss_cloud_rgb_<<directory <<i<<"cloud_rgb.pcd";
              ss_cloud_inliers_<<directory <<i<<"cloud_inliers.pcd";
              ss_image_<<directory <<i<<"image.jpg";
              ss_no_plane_image_<<directory <<i<<"image_no_plane.jpg";
              i++;
              ROS_INFO_STREAM(ss_cloud_rgb_.str());
              ROS_INFO_STREAM(ss_cloud_inliers_.str());
              ROS_INFO_STREAM(ss_image_.str());
              ROS_INFO_STREAM(ss_no_plane_image_.str());
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(
                          new pcl::PointCloud<pcl::PointXYZRGB>);

              pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_inliers(
                          new pcl::PointCloud<pcl::PointXYZLRegionF>);

              pcl::io::loadPCDFile(ss_cloud_rgb_.str(), *cloud_rgb);
              pcl::io::loadPCDFile(ss_cloud_inliers_.str(), *cloud_inliers);
              cv::Mat image=cv::imread( ss_image_.str(), 1 );
              cv::Mat image_no_plane=cv::imread(ss_no_plane_image_.str(),1);

              Template temp(image, image_no_plane, cloud_rgb, cloud_inliers,path.stem().string());
              templates_.push_back(temp);

          }
    }
    return getTemplates();
}


std::vector<Template> TemplateLibrary::getTemplates()
{
       return templates_;
}



void TemplateLibrary::generateNames(const int &i,std::stringstream &ss_image,std::stringstream &ss_no_plane_image,std::stringstream &ss_cloud_rgb,std::stringstream &ss_cloud_inliers)
{
    std::string directory=data_directory_+templates_[i].name_+"_template";

    ROS_DEBUG_STREAM(directory);
    ss_cloud_rgb<<directory <<i<<"cloud_rgb.pcd";
    ss_cloud_inliers<<directory <<i<<"cloud_inliers.pcd";
    ss_image<<directory <<i<<"image.jpg";
    ss_no_plane_image<<directory <<i<<"image_no_plane.jpg";

}




cv::Mat TemplateLibrary::restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr)
{
    cv::Mat restored_image = cv::Mat (cloud_in_ptr->height, cloud_in_ptr->width, CV_8UC3);
    for (uint rows = 0; rows < cloud_in_ptr->height; rows++)
    {
        for (uint cols = 0; cols < cloud_in_ptr->width; ++cols)
        {
            restored_image.at<cv::Vec3b> (rows, cols)[0] = cloud_in_ptr->at (cols, rows).b;
            restored_image.at<cv::Vec3b> (rows, cols)[1] = cloud_in_ptr->at (cols, rows).g;
            restored_image.at<cv::Vec3b> (rows, cols)[2] = cloud_in_ptr->at (cols, rows).r;
        }
    }
    return restored_image;
}


cv::Mat TemplateLibrary::restoreCVMatNoPlaneFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB> cloud_in,pcl::PointIndices &inliers)
{
    cv::Mat restored_image = cv::Mat (cloud_in.height, cloud_in.width, CV_8UC3);


    for (uint i = 0; i < inliers.indices.size(); i++)
    {
        //        int rows=static_cast<int>(inliers.indices[i]/cloud_in_ptr->width);
        //        int cols=static_cast<int>(inliers.indices[i]%cloud_in_ptr->width);
        cloud_in.points[inliers.indices[i]].b = 0;
        cloud_in.points[inliers.indices[i]].g = 0;
        cloud_in.points[inliers.indices[i]].r = 0;

    }

    int border_no_nan_width=180;
    int border_no_nan_height=180;

    for (uint rows = 0; rows < cloud_in.height; rows++)
    {
        for (uint cols = 0; cols < cloud_in.width; ++cols)
        {                
            if((std::isnan(cloud_in.at(cols, rows).x))
                    && ((static_cast<int>(cols)<border_no_nan_width)||(static_cast<int>(cols)>cloud_in.width-border_no_nan_width)
                        || (static_cast<int>(rows)<border_no_nan_height)||(static_cast<int>(rows)>cloud_in.height-border_no_nan_height)) )
            {
                cloud_in.at (cols, rows).b = 0;
                cloud_in.at (cols, rows).g = 0;
                cloud_in.at (cols, rows).r = 0;
            }



            restored_image.at<cv::Vec3b> (rows, cols)[0] =cloud_in.at (cols, rows).b;
            restored_image.at<cv::Vec3b> (rows, cols)[1] =cloud_in.at (cols, rows).g;
            restored_image.at<cv::Vec3b> (rows, cols)[2] =cloud_in.at (cols, rows).r;

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


void TemplateLibrary::generateTemplateDataEuclidean(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input,pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input_with_inliers, pcl::PointIndices &template_inliers)
{
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr plane_outliers(new pcl::PointIndices);

//    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::removeNaNFromPointCloud(*cloud_input,*cloud_temp,*indices);

    planeSegmentation(cloud_input,coefficients,*plane_inliers);
    generatePlaneOutliers(*plane_inliers,cloud_input->size(),*plane_outliers);

//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
//    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(
//                new pcl::search::KdTree<pcl::PointXYZRGB>);

//    ec.setClusterTolerance(0.02);

//    ec.setMinClusterSize(100);
//    ec.setSearchMethod(kd_tree);
//    ec.setInputCloud(cloud_input);
    //    pcl::PointIndices::Ptr outliers2(new pcl::PointIndices);

    //        for (uint z=0;z<inliers->indices.size();z++)
    //            if((inliers->indices[z]<=euclidian_cloud->points.size())&&(inliers->indices[z]>=0))
    //                outliers2->indices.push_back(inliers->indices[z]);

    //        sort( outliers2->indices.begin(), outliers2->indices.end() );
    //        outliers2->indices.erase( unique( outliers2->indices.begin(), outliers2->indices.end() ), outliers2->indices.end() );
    //            if((inliers->indices[z]>=euclidian_cloud->points.size())||(inliers->indices[z]<0))
    //                ROS_INFO("HERE I AM");

    //        for (uint j=0;j<(euclidian_cloud->points.size());j++)
    //            outliers2->indices.push_back(j);
//    ec.setIndices(plane_inliers);
    //        ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr temp_cloud(
                new pcl::PointCloud<pcl::PointXYZLRegionF>);


        pcl::copyPointCloud(*cloud_input,*temp_cloud);
        for (uint i=0;i<plane_outliers->indices.size();i++)
        {
            temp_cloud->points[plane_outliers->indices[i]].f=2;
        }

    temp_cloud->width =
            static_cast<uint32_t>(temp_cloud->points.size());
    temp_cloud->height = 1;
    temp_cloud->is_dense = true;

//    pcl::io::savePCDFile("/home/karol/ros_workspace/interactive_object_recognition/template_library/data/cloud_temp.pcd", *temp_cloud);
    pcl::copyPointCloud(*temp_cloud,*cloud_input_with_inliers);

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






