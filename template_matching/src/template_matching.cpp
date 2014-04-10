/*
 * template_matching.cpp
 *
 *  Created on: jan 8, 2013
 *      Author: Karol Hausman
 */

#include "template_matching/template_matching.h"

#include "util.cpp"


const static std::string template_filename ="/home/karol/Desktop/frame0000.jpg";
const static std::string subscribe_topic ="/camera/rgb/image_color";
const static std::string image_matches_topic = "/image_matches";
const static std::string cloud_name="/home/karol/Desktop/template1.pcd";
const static double plane_detection_distance_threshold = 0.02;



TemplateMatcher::TemplateMatcher(ros::NodeHandle nh):
    matcher_(),
    ransac_transformer_(),
    image_transport_(nh),
    reconfig_srv_(nh),
    current_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
    dense_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),    
    template_library_()

{
    first_one_=true;
    publish_time_=ros::Time::now();
//    template_image_ = cv::Mat (cvLoadImage (template_filename.c_str (), CV_LOAD_IMAGE_COLOR));
//    template_library_.generateTemplateData();
//    template_library_.loadTemplates();
//    template_image_ =template_library_.loadTemplates()[0].no_plane_image_;

//    template_image_ =template_library_.loadTemplates()[0].image_;
//    library_templates_ = template_library_.loadTemplates();

    int i = 0;
//    std::vector<std::string> names;
//    for (std::vector<Template>::iterator it=library_templates_.begin(); it!=library_templates_.end(); it++)
//    {
//        template_images_.push_back( it->image_);

//        std::vector<int> init_vector;
//        init_vector.push_back(0);


//        template_bin_.push_back(init_vector);
//        template_map_.insert(std::pair<int, std::string> (i, it->name_));
////        names.push_back(it->name_);

//        template_single_map_.insert(std::pair<std::string,int>(it->name_,0));
//        template_single_map_history_.insert(std::pair<std::string,int>(it->name_,0));


//        i++;

//    }
//    sum_history_=0;
//    absolute_matches_threshold_ = 30;
//    single_ratio_threshold_ = 0.85;
//    cumultative_ratio_threshold_ = 0.99;

    reconfig_callback_ = boost::bind (&TemplateMatcher::reconfigCallback, this, _1, _2);
    reconfig_srv_.setCallback (reconfig_callback_);

//    extractTemplateFeatures(template_images_, template_keypoints_, template_descriptors_);


    //    subscriber_ = image_transport_.subscribe(subscribe_topic, 1, &TemplateMatcher::imageCallback, this);
    cloud_subscriber_ = nh.subscribe("/camera/depth_registered/points", 1, &TemplateMatcher::cloudCallback, this);

    publisher_ = image_transport_.advertise (image_matches_topic, 1);
}


void TemplateMatcher::reconfigCallback (template_matching::MatcherConfig&config, uint32_t level)
{
    absolute_matches_threshold_ = config.absolute_matches_threshold;
    single_ratio_threshold_ = config.single_ratio_threshold;
    cumultative_ratio_threshold_ = config.cumultative_ratio_threshold;

    if (config.empty_history_buffer)
    {
        sum_history_ = 0;
        for (std::map<std::string,int>::iterator it=template_single_map_history_.begin(); it!=template_single_map_history_.end(); it++)
        {
            it->second = 0;
        }
        config.empty_history_buffer = false;
    }

}

void TemplateMatcher::detectPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in_ptr, pcl::ModelCoefficients &coefficients,
                                pcl::PointIndices &inliers)
{
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(plane_detection_distance_threshold);
            seg.setInputCloud(cloud_in_ptr);
            seg.segment(inliers, coefficients);

}


bool cvPointEqualTo(cv::Point pointA,cv::Point pointB)
{
    return ((abs(pointA.x-pointB.x)<=1)&&(abs(pointA.y-pointB.y)<=1));
}


void TemplateMatcher::printBins()
{
    for (uint i=0; i < template_bin_.size() ; i++)
    {
        std::stringstream stream;

        stream<<template_map_[i]<<" : "<<template_bin_[i].back();

        std::cout<<stream.str()<<std::endl;

    }
//    std::cout<<std::endl;
}



void TemplateMatcher::printAllFramesBins()
{
    for (uint i=0; i < template_bin_.size() ; i++)
    {
        std::stringstream stream;

        stream<<template_map_[i]<<" : ";

        for (uint j=0; j < template_bin_[i].size() ; j++)
        {
            stream<<", "<<template_bin_[i][j]<<", ";
        }

        std::cout<<stream.str()<<std::endl;

    }

}


void TemplateMatcher::checkRecognition (std::string &object_name)
{

//    std::vector<int> template_scores;
    int sum = 0;
    bool absolute_threshold_check = false;
    bool single_ratio_check = false;
    bool cumultative_ratio_check = false;
    std::string name_absolute;
    std::string name_single_ratio;
    std::string name_cumultative_ratio;

    //not neccessary - emptying the history
    for (std::map<std::string,int>::iterator it=template_single_map_.begin(); it!=template_single_map_.end(); it++)
    {
        it->second = 0;
    }



    for (uint i=0; i < template_bin_.size() ; i++)
    {
        template_single_map_ [template_map_[i]] += template_bin_[i].back();

        template_single_map_history_ [template_map_[i]] += template_bin_[i].back();



        sum += template_bin_[i].back();

        sum_history_ += template_bin_[i].back();

        if(template_bin_[i].back() > absolute_matches_threshold_)
        {
            absolute_threshold_check = true;
            name_absolute = template_map_[i];
        }

    }

    std::cout<<"current sum = "<<sum<<std::endl;
    std::cout<<"cumultative sum = "<<sum_history_<<std::endl;

    std::cout<<std::endl;


    std::cout<<"Single scores for objects: "<<std::endl;
    for (std::map<std::string,int>::iterator it=template_single_map_.begin(); it!=template_single_map_.end(); it++)
    {
        std::cout<<it->first<<" = "<<it->second<<std::endl;

    }
    std::cout<<std::endl;


    std::cout<<"Ratio single scores for objects: "<<std::endl;
    for (std::map<std::string,int>::iterator it=template_single_map_.begin(); it!=template_single_map_.end(); it++)
    {
        std::cout<<it->first<<" = "<<(double)it->second/sum<<std::endl;

        if((double)it->second/sum > single_ratio_threshold_)
        {
            single_ratio_check=true;
            name_single_ratio = it->first;
        }

    }
    std::cout<<std::endl;


    std::cout<<"Cumultative scores for objects: "<<std::endl;
    for (std::map<std::string,int>::iterator it=template_single_map_history_.begin(); it!=template_single_map_history_.end(); it++)
    {
        std::cout<<it->first<<" = "<<it->second<<std::endl;

    }
    std::cout<<std::endl;


    std::cout<<"Ratio cumultative scores for objects: "<<std::endl;
    for (std::map<std::string,int>::iterator it=template_single_map_history_.begin(); it!=template_single_map_history_.end(); it++)
    {
        std::cout<<it->first<<" = "<<(double)it->second/sum_history_<<std::endl;
        if((double)it->second/sum_history_ > cumultative_ratio_threshold_)
        {
            cumultative_ratio_check=true;
            name_cumultative_ratio = it->first;
        }

    }



    std::cout<<std::endl;



    //final check
    if(absolute_threshold_check && single_ratio_check)
    {
        if(name_absolute == name_single_ratio)
            object_name = name_single_ratio;
    }

    if( absolute_threshold_check && cumultative_ratio_check)
    {
        if(name_absolute == name_cumultative_ratio)
            object_name = name_cumultative_ratio;
    }
    if(single_ratio_check && cumultative_ratio_check)
    {
        if(name_single_ratio == name_cumultative_ratio)
            object_name = name_cumultative_ratio;
    }


}

void TemplateMatcher::extractTemplateFeatures (const std::vector<cv::Mat>& images, std::vector<std::vector<cv::KeyPoint> >& keypoints, std::vector<cv::Mat>& descriptors)
{
//    std::vector<cv::Mat> descriptors_vector;
//    int descriptors_width = 0;

    // for all the images in the database extract all features
    for (uint i=0; i<images.size(); i++)
    {
        std::vector<cv::KeyPoint> temp_keypoints;
        cv::Mat temp_descriptors;
        matcher_.getFeatures(images[i], temp_keypoints, temp_descriptors);

        std::cerr<<"number of descriptors for "<<i<<" image: "<<temp_descriptors.rows<<std::endl;

//        keypoints.insert(keypoints.end(), temp_keypoints.begin(), temp_keypoints.end());
        keypoints.push_back(temp_keypoints);

        /*if(i==1)*/ descriptors.push_back(temp_descriptors);


//        for(int i = 0; i < temp_descriptors.rows; i++)
//        {
//            descriptors.push_back(temp_descriptors.row(i));
//        }
//        keypoints = temp_keypoints;
//        descriptors = temp_descriptors;



//        descriptors_vector.push_back(temp_descriptors);
//        descriptors_width += temp_descriptors.rows;
    }
//    std::cerr<<"number of descriptors for everything: "<<descriptors.rows<<std::endl;

//    for(std::vector<cv::Mat>::iterator it = descriptors_vector.begin(); it != descriptors_vector.end(); it++)
//    {
//        for(uint i = 0; i < it->rows; i++)
//        {
//            keypoints.push_back(it->row(i));
//        }
//    }



}


void TemplateMatcher::cloudCallback (const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud_color_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::fromROSMsg(*cloud_msg,*dense_cloud_color_ptr);
//    pcl::copyPointCloud(*dense_cloud_color_ptr, *current_cloud_ptr_);
//    cv::Mat search_image = template_library_.restoreCVMatFromPointCloud(dense_cloud_color_ptr);

//    cv::Mat img_matches;
//    std::vector<cv::Point2f> template_points,search_points;
//    uint max_matches = 0;






//    for (uint i=0; i<template_images_.size(); i++)
//    {

        cv::Mat temp_img_matches;
        std::vector<cv::Point2f> temp_template_points, temp_search_points;
        std::vector<cv::DMatch> temp_matches;


//        matcher_.getDescriptorMatches(template_images_[1], template_images_[1], template_keypoints_, template_descriptors_, temp_img_matches, temp_template_points, temp_search_points, temp_matches);

        std::stringstream d_image;
        d_image << "/home/karol/ros_workspace/interactive_object_recognition/template_library/data/first_home_book_template0image.jpg";
        cv::Mat database_image=cv::imread( d_image.str(), 1 );

        std::stringstream s_image;
        s_image << "/home/karol/ros_workspace/interactive_object_recognition/template_library/training_data/first_home_book_template0image.jpg";
        cv::Mat search_image=cv::imread( s_image.str(), 1 );

        std::vector<cv::KeyPoint> temp_keypoints;
        cv::Mat temp_descriptors;
        matcher_.getFeatures(database_image, temp_keypoints, temp_descriptors);

        matcher_.getDescriptorMatches(database_image, search_image, temp_keypoints, temp_descriptors, temp_img_matches, temp_template_points, temp_search_points, temp_matches);


//        matcher_.getMatches(database_image, search_image, temp_img_matches, temp_template_points, temp_search_points);


//        template_bin_[i].push_back(temp_template_points.size());


//        if (temp_template_points.size() > max_matches)
//        {
//            template_image_ = template_images_[i];
//            max_matches = temp_template_points.size();
//            img_matches = temp_img_matches;
//            template_points = temp_template_points;
//            search_points = temp_search_points;
//        }

//    }
//recognition part
//    printBins();
//    std::string object ="NOT_RECOGNIZED";
//    checkRecognition(object);
//    std::cout<<"RECOGNIZED OBJECT ==== "<<object<<std::endl;
//    std::cout<<std::endl;
//    std::cout<<std::endl;

    //ransacing part
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_color_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

//    for(uint i=0; i < template_points.size(); i++)
//    {
//        template_color_cloud_ptr->points.push_back(template_library_.getTemplates()[0].cloud_ptr_->at(template_points[i].x, template_points[i].y));
//        search_cloud_ptr->points.push_back(current_cloud_ptr_->at(search_points[i].x, search_points[i].y));
//    }
//    pcl::copyPointCloud(*template_color_cloud_ptr,*template_cloud_ptr);

//    Eigen::Matrix4f transformation_result=Eigen::Matrix4f::Identity();
    int inliers(0);
//    if (search_cloud_ptr->points.size()>0){

//        inliers = ransac_transformer_.ransacUmeyama(search_cloud_ptr,template_cloud_ptr,transformation_result);
//        ROS_DEBUG_STREAM("transform: \n"<<transformation_result);

//        publishTF(transformation_result,"start","estimate");

//    }
    ros::Duration time_since_last_callback = ros::Time::now() - publish_time_;
    publish_time_ = ros::Time::now();

    drawOnImage(inliers, static_cast<int>(temp_template_points.size()), (1/time_since_last_callback.toSec()), temp_img_matches);
/*
    Testing for flickering matches -BEGIN
*/

//    cv::Scalar color(255,0,0);
//    int square_edge=32;
//    if (first_one_)
//    {
//        upper_left_.x=template_points[0].x-4;
//        upper_left_.y=template_points[0].y-4;
//        bottom_right_.x=template_points[0].x;
//        bottom_right_.y=template_points[0].y;

//        search_upper_left_.x=search_points[0].x-4+template_image_.cols;
//        search_upper_left_.y=search_points[0].y-4;
//        search_bottom_right_.x=search_points[0].x+template_image_.cols;
//        search_bottom_right_.y=search_points[0].y;

//        first_one_=false;
//        cv::Size size_four(square_edge*2,square_edge*2);
//        cv::Mat image_four_temp(size_four,CV_8UC3);

//        image_four_=image_four_temp;

//    }
//    cv::rectangle(img_matches,upper_left_,bottom_right_,color);
//    cv::rectangle(img_matches,search_upper_left_,search_bottom_right_,color);


//    if(std::find_if(template_points.begin(), template_points.end(),boost::bind(&cvPointEqualTo,bottom_right_,_1)) != template_points.end())
//    {
//        cv::Mat image=template_image_;
//        cv::Mat image_search=search_image;

//        cv::imwrite( "template_contains.jpg", image);
//        cv::imwrite( "search_contains.jpg", image_search);

//        // select a roi
//        cv::Mat roi(image, cv::Rect(bottom_right_.x-square_edge/2,bottom_right_.y-square_edge/2,square_edge,square_edge));
//        cv::Mat roi_search(image_search, cv::Rect(search_bottom_right_.x-template_image_.cols-square_edge/2,search_bottom_right_.y-square_edge/2,square_edge,square_edge));

//        cv::Size size(roi.cols*2,roi.cols);
//        cv::Mat image_both(size,CV_8UC3);
//        cv::Mat roi_one (image_four_,cv::Rect(0,0,roi.cols,roi.rows));
//        cv::Mat roi_two (image_four_,cv::Rect(roi.cols,0,roi_search.cols,roi_search.rows));
//        roi.copyTo(roi_one);
//        roi_search.copyTo(roi_two);
////        cv::imwrite( "contains.jpg", image_both);

//        ROS_DEBUG("contains");
//    }
//    else
//    {
//        cv::Scalar color_not_find(0,0,255);
//        cv::rectangle(img_matches,upper_left_,bottom_right_,color_not_find,-1);
//        cv::rectangle(img_matches,search_upper_left_,search_bottom_right_,color_not_find,-1);


//        cv::Mat image=template_image_;
//        cv::Mat image_search=search_image;
//        cv::imwrite( "template_doesnt_contain.jpg", image);
//        cv::imwrite( "search_doesnt_contain.jpg", image_search);

//        cv::Mat roi(image, cv::Rect(bottom_right_.x-square_edge/2,bottom_right_.y-square_edge/2,square_edge,square_edge));
//        cv::Mat roi_search(image_search, cv::Rect(search_bottom_right_.x-template_image_.cols-square_edge/2,search_bottom_right_.y-square_edge/2,square_edge,square_edge));

//        cv::Size size(roi.cols*2,roi.cols);
//        cv::Mat image_both(size,CV_8UC3);
//        cv::Mat roi_one (image_four_,cv::Rect(0,roi.rows,roi.cols,roi.rows));
//        cv::Mat roi_two (image_four_,cv::Rect(roi.cols,roi.rows,roi_search.cols,roi_search.rows));

//        roi.copyTo(roi_one);
//        roi_search.copyTo(roi_two);

////        cv::imwrite( "doesnt_contain.jpg", image_four_);

//        ROS_DEBUG("doesnt contain");

//    }
//    cv::imwrite( "image_four.jpg", image_four_);

    /*
        Testing for flickering matches - END
    */


    publisher_.publish (convertCVToSensorMsg (temp_img_matches));

}

void TemplateMatcher::publishTF(const Eigen::Matrix4f &transformation,const std::string &frame_id, const std::string &child_frame_id)
{
    Eigen::Matrix3f Rotation= transformation.block<3,3>(0,0);
    Eigen::Quaternion<float> quat_rot(Rotation);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0,0,0) );//transformation_result(0,3), transformation_result(1,3), transformation_result(2,3)) );
    transform.setRotation( tf::Quaternion(quat_rot.x(),quat_rot.y(),quat_rot.z(),quat_rot.w()));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}

void TemplateMatcher::drawOnImage(const int &inliers, const int &matches, const double &frequency, cv::Mat &image)
{

    std::stringstream image_text_matches,image_text_inliers,image_text_hz;
    image_text_matches << "Matches: " << matches;
    image_text_hz << "Hz: " << frequency;
    image_text_inliers << "Transform Inliers: " << inliers ;

    cv::putText(image, image_text_matches.str(), cvPoint(10,15),
                cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);
    cv::putText(image, image_text_inliers.str(), cvPoint(10,25),
                cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);
    cv::putText(image, image_text_hz.str(), cvPoint(10,35),
                cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);


}


void TemplateMatcher::imageCallback (const sensor_msgs::ImageConstPtr & msg)
{

    cv::Mat search_image = convertSensorMsgToCV (msg);
    cv::Mat img_matches;
    std::vector<cv::Point2f> template_points,search_points;

    matcher_.getMatches(template_image_, search_image, img_matches, template_points, search_points);
    publisher_.publish (convertCVToSensorMsg (img_matches));


}




