#include "template_matching/probabilisticmatcher.h"
#include "util.cpp"

const static std::string image_matches_topic = "/image_matches";



ProbabilisticMatcher::ProbabilisticMatcher(ros::NodeHandle &nh, ObjectsDatabase* objects_database):
    matcher_(), image_transport_(nh), objects_database_(objects_database), template_library_()
{

    cloud_subscriber_ = nh.subscribe("/camera/depth_registered/points", 1, &ProbabilisticMatcher::cloudCallback, this);
    publisher_ = image_transport_.advertise (image_matches_topic, 1);



}

void ProbabilisticMatcher::cloudCallback (const sensor_msgs::PointCloud2Ptr& cloud_msg)
{


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud_color_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg,*dense_cloud_color_ptr);
    cv::Mat query_image = template_library_.restoreCVMatFromPointCloud(dense_cloud_color_ptr);

    cv::Mat temp_img_matches;
    std::vector<cv::Point2f> temp_template_points, temp_search_points;
    std::vector<cv::DMatch> temp_matches;

    std::vector<cv::Point2f> training_image_template_points, training_image_search_points;
    std::vector<cv::DMatch> training_image_matches;

    for (uint database_i = 0; database_i < objects_database_->databaseObjects_.size(); database_i++)
    {

        matcher_.getDescriptorMatches(objects_database_->databaseObjects_[database_i].image_, query_image, objects_database_->databaseObjects_[database_i].database_feature_keypoints_, objects_database_->databaseObjects_[database_i].database_feature_descriptors_, temp_img_matches, temp_template_points, temp_search_points, temp_matches);

        if(database_i == 0)
        {
            training_image_matches = temp_matches;
            training_image_template_points = temp_template_points;
            training_image_search_points = temp_search_points;
        }
        else
        {
            training_image_template_points.insert(training_image_template_points.end(), temp_template_points.begin(), temp_template_points.end());
            training_image_search_points.insert(training_image_search_points.end(), temp_search_points.begin(), temp_search_points.end());
            training_image_matches.insert(training_image_matches.end(), temp_matches.begin(), temp_matches.end());

        }

    }
    std::vector<float> match_distances_per_image;
    for(std::vector<cv::DMatch>::iterator m_it = training_image_matches.begin(); m_it != training_image_matches.end(); m_it++)
    {
        match_distances_per_image.push_back(m_it->distance);
    }
//    match_distances.push_back(match_distances_per_image);


}

