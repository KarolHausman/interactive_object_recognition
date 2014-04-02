#include <template_matching/object_data.h>


ObjectData::ObjectData(const std::string& id, const POSE& pose, const cv::Mat& image, const std::vector <cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    id_ = id;
    image_ = image;
    database_feature_keypoints_ = keypoints;
    database_feature_descriptors_ = descriptors;
    pose_ = pose;
}


void ObjectData::setDatabaseFeatures(const std::vector <cv::KeyPoint>& keypoints, const cv::Mat& descriptors)
{
    database_feature_keypoints_ = keypoints;
    database_feature_descriptors_ = descriptors;
}


ObjectData::ObjectData()
{}
