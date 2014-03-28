#ifndef OBJECT_DATA_H
#define OBJECT_DATA_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>



enum POSE
{
    Z_UP_0,
    Z_UP_180,    
    POSE_COUNT
};

class ObjectData
{
public:

    ObjectData(const std::string& id, const POSE& pose, const cv::Mat& image, const std::vector <cv::KeyPoint>& keypoints, cv::Mat& descriptors);
    virtual ~ObjectData(){}
    void train(){}
    void getProbability(){}
    void setDatabaseFeatures(const std::vector <cv::KeyPoint>& keypoints, const cv::Mat& descriptors);



//private:

    POSE pose_;
    std::string id_;
    cv::Mat image_;
    std::vector <cv::KeyPoint> database_feature_keypoints_;
    cv::Mat database_feature_descriptors_;

    std::vector <std::vector <cv::KeyPoint> > training_feature_keypoints_;
    std::vector <cv::Mat > training_feature_descriptors_;

    //TODO: add Gaussians for matching scores




};

#endif // OBJECT_DATA_H
