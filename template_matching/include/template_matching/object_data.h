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

    ObjectData();
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


//vector of features where each feature has a vector of distances for the training data
    std::vector <std::vector <float> > training_matches_;

//Gaussians for matching scores
    std::vector <std::pair<double, double> > matching_gaussians_;



};

#endif // OBJECT_DATA_H
