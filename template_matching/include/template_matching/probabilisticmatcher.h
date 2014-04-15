#ifndef PROBABILISTICMATCHER_H
#define PROBABILISTICMATCHER_H
#include <pcl/io/pcd_io.h>
#include <feature_cv_waitinglist/feature_matching.h>
#include <template_library/template_library.h>
#include <image_transport/image_transport.h>
#include <template_matching/objects_database.h>



class ProbabilisticMatcher
{
public:
    ProbabilisticMatcher(ros::NodeHandle &nh, ObjectsDatabase* objects_database);

    FeatureMatching matcher_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher publisher_;
    ros::Subscriber cloud_subscriber_;
    ObjectsDatabase* objects_database_;
    TemplateLibrary template_library_;
    std::vector<float> matches_per_image_;



    void cloudCallback (const sensor_msgs::PointCloud2Ptr& cloud_msg);

};

#endif // PROBABILISTICMATCHER_H
