
#include <ransac_waitinglist/ransac_transformation.h>
#include <feature_cv_waitinglist/feature_matching.h>
#include <dense_reconstruction/DenseReconstruction.h>

#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
/*
 * template_matching.h
 *
 *  Created on: jan 8, 2013
 *      Author: Karol Hausman
 */

#include <tf/transform_broadcaster.h>

#include <string>

class TemplateMatcher
{
public:
    TemplateMatcher(ros::NodeHandle nh);

private:
    cv::Mat restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr);

    cv::Mat restoreCVMatNoPlaneFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr);

    void cloudCallback (const sensor_msgs::PointCloud2Ptr& cloud_msg);

    void imageCallback (const sensor_msgs::ImageConstPtr & msg);

    void publishTF(const Eigen::Matrix4f &transformation,const std::string &frame_id, const std::string &child_frame_id);

    void drawOnImage(const int &inliers, const int &matches, const double &frequency, cv::Mat &image);

    void detectPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in_ptr, pcl::ModelCoefficients &coefficients,
                                    pcl::PointIndices &inliers);

    FeatureMatching matcher_;
    RANSACTransformation ransac_transformer_;

    image_transport::ImageTransport image_transport_;
    image_transport::Publisher publisher_;
    image_transport::Subscriber subscriber_;
    ros::Subscriber cloud_subscriber_;

    ros::Time publish_time_;
    cv::Mat template_image_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr dense_cloud_ptr_;
    boost::shared_ptr<DenseReconstruction<pcl::PointXYZLRegionF> > dense_reconstructor_;

};


