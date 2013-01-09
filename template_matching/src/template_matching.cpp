/*
 * template_matching.cpp
 *
 *  Created on: jan 8, 2013
 *      Author: Karol Hausman
 */


#include <ransac_waitinglist/ransac_transformation.h>
#include <feature_cv_waitinglist/feature_matching.h>

#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>

#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>

#include "util.cpp"

#include <string>

const static std::string template_filename ="/home/karol/Desktop/frame0000.jpg";
const static std::string subscribe_topic ="/camera/rgb/image_color";
const static std::string image_matches_topic = "/image_matches";
const static std::string cloud_name="/home/karol/Desktop/template1.pcd";


class TemplateMatcher
{
  FeatureMatching matcher_;
  RANSACTransformation ransac_transformer_;


  image_transport::ImageTransport image_transport_;
  image_transport::Publisher publisher_;
  image_transport::Subscriber subscriber_;
  ros::Subscriber cloud_subscriber_;

  cv::Mat template_image_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr_;

public:
  TemplateMatcher(ros::NodeHandle nh):
    matcher_(),
    ransac_transformer_(),
    image_transport_(nh),
    template_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
    current_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    template_image_ = cv::Mat (cvLoadImage (template_filename.c_str (), CV_LOAD_IMAGE_COLOR));
    pcl::io::loadPCDFile(cloud_name,*template_cloud_ptr_);
    subscriber_ = image_transport_.subscribe(subscribe_topic, 1, &TemplateMatcher::imageCallback, this);
    cloud_subscriber_ = nh.subscribe("/camera/depth_registered/points", 1, &TemplateMatcher::cloudCallback, this);

    publisher_ = image_transport_.advertise (image_matches_topic, 1);
  }


  cv::Mat restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr)
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


  void cloudCallback (const sensor_msgs::PointCloud2Ptr& cloud_msg)
  {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud_color_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg,*dense_cloud_color_ptr);
      pcl::copyPointCloud(*dense_cloud_color_ptr, *current_cloud_ptr_);
      cv::Mat search_image = restoreCVMatFromPointCloud(dense_cloud_color_ptr);

      cv::Mat img_matches;
      std::vector<cv::Point2f> template_points,search_points;

      template_image_ = restoreCVMatFromPointCloud(template_cloud_ptr_);
      matcher_.getMatches(template_image_, search_image, img_matches, template_points, search_points);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_color_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

      for(uint i=0; i < template_points.size(); i++)
      {
          template_color_cloud_ptr->points.push_back(template_cloud_ptr_->at(template_points[i].x, template_points[i].y));
//          template_cloud_ptr->points.push_back(current_cloud_ptr_->at(search_points[i].x, search_points[i].y));
          search_cloud_ptr->points.push_back(current_cloud_ptr_->at(search_points[i].x, search_points[i].y));
          pcl::copyPointCloud(*template_color_cloud_ptr,*template_cloud_ptr);
      }

      Eigen::Matrix4f transformation_result=Eigen::Matrix4f::Identity();
      int inliers(0);
      if (search_cloud_ptr->points.size()>0){

          inliers = ransac_transformer_.ransacUmeyama(search_cloud_ptr,template_cloud_ptr,transformation_result);
          ROS_INFO_STREAM("transform: \n"<<transformation_result);
      }

      std::stringstream image_text_matches,image_text_inliers;
      image_text_inliers << "Transform Inliers: " << inliers ;
      image_text_matches << "Matches: " << template_points.size();
      cv::putText(img_matches, image_text_inliers.str(), cvPoint(10,15),
                  cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);
      cv::putText(img_matches, image_text_matches.str(), cvPoint(10,25),
                  cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);
      publisher_.publish (convertCVToSensorMsg (img_matches));

  }

  void imageCallback (const sensor_msgs::ImageConstPtr & msg)
  {

      cv::Mat search_image = convertSensorMsgToCV (msg);
      cv::Mat img_matches;
      std::vector<cv::Point2f> template_points,search_points;

      matcher_.getMatches(template_image_, search_image, img_matches, template_points, search_points);
      publisher_.publish (convertCVToSensorMsg (img_matches));


  }

};
int main (int argc, char** argv)
{
  ros::init (argc, argv, "template_matcher");

  ros::NodeHandle nh("~") ;
  TemplateMatcher matcher(nh);

  ros::Rate loop_rate (30);
  while (ros::ok())
  {
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}

