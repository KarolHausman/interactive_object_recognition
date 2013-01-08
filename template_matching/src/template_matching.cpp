/*
 * template_matching.cpp
 *
 *  Created on: jan 8, 2013
 *      Author: Karol Hausman
 */


#include "feature_cv_waitinglist/feature_matching.h"

#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>


#include "util.cpp"

#include <string>

const static std::string template_filename ="/home/karol/Desktop/frame0000.jpg";
const static std::string subscribe_topic ="/camera/rgb/image_color";
const static std::string image_matches_topic = "/image_matches";


class TemplateMatcher
{
  FeatureMatching matcher_;
  cv::Mat template_image_;


  image_transport::Publisher publisher_;
  image_transport::Subscriber subscriber_;
  image_transport::ImageTransport image_transport_;

public:
  TemplateMatcher(ros::NodeHandle nh):
    matcher_(), image_transport_(nh)
  {
    template_image_ = cv::Mat (cvLoadImage (template_filename.c_str (), CV_LOAD_IMAGE_COLOR));
    subscriber_ = image_transport_.subscribe(subscribe_topic, 1, &TemplateMatcher::imageCallback, this);

    publisher_ = image_transport_.advertise (image_matches_topic, 1);
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

