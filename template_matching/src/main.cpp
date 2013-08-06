
#include "ros/ros.h"
#include "template_matching/template_matching.h"


int main (int argc, char** argv)
{
    ros::init (argc, argv, "template_matcher");

    ros::NodeHandle nh("~") ;
    TemplateMatcher matcher(nh);
//    cv::Mat image_contain=cv::imread( "/home/karol/ros_workspace/interactive_object_recognition/template_matching/bin/search_contains.jpg", 1 );
//    std::vector<cv::KeyPoint> keypoints_contain;
//    matcher.matcher_.detectFeatures(image_contain,keypoints_contain);
//    ROS_INFO_STREAM("number of keypoints contain: "<<keypoints_contain.size());


//    cv::Mat image_doesnt=cv::imread( "/home/karol/ros_workspace/interactive_object_recognition/template_matching/bin/search_doesnt_contain.jpg", 1 );
//    std::vector<cv::KeyPoint> keypoints_doesnt;
//    matcher.matcher_.detectFeatures(image_doesnt,keypoints_doesnt);
//    ROS_INFO_STREAM("number of keypoints doesnt contain: "<<keypoints_doesnt.size());

//    for (int i=0;i<keypoints_doesnt.size();i++)
//    {
//        ROS_INFO_STREAM("response: "<<keypoints_doesnt[i].response);
//        ROS_INFO_STREAM("octave: "<<keypoints_doesnt[i].octave);
//        ROS_INFO_STREAM("angle: "<<keypoints_doesnt[i].angle);

//        cv::KeyPoint p;
//        p.angle;

//    }

    ros::Rate loop_rate (30);
    while (ros::ok())
    {
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    matcher.printAllFramesBins();
}
