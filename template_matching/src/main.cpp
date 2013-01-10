
#include "ros/ros.h"
#include "template_matching/template_matching.h"


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
