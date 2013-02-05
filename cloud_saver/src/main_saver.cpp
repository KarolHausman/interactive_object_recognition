/*
 * main_library.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: Karol Hausman
 */

#include <ros/ros.h>
#include "cloud_saver/cloud_saver.h"

int main(int argc, char **argv)
{

        ros::init(argc, argv, "cloud_saver");

        CloudSaver saver;

//        library.generateTemplateData();
        ros::Rate loop_rate (30);
        while (ros::ok())
        {
            saver.spinVisualizer();
            ros::spinOnce ();
            loop_rate.sleep ();
        }
}

