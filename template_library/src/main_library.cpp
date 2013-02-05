/*
 * main_library.cpp
 *
 *  Created on: Sep 28, 2012
 *      Author: Karol Hausman
 */

#include <ros/ros.h>
#include "template_library/template_library.h"

int main(int argc, char **argv)
{

        ros::init(argc, argv, "library");

        TemplateLibrary library;

        library.generateTemplateData();

}

