
#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <visualizer/Visualizer.h>
#include <pcl/io/pcd_io.h>


#include <dynamic_reconfigure/server.h>
#include "../../cfg/cpp/cloud_saver/SaverConfig.h"


/*
 * template_library.h
 *
 *  Created on: jan 21, 2013
 *      Author: Karol Hausman
 */



class CloudSaver
{
public:
    CloudSaver();
    void spinVisualizer();



private:
    void reconfigCallback (cloud_saver::SaverConfig&config, uint32_t level);
    void cloudCallback (const sensor_msgs::PointCloud2Ptr& cloud_msg);
//    void saveTemplates();
    void generateNames(const int &i,std::stringstream &ss_image,std::stringstream &ss_no_plane_image,std::stringstream &ss_cloud_rgb,std::stringstream &ss_cloud_inliers);


    ros::NodeHandle nh_;
    ros::Subscriber cloud_subscriber_;
    Visualizer visualizer_;
    dynamic_reconfigure::Server<cloud_saver::SaverConfig> reconfig_srv_;
    dynamic_reconfigure::Server<cloud_saver::SaverConfig>::CallbackType
    reconfig_callback_;
    std::string cloud_name_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr_;

    int cloud_number_;





};
