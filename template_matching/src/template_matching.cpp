/*
 * template_matching.cpp
 *
 *  Created on: jan 8, 2013
 *      Author: Karol Hausman
 */

#include "template_matching/template_matching.h"

#include "util.cpp"


const static std::string template_filename ="/home/karol/Desktop/frame0000.jpg";
const static std::string subscribe_topic ="/camera/rgb/image_color";
const static std::string image_matches_topic = "/image_matches";
const static std::string cloud_name="/home/karol/Desktop/template1.pcd";
const static double plane_detection_distance_threshold = 0.02;



TemplateMatcher::TemplateMatcher(ros::NodeHandle nh):
    matcher_(),
    ransac_transformer_(),
    image_transport_(nh),
    template_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
    current_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
    dense_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZLRegionF>),
//    dense_reconstructor_(),
    template_library_()
{
    first_one_=true;
    publish_time_=ros::Time::now();
//    template_image_ = cv::Mat (cvLoadImage (template_filename.c_str (), CV_LOAD_IMAGE_COLOR));
    pcl::io::loadPCDFile(cloud_name,*template_cloud_ptr_);
//    template_library_.generateTemplateData();
//    template_library_.loadTemplates();
//    template_image_ =template_library_.loadTemplates()[0].no_plane_image_;
    template_image_ =template_library_.loadTemplates()[0].image_;

//    pcl::copyPointCloud(*template_cloud_ptr_,*dense_cloud_ptr_);
//    dense_reconstructor_.reset(new DenseReconstruction<pcl::PointXYZLRegionF>(dense_cloud_ptr_));
//    pcl::PointIndices indices_reconstruct;
//    dense_reconstructor_->activeSegmentation(*dense_cloud_ptr_,0.01f,89,100,indices_reconstruct);
//    dense_reconstructor_.reset(new DenseReconstruction<pcl::PointXYZLRegionF>(dense_cloud_ptr_));
//    dense_reconstructor_->reconstructDenseModel(1);

//    template_image_ = restoreCVMatNoPlaneFromPointCloud(template_cloud_ptr_);

//    template_image_ = restoreCVMatFromPointCloud(template_cloud_ptr_);

    //    subscriber_ = image_transport_.subscribe(subscribe_topic, 1, &TemplateMatcher::imageCallback, this);
    cloud_subscriber_ = nh.subscribe("/camera/depth_registered/points", 1, &TemplateMatcher::cloudCallback, this);

    publisher_ = image_transport_.advertise (image_matches_topic, 1);
}


void TemplateMatcher::detectPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in_ptr, pcl::ModelCoefficients &coefficients,
                                pcl::PointIndices &inliers)
{
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(plane_detection_distance_threshold);
            seg.setInputCloud(cloud_in_ptr);
            seg.segment(inliers, coefficients);

}

cv::Mat TemplateMatcher::restoreCVMatFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr)
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

cv::Mat TemplateMatcher::restoreCVMatNoPlaneFromPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_ptr)
{
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    detectPlane(cloud_in_ptr,coefficients,inliers);
    cv::Mat restored_image = cv::Mat (cloud_in_ptr->height, cloud_in_ptr->width, CV_8UC3);

    for (uint i = 0; i < inliers.indices.size(); i++)
    {
        cloud_in_ptr->points[inliers.indices[i]].b = 0;
        cloud_in_ptr->points[inliers.indices[i]].g = 0;
        cloud_in_ptr->points[inliers.indices[i]].r = 0;
    }

    for (uint rows = 0; rows < cloud_in_ptr->height; rows++)
    {
        for (uint cols = 0; cols < cloud_in_ptr->width; ++cols)
        {

            {
                restored_image.at<cv::Vec3b> (rows, cols)[0] = cloud_in_ptr->at (cols, rows).b;
                restored_image.at<cv::Vec3b> (rows, cols)[1] = cloud_in_ptr->at (cols, rows).g;
                restored_image.at<cv::Vec3b> (rows, cols)[2] = cloud_in_ptr->at (cols, rows).r;
            }
        }
    }
    return restored_image;
}

bool cvPointEqualTo(cv::Point pointA,cv::Point pointB)
{
    return ((abs(pointA.x-pointB.x)<=1)&&(abs(pointA.y-pointB.y)<=1));
}

void TemplateMatcher::cloudCallback (const sensor_msgs::PointCloud2Ptr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud_color_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg,*dense_cloud_color_ptr);
    pcl::copyPointCloud(*dense_cloud_color_ptr, *current_cloud_ptr_);
    cv::Mat search_image = restoreCVMatFromPointCloud(dense_cloud_color_ptr);

    cv::Mat img_matches;
    std::vector<cv::Point2f> template_points,search_points;

//    template_image_ = restoreCVMatNoPlaneFromPointCloud(template_cloud_ptr_);
    matcher_.getMatches(template_image_, search_image, img_matches, template_points, search_points);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_color_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(uint i=0; i < template_points.size(); i++)
    {
        template_color_cloud_ptr->points.push_back(template_cloud_ptr_->at(template_points[i].x, template_points[i].y));
        search_cloud_ptr->points.push_back(current_cloud_ptr_->at(search_points[i].x, search_points[i].y));
    }
    pcl::copyPointCloud(*template_color_cloud_ptr,*template_cloud_ptr);

    Eigen::Matrix4f transformation_result=Eigen::Matrix4f::Identity();
    int inliers(0);
    if (search_cloud_ptr->points.size()>0){

        inliers = ransac_transformer_.ransacUmeyama(search_cloud_ptr,template_cloud_ptr,transformation_result);
        ROS_DEBUG_STREAM("transform: \n"<<transformation_result);

        publishTF(transformation_result,"start","estimate");

    }
    ros::Duration time_since_last_callback = ros::Time::now() - publish_time_;
    publish_time_ = ros::Time::now();

    drawOnImage(inliers, static_cast<int>(template_points.size()), (1/time_since_last_callback.toSec()), img_matches);
/*
    Testing for flickering matches -BEGIN
*/

    cv::Scalar color(255,0,0);
    if (first_one_)
    {
        upper_left_.x=template_points[0].x-4;
        upper_left_.y=template_points[0].y-4;
        bottom_right_.x=template_points[0].x;
        bottom_right_.y=template_points[0].y;

        search_upper_left_.x=search_points[0].x-4+template_image_.cols;
        search_upper_left_.y=search_points[0].y-4;
        search_bottom_right_.x=search_points[0].x+template_image_.cols;
        search_bottom_right_.y=search_points[0].y;

        first_one_=false;
    }
    cv::rectangle(img_matches,upper_left_,bottom_right_,color);
    cv::rectangle(img_matches,search_upper_left_,search_bottom_right_,color);


    int square_edge=32;
    if(std::find_if(template_points.begin(), template_points.end(),boost::bind(&cvPointEqualTo,bottom_right_,_1)) != template_points.end())
    {
        cv::Mat image=template_image_;
        cv::Mat image_search=search_image;

        // select a roi
        cv::Mat roi(image, cv::Rect(bottom_right_.x-square_edge/2,bottom_right_.y-square_edge/2,square_edge,square_edge));
        cv::Mat roi_search(image_search, cv::Rect(search_bottom_right_.x-template_image_.cols-square_edge/2,search_bottom_right_.y-square_edge/2,square_edge,square_edge));

        cv::Size size(roi.cols*2,roi.cols);
        cv::Mat image_both(size,CV_8UC3);
        cv::Mat roi_one (image_both,cv::Rect(0,0,roi.cols,roi.rows));
        cv::Mat roi_two (image_both,cv::Rect(roi.cols,0,roi_search.cols,roi_search.rows));

        roi.copyTo(roi_one);
        roi_search.copyTo(roi_two);
        cv::imwrite( "contains.jpg", image_both);

        ROS_DEBUG("contains");
    }
    else
    {
        cv::Scalar color_not_find(0,0,255);
        cv::rectangle(img_matches,upper_left_,bottom_right_,color_not_find,-1);
        cv::rectangle(img_matches,search_upper_left_,search_bottom_right_,color_not_find,-1);


        cv::Mat image=template_image_;
        cv::Mat image_search=search_image;

        cv::Mat roi(image, cv::Rect(bottom_right_.x-square_edge/2,bottom_right_.y-square_edge/2,square_edge,square_edge));
        cv::Mat roi_search(image_search, cv::Rect(search_bottom_right_.x-template_image_.cols-square_edge/2,search_bottom_right_.y-square_edge/2,square_edge,square_edge));

        cv::Size size(roi.cols*2,roi.cols);
        cv::Mat image_both(size,CV_8UC3);
        cv::Mat roi_one (image_both,cv::Rect(0,0,roi.cols,roi.rows));
        cv::Mat roi_two (image_both,cv::Rect(roi.cols,0,roi_search.cols,roi_search.rows));

        roi.copyTo(roi_one);
        roi_search.copyTo(roi_two);

        cv::imwrite( "doesnt_contain.jpg", image_both);

        ROS_DEBUG("doesnt contain");

    }

    /*
        Testing for flickering matches - END
    */


    publisher_.publish (convertCVToSensorMsg (img_matches));

}

void TemplateMatcher::publishTF(const Eigen::Matrix4f &transformation,const std::string &frame_id, const std::string &child_frame_id)
{
    Eigen::Matrix3f Rotation= transformation.block<3,3>(0,0);
    Eigen::Quaternion<float> quat_rot(Rotation);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0,0,0) );//transformation_result(0,3), transformation_result(1,3), transformation_result(2,3)) );
    transform.setRotation( tf::Quaternion(quat_rot.x(),quat_rot.y(),quat_rot.z(),quat_rot.w()));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}

void TemplateMatcher::drawOnImage(const int &inliers, const int &matches, const double &frequency, cv::Mat &image)
{

    std::stringstream image_text_matches,image_text_inliers,image_text_hz;
    image_text_matches << "Matches: " << matches;
    image_text_hz << "Hz: " << frequency;
    image_text_inliers << "Transform Inliers: " << inliers ;

    cv::putText(image, image_text_matches.str(), cvPoint(10,15),
                cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);
    cv::putText(image, image_text_inliers.str(), cvPoint(10,25),
                cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);
    cv::putText(image, image_text_hz.str(), cvPoint(10,35),
                cv::FONT_HERSHEY_PLAIN, 0.8, cvScalar(255,200,200), 1, CV_AA);


}


void TemplateMatcher::imageCallback (const sensor_msgs::ImageConstPtr & msg)
{

    cv::Mat search_image = convertSensorMsgToCV (msg);
    cv::Mat img_matches;
    std::vector<cv::Point2f> template_points,search_points;

    matcher_.getMatches(template_image_, search_image, img_matches, template_points, search_points);
    publisher_.publish (convertCVToSensorMsg (img_matches));


}




