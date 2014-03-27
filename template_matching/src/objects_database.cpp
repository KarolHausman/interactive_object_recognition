#include <template_matching/objects_database.h>

void ObjectsDatabase::addObject(const ObjectData &object)
{

}


void ObjectsDatabase::createDatabase(TemplateLibrary& template_library)
{
    library_templates_ = template_library.loadTemplates();

    for (std::vector<Template>::iterator it=library_templates_.begin(); it!=library_templates_.end(); it++)
    {

        std::vector<cv::KeyPoint> temp_keypoints;
        cv::Mat temp_descriptors;
        matcher_.getFeatures(it->image_, temp_keypoints, temp_descriptors);


        ObjectData object(it->name_, static_cast<POSE>(it->pose_), it->image_, temp_keypoints, temp_descriptors);
        objects_.push_back(object);
    }

}

void ObjectsDatabase::printDatabase()
{
    ROS_INFO("DATABASE OBJECTS: ");
    for (std::vector<ObjectData>::iterator it = objects_.begin(); it != objects_.end(); it++)
    {
        ROS_INFO_STREAM("ID: "<<it->id_<<", Pose: "<<it->pose_);
        ROS_INFO_STREAM("No of keypoints: "<<it->database_feature_keypoints_.size()<<", No of descriptors: "<<it->database_feature_descriptors_.rows);

    }
}


