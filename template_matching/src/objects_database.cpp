#include <template_matching/objects_database.h>

void ObjectsDatabase::addObject(const ObjectData &object)
{

}


void ObjectsDatabase::createDatabase(TemplateLibrary& templateLibrary)
{
    std::vector<Template> library_templates = templateLibrary.loadTemplates();

    for (std::vector<Template>::iterator it=library_templates.begin(); it!=library_templates.end(); it++)
    {

        std::vector<cv::KeyPoint> temp_keypoints;
        cv::Mat temp_descriptors;
        matcher_.getFeatures(it->image_, temp_keypoints, temp_descriptors);

        ObjectData object(it->name_, static_cast<POSE>(it->pose_ - 1), it->image_, temp_keypoints, temp_descriptors);
        objects_.push_back(object);

    }

}

void ObjectsDatabase::createTrainingDatabase(TemplateLibrary& templateLibrary)
{
    std::vector<Template> training_templates = templateLibrary.loadTemplates("training");

    for (std::vector<Template>::iterator it=training_templates.begin(); it!=training_templates.end(); it++)
    {

        std::vector<cv::KeyPoint> temp_keypoints;
        cv::Mat temp_descriptors;
        matcher_.getFeatures(it->image_, temp_keypoints, temp_descriptors);


        ObjectData trainingObject(it->name_, static_cast<POSE>((it->pose_ - 1)%(int)POSE_COUNT), it->image_, temp_keypoints, temp_descriptors);
        trainingObjects_.push_back(trainingObject);
    }

    //rearrange the training objects


}

void ObjectsDatabase::printDatabases()
{
    ROS_INFO("DATABASE OBJECTS: ");
    for (std::vector<ObjectData>::iterator it = objects_.begin(); it != objects_.end(); it++)
    {
        ROS_INFO_STREAM("ID: "<<it->id_<<", Pose: "<<it->pose_);
        ROS_INFO_STREAM("No of keypoints: "<<it->database_feature_keypoints_.size()<<", No of descriptors: "<<it->database_feature_descriptors_.rows);

    }

    ROS_INFO("TRAINING DATABASE OBJECTS: ");
    for (std::vector<ObjectData>::iterator it = trainingObjects_.begin(); it != trainingObjects_.end(); it++)
    {
        ROS_INFO_STREAM("ID: "<<it->id_<<", Pose: "<<it->pose_);
        ROS_INFO_STREAM("No of keypoints: "<<it->database_feature_keypoints_.size()<<", No of descriptors: "<<it->database_feature_descriptors_.rows);

    }
}


