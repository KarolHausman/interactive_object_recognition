#include <template_matching/objects_database.h>

void ObjectsDatabase::addObject(const ObjectData &object)
{

}

void ObjectsDatabase::trainDatabase()
{
    for (uint i = 0; i < databaseObjects_.size(); i++)
    {

        for (std::vector<ObjectData>::iterator t_it = trainingObjectsArranged_[i].begin(); t_it != trainingObjectsArranged_[i].end(); t_it++)
        {
            cv::Mat temp_img_matches;
            std::vector<cv::Point2f> temp_template_points, temp_search_points;
            std::vector<cv::DMatch> temp_matches;

            //change such that it doesnt extract features
//            matcher_.getDescriptorMatches(databaseObjects_[i].image_, t_it->image_, databaseObjects_[i].database_feature_keypoints_, databaseObjects_[i].database_feature_descriptors_, temp_img_matches, temp_template_points, temp_search_points, temp_matches);

            //create the association between matches of the same features and store them in the vector of vector of doubles
//            std::vector<double> match_distances;
//            databaseObjects_[i].trainingMatches_.push_back(match_distances);
//            match_distances.push_back();
//            databaseObjects_[i].trainingMatches_.push_back(temp_matches[i][0].distance);
        }

    }
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
        databaseObjects_.push_back(object);

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

    for(uint i = 0; i < databaseObjects_.size(); i++)
    {
        std::vector<ObjectData> objects_of_the_same_pair;
        trainingObjectsArranged_.push_back(objects_of_the_same_pair);
    }

    for(uint i = 0; i < trainingObjects_.size(); i++)
    {
        trainingObjectsArranged_[i%databaseObjects_.size()].push_back(trainingObjects_[i]);
    }

}

void ObjectsDatabase::printDatabases()
{
    ROS_INFO("DATABASE OBJECTS: ");
    for (std::vector<ObjectData>::iterator it = databaseObjects_.begin(); it != databaseObjects_.end(); it++)
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

    ROS_INFO("ARRANGED TRAINING DATABASE OBJECTS: ");
    for (uint i = 0; i < trainingObjectsArranged_.size(); i++)
    {
        ROS_INFO_STREAM("Object - Pose pair no: " << i);
        for (std::vector<ObjectData>::iterator it = trainingObjectsArranged_[i].begin(); it != trainingObjectsArranged_[i].end(); it++)
        {
            ROS_INFO_STREAM("ID: "<<it->id_<<", Pose: "<<it->pose_);
            ROS_INFO_STREAM("No of keypoints: "<<it->database_feature_keypoints_.size()<<", No of descriptors: "<<it->database_feature_descriptors_.rows);
        }
    }

}


