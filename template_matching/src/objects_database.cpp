#include <template_matching/objects_database.h>
#include <numeric>
#include <fstream>
#include <highgui.h>
#include <cv.h>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

void ObjectsDatabase::addObject(const ObjectData &object)
{

}

void ObjectsDatabase::fit1DGaussianToData(const std::vector<float>& v, double& m, double& stdev)
{
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    m =  sum / v.size();

    double accum = 0.0;
    for(uint i = 0; i < v.size(); i++)
    {
        accum += (v[i] - m) * (v[i] - m);
    };

    stdev = sqrt(accum / (v.size()-1));
}


void ObjectsDatabase::trainDatabase()
{
    for (uint i = 0; i < databaseObjects_.size(); i++)
    {
        std::vector<std::vector<float> > match_distances;


        for (std::vector<ObjectData>::iterator t_it = trainingObjectsArranged_[i].begin(); t_it != trainingObjectsArranged_[i].end(); t_it++)
        {


            std::vector<cv::Point2f> training_image_template_points, training_image_search_points;
            std::vector<cv::DMatch> training_image_matches;

            for (uint database_i = 0; database_i < databaseObjects_.size(); database_i++)
            {
                cv::Mat temp_img_matches;
                std::vector<cv::Point2f> temp_template_points, temp_search_points;
                std::vector<cv::DMatch> temp_matches;

                //change such that it doesnt extract features
                matcher_.getDescriptorMatches(databaseObjects_[database_i].image_, t_it->image_, databaseObjects_[database_i].database_feature_keypoints_, databaseObjects_[database_i].database_feature_descriptors_, temp_img_matches, temp_template_points, temp_search_points, temp_matches);

                if(database_i == 0)
                {
                    training_image_matches = temp_matches;
                    training_image_template_points = temp_template_points;
                    training_image_search_points = temp_search_points;
                }
                else
                {
                    training_image_template_points.insert(training_image_template_points.end(), temp_template_points.begin(), temp_template_points.end());
                    training_image_search_points.insert(training_image_search_points.end(), temp_search_points.begin(), temp_search_points.end());
                    training_image_matches.insert(training_image_matches.end(), temp_matches.begin(), temp_matches.end());

                }


            }

//            std::cout<<"no of feature matches per training image : "<<training_image_matches.size()<<std::endl;


            //create the association between matches of the same features and store them in the vector of vector of doubles
            std::vector<float> match_distances_per_image;
            for(std::vector<cv::DMatch>::iterator m_it = training_image_matches.begin(); m_it != training_image_matches.end(); m_it++)
            {
                match_distances_per_image.push_back(m_it->distance);
            }
            match_distances.push_back(match_distances_per_image);
        }


        //rearrange the features and put them in the trainingMatches
        for(uint mpi_i = 0; mpi_i < match_distances[0].size(); mpi_i++)
        {
            std::vector<float> matches_per_feature;

            for(uint m_i = 0; m_i < match_distances.size(); m_i++)
            {
                matches_per_feature.push_back(match_distances[m_i][mpi_i]);
            }
            databaseObjects_[i].training_matches_.push_back(matches_per_feature);
        }

        for(uint f_i = 0; f_i < databaseObjects_[i].training_matches_.size(); f_i++)
        {
            double mean = 0;
            double stdev = 0;
            fit1DGaussianToData(databaseObjects_[i].training_matches_[f_i], mean, stdev);

            std::pair<double, double> gaussian;
            gaussian.first = mean;
            gaussian.second = stdev;
            databaseObjects_[i].matching_gaussians_.push_back(gaussian);
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

        ObjectData object(it->name_, static_cast<POSE>(it->pose_), it->image_, temp_keypoints, temp_descriptors);
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


        ObjectData trainingObject(it->name_, static_cast<POSE>((it->pose_)%(int)POSE_COUNT), it->image_, temp_keypoints, temp_descriptors);
        trainingObjects_.push_back(trainingObject);
    }

    //rearrange the training objects

    for(uint i = 0; i < databaseObjects_.size(); i++)
    {
        std::vector<ObjectData> objects_of_the_same_pair;
        trainingObjectsArranged_.push_back(objects_of_the_same_pair);
    }

    for(uint i = 0; i < databaseObjects_.size(); i++)
    {

        for(uint j = 0; j < trainingObjects_.size(); j++)
        {
            if(trainingObjects_[j].id_ == "used")
            {
                continue;
            }

            if(( trainingObjects_ [j].id_ == databaseObjects_ [i].id_ ) && ( trainingObjects_ [j].pose_ == databaseObjects_[i].pose_))
            {
                trainingObjectsArranged_[i].push_back(trainingObjects_ [j]);
                trainingObjects_[j].id_ = "used";
            }
        }

//        trainingObjectsArranged_[i % (databaseObjects_.size())].push_back(trainingObjects_[i]);
//        std::cout<< "position in arranged = " << i% (databaseObjects_.size()) <<std::endl;
//        std::cout<< "pushing object number: " << i <<std::endl;
    }

}

void ObjectsDatabase::printDatabases()
{
    ROS_INFO("DATABASE OBJECTS: ");
    for (std::vector<ObjectData>::iterator it = databaseObjects_.begin(); it != databaseObjects_.end(); it++)
    {
        ROS_INFO_STREAM("ID: "<<it->id_<<", Pose: "<<it->pose_);
        ROS_INFO_STREAM("No of keypoints: "<<it->database_feature_keypoints_.size()<<", No of descriptors: "<<it->database_feature_descriptors_.rows);
//        ROS_INFO_STREAM("No of features in training matches: "<<it->training_matches_.size());
//        ROS_INFO_STREAM("No of matches per feature in training matches: "<<it->training_matches_[0].size());
//        cv::imshow("database_object", it->image_);
//        cv::waitKey(0);


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
//            cv::imshow("object", it->image_);
//            cv::waitKey(0);
        }
    }

}

void ObjectsDatabase::loadModels(const std::string& file_name)
{

    std::string line;
    std::ifstream myfile (file_name.c_str());

    boost::char_separator<char> sep(", ");


    if (myfile.is_open())
    {
        ObjectData object;
        bool first_iter = true;

        std::vector < std::vector<float> > features_with_matches;

        while ( std::getline(myfile,line) )
        {
            if(isalpha( ( line.at(0) ) ))
            {

                if(!first_iter)
                {
                    object.training_matches_ = features_with_matches;

                    //load the descriptors
                    std::stringstream jpg_stream;
                    jpg_stream << object.id_ << "_" << object.pose_ <<".jpg";
                    cv::Mat temp = cv::imread(jpg_stream.str());
                    cv::Mat descriptors_image;
                    cv::cvtColor( temp, descriptors_image, CV_BGR2GRAY );

                    object.database_feature_descriptors_ = descriptors_image;


                    databaseObjects_.push_back(object);

                    //clean the object
                    object.training_matches_.clear();
                    features_with_matches.clear();

                }
                first_iter = false;

                boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
                std::string id = *(tokens.begin());
                std::string pose_str = *(++tokens.begin());
                int pose = atoi(pose_str.c_str());
                object.id_ = id;
                object.pose_ = static_cast<POSE>(pose);
//                std::cout<< "pose: " <<pose << "id: " <<id << std::endl;
            }
            else
            {
                //extract the matching errors
                std::vector<float> matching_per_feature;

                boost::tokenizer< boost::char_separator<char> > tokens(line, sep);
                BOOST_FOREACH (const std::string& t, tokens)
                {
                  float matching = boost::lexical_cast<float>(t);
                  matching_per_feature.push_back(matching);
                }
                features_with_matches.push_back(matching_per_feature);
            }
        }
        //after last iteration
        object.training_matches_ = features_with_matches;
        std::stringstream jpg_stream;
        jpg_stream << object.id_ << "_" << object.pose_ <<".jpg";
        cv::Mat temp = cv::imread(jpg_stream.str());
        cv::Mat descriptors_image;
        cv::cvtColor( temp, descriptors_image, CV_BGR2GRAY );

        object.database_feature_descriptors_ = descriptors_image;

        databaseObjects_.push_back(object);

        myfile.close();
    }
}



  /*
  * Saves all the databaseObjects_ into a file called file_name
  */
void ObjectsDatabase::saveModels(const std::string& file_name)
{
    std::ofstream myfile;
    myfile.open (file_name.c_str());


    //iterate through all database objects
    for (std::vector<ObjectData>::iterator it = databaseObjects_.begin(); it != databaseObjects_.end(); it++)
    {
        //get the name and the pose of the respective object and save it
        myfile << it->id_ << ", " << it->pose_ << "\n";

        for (uint i = 0; i < it->training_matches_.size(); i++)
        {
            for (uint m_i = 0; m_i < it->training_matches_[i].size(); m_i++)
            {
                myfile << it->training_matches_[i][m_i] << ", ";
            }
            myfile << "\n";
            myfile.flush();
        }
        std::stringstream jpg_stream;
        jpg_stream << it->id_ << "_" << it->pose_ <<".jpg";
        cv::imwrite(jpg_stream.str(), it->database_feature_descriptors_);

    }

    myfile.close();

}




