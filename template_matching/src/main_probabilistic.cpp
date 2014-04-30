
#include "ros/ros.h"
#include "template_matching/objects_database.h"
#include "template_matching/probabilisticmatcher.h"



int main (int argc, char** argv)
{
    ros::init (argc, argv, "template_matcher");

    ros::NodeHandle nh("~") ;

    ObjectsDatabase database;
    TemplateLibrary library;
    database.createDatabase(library);
    database.createTrainingDatabase(library);

    database.trainDatabase();
    database.printDatabases();
    database.saveModels("MODEL_SIFT_STANDARD.txt");

//    database.loadModels("MODEL_SIFT_STANDARD.txt");
//    database.printDatabases();


//    ProbabilisticMatcher matcher(nh, &database);

//    database.printDatabases();

    std::cout << "training ready" << std::endl;
    ros::Rate loop_rate (30);
    while (ros::ok())
    {
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}

