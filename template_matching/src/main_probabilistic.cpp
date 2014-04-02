
#include "ros/ros.h"
#include "template_matching/objects_database.h"



int main (int argc, char** argv)
{
    ros::init (argc, argv, "template_matcher");

    ros::NodeHandle nh("~") ;

    ObjectsDatabase database;
//    TemplateLibrary library;
//    database.createDatabase(library);
//    database.createTrainingDatabase(library);


//    database.trainDatabase();
//    database.printDatabases();
//    database.saveModels("models3.txt");

    database.loadModels("models3.txt");
    database.printDatabases();

    ros::Rate loop_rate (30);
    while (ros::ok())
    {
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}

