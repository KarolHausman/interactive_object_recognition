#ifndef OBJECTS_DATABASE_H
#define OBJECTS_DATABASE_H

#include <template_matching/object_data.h>
#include <template_library/template_library.h>
#include <feature_cv_waitinglist/feature_matching.h>



class ObjectsDatabase
{

public:
    ObjectsDatabase(){}
    virtual ~ObjectsDatabase(){}

    void addObject(const ObjectData& object);
    void createDatabase(TemplateLibrary& templateLibrary);
    void printDatabases();
    void createTrainingDatabase(TemplateLibrary& templateLibrary);
    void trainDatabase();
    void saveModels(const std::string& file_name);



private:
    void fit1DGaussianToData(const std::vector<float>& v, double& m, double& stdev);

    std::vector<ObjectData> databaseObjects_;
    std::vector<ObjectData> trainingObjects_;
    std::vector<std::vector<ObjectData> > trainingObjectsArranged_;


    FeatureMatching matcher_;



};


#endif // OBJECTS_DATABASE_H
