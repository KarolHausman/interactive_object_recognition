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
    void createDatabase(TemplateLibrary& template_library);
    void printDatabase();


private:
    std::vector<ObjectData> objects_;
    std::vector<Template> library_templates_;
    FeatureMatching matcher_;



};


#endif // OBJECTS_DATABASE_H
