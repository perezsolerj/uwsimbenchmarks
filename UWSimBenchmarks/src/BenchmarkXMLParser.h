#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#ifndef BENCHMARKXMLPARSER_H_
#define BENCHMARKXMLPARSER_H_

#include <libxml++/libxml++.h>
#include <boost/shared_ptr.hpp>

#include <iostream>
using namespace std;
#include <cstdlib>
#include <list>
#include <osg/Matrixd>

#include "SimulatorConfig.h"

struct TriggerInfo{
  typedef enum { Topic, AlwaysOn, AlwaysOff, Service, OnMove, OnNoMove, Position } TriggerType;
  TriggerType type;
  string target;
  double position[3]; //needed for PositionTrigger
};

struct MeasureInfo{ 
  typedef enum { Unknown, Time, Collisions, PositionError, Distance, EuclideanNorm , ObjectCenteredOnCam} type_t;
  typedef enum { Constant, CornersFromCam, CentroidFromCam } type_s;
  type_t type;
  type_s subtype; //Subtype for euclideanNorm
  string name,target;
  double log;
  string camera,object,publishOn; //Needed for EuclideanNorm
  double position[3]; //needed for PositionError
  std::vector<double>  groundTruth; //Used in euclideanNorm
  TriggerInfo startOn, stopOn;
};

struct SceneUpdaterInfo{
  typedef enum { None, SceneFogUpdater, CurrentForceUpdater} SceneUpdaterType;
  SceneUpdaterType type;
  double initialFog, finalFog, step, interval; //needed for SceneFogUpdater
  double initialCurrent, finalCurrent;
  std::string target;
};

class BenchmarkXMLParser{
  private:
    void esPi(string in,double * param);

    void extractFloatChar(const xmlpp::Node* node,double * param);
    void extractIntChar(const xmlpp::Node* node,int * param);
    void extractUIntChar(const xmlpp::Node* node, unsigned int * param);
    void extractStringChar(const xmlpp::Node* node,string * param);
    void extractPositionOrColor(const xmlpp::Node* node,double * param);

    void processXML(const xmlpp::Node* node);
    void processVector(const xmlpp::Node* node, std::vector<double> &groundTruth);
    void processMeasure(const xmlpp::Node* node,MeasureInfo * measure);
    void processMeasures(const xmlpp::Node* node);
    void processTrigger(const xmlpp::Node* node,TriggerInfo * trigger);
    void processSceneUpdater(const xmlpp::Node* node,SceneUpdaterInfo * su);
    void processSceneUpdaters(const xmlpp::Node* node,SceneUpdaterInfo * su);
    void processGTFromCam(const xmlpp::Node* node,MeasureInfo * measure);

    void applyOffset(double position[3],double offsetp[3],double offsetr[3]);
  public:
    list <MeasureInfo> measures;
    TriggerInfo startOn, stopOn;
    string function;
    SceneUpdaterInfo sceneUpdater;
    BenchmarkXMLParser(const std::string &fName);
    BenchmarkXMLParser(); //IF no benchmark defined

    void postProcessWorldOffset(double offsetp[3],double offsetr[3]);
};


#endif
