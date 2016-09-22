#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include <list>
using namespace std;


#include "Measures.h"
#include "BenchmarkXMLParser.h"
#include "SceneUpdater.h"
#include "uwsim/SceneBuilder.h"

class Benchmark{
private:
  Measures ** measures;
  int numMeasures;
  ros::WallTime time;
  int * active; // 0 if measure was off last iteration,1 if it was on (Measures)
  BenchmarkInfoToROSString * benchmarkInfo;
  BenchmarkResultToROSFloat32MultiArray * resultsPublisher;
  ros::WallTime lastResultsPublish;
  double publishRate;

  Measures * createTimeMeasure(MeasureInfo measureInfo);
  Measures * createPositionErrorMeasure(MeasureInfo measureInfo,osg::Group * root);
  Measures * createDistanceMeasure(MeasureInfo measureInfo,osg::Group * root);
  Measures * createCollisionMeasure(MeasureInfo measureInfo, BulletPhysics * physics);
  Measures * createEuclideanNormMeasure(MeasureInfo measureInfo, SceneBuilder * builder);
  Measures * createObjectCenteredOnCam(MeasureInfo measureInfo, SceneBuilder * builder);
  Measures * createReconstruction3D(MeasureInfo measureInfo, SceneBuilder * builder);
  Measures * createPathFollowing(MeasureInfo measureInfo, SceneBuilder * builder);

  std::string function;

  Trigger * createTrigger(TriggerInfo triggerInfo,osg::Group * root);
  Trigger * startOn,* stopOn;

  SceneUpdater * sceneUpdater;
  SceneUpdater * createSceneUpdater(SceneUpdaterInfo su, SceneBuilder * builder);

  int activeBenchmark;

  void stopMeasures();
  void updateMeasures();
  void reset(int suLevel);
  void printResults();

  std::list<std::vector<double> > results;
  std::vector<std::list<double> > logging;
  std::vector<std::list<double> > timeLogging;
  std::list<double> iterationStart;
  //ROSTopicToShapeShifter* asd;
public:
  Benchmark(BenchmarkXMLParser * bench,SceneBuilder * builder,BulletPhysics * physics);
  Benchmark();
  void step();

};

#endif
