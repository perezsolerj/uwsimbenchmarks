#ifndef BENCHMARK_H_
#define BENCHMARK_H_

#include <list>
using namespace std;


#include "Measures.h"
#include "BenchmarkXMLParser.h"
#include "SceneUpdater.h"
#include "SceneBuilder.h"

class Benchmark{
private:
  Measures ** measures;
  int numMeasures;
  int * active; // 0 if measure was off last iteration,1 if it was on (Measures)

  Measures * createTimeMeasure(MeasureInfo measureInfo);
  Measures * createPositionErrorMeasure(MeasureInfo measureInfo,osg::Group * root);
  Measures * createDistanceMeasure(MeasureInfo measureInfo,osg::Group * root);
  Measures * createCollisionMeasure(MeasureInfo measureInfo, BulletPhysics * physics);
  Measures * createEuclideanNormMeasure(MeasureInfo measureInfo);

  std::string function;

  Trigger * createTrigger(TriggerInfo triggerInfo,osg::Group * root);
  Trigger * startOn,* stopOn;

  SceneUpdater * sceneUpdater;
  SceneUpdater * createSceneUpdater(SceneUpdaterInfo su, SceneBuilder * builder);

  int activeBenchmark;

  void stopMeasures();
  void updateMeasures();
  void reset();
  void printResults();

  std::list<std::vector<double> > results;
public:
  Benchmark(BenchmarkXMLParser * bench,SceneBuilder * builder,BulletPhysics * physics);
  Benchmark();
  void step();

};

#endif
