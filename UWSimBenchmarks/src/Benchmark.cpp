#include "Benchmark.h"

#include "muParser/muParser.h"  //Used to evaluate mathematical expresions on benchmark scores

Benchmark::Benchmark(){
numMeasures=0;
}

Benchmark::Benchmark(BenchmarkXMLParser *bench,osg::Group*  root,BulletPhysics * physics,std::vector<osg::Fog *>  camerasFog, osg::ref_ptr<osgOceanScene> scene){

  function=bench->function;
  MeasureInfo measureInfo;
  numMeasures= bench->measures.size();
  measures = new Measures *[numMeasures];
  active = new int[numMeasures];
  int i=0;
  while( bench->measures.size() >0){
    measureInfo=  bench->measures.front();
    
    if(measureInfo.type == MeasureInfo::Time)
      measures[i]=createTimeMeasure(measureInfo);
    else if(measureInfo.type == MeasureInfo::Collisions)
      measures[i]=createCollisionMeasure(measureInfo,physics);
    else if(measureInfo.type == MeasureInfo::PositionError)
      measures[i]=createPositionErrorMeasure(measureInfo,root);
    else if(measureInfo.type == MeasureInfo::Distance)
      measures[i]=createDistanceMeasure(measureInfo,root);
    else if(measureInfo.type == MeasureInfo::EuclideanNorm)
      measures[i]=createEuclideanNormMeasure(measureInfo);

    measures[i]->setTriggers(createTrigger(measureInfo.startOn,root),createTrigger(measureInfo.stopOn,root));
    measures[i]->setName(measureInfo.name);
    active[i]=0;
    bench->measures.pop_front();
    i++;
  }

  startOn= createTrigger(bench->startOn,root);
  stopOn= createTrigger(bench->stopOn,root);
  sceneUpdater= createSceneUpdater(bench->sceneUpdater,camerasFog,scene);
  activeBenchmark=0;
}

Trigger * Benchmark::createTrigger(TriggerInfo triggerInfo,osg::Group * root){
  if(triggerInfo.type == TriggerInfo::Topic)
    return new TopicTrigger(triggerInfo.target);
  else if(triggerInfo.type == TriggerInfo::AlwaysOn)
    return new AlwaysOnTrigger();
  else if(triggerInfo.type == TriggerInfo::AlwaysOff)
    return new AlwaysOffTrigger();
  else if(triggerInfo.type == TriggerInfo::Service)
    return new ServiceTrigger(triggerInfo.target);
  else if(triggerInfo.type == TriggerInfo::OnMove){
    osg::Node * first= findRN(triggerInfo.target,root);
    if(first==NULL){
      std::cerr<<"Can't find target "<<triggerInfo.target<<" in MoveTrigger check benchmark's XML, never trigger used."<<std::endl;
      return new AlwaysOffTrigger();
    }
    else
      return new MoveTrigger(first);
  }
  else if(triggerInfo.type == TriggerInfo::OnNoMove){
    osg::Node * first= findRN(triggerInfo.target,root);
    if(first==NULL){
      std::cerr<<"Can't find target "<<triggerInfo.target<<" in NoMoveTrigger check benchmark's XML, never trigger used."<<std::endl;
      return new AlwaysOffTrigger();
    }
    else
      return new NoMoveTrigger(first);
  }
  else if(triggerInfo.type == TriggerInfo::Position){
    osg::Node * first= findRN(triggerInfo.target,root);
    if(first==NULL){
      std::cerr<<"Can't find target "<<triggerInfo.target<<" in PositionTrigger check benchmark's XML, never trigger used."<<std::endl;
      return new AlwaysOffTrigger();
    }
    else
      return new PositionTrigger(first,triggerInfo.position);
  }
  else{
    std::cerr<<"Unknown trigger"<<std::endl;
    exit(1);
  }
}

SceneUpdater * Benchmark::createSceneUpdater(SceneUpdaterInfo su,std::vector<osg::Fog *>  camerasFog, osg::ref_ptr<osgOceanScene> scene){
  if(su.type==SceneUpdaterInfo::None)
    return new NullSceneUpdater();
  else if(su.type==SceneUpdaterInfo::SceneFogUpdater)
    return new SceneFogUpdater(su.initialFog, su.finalFog, su.step, su.interval,camerasFog,scene);
  else{
    std::cerr<<"Unknown scene updater"<<std::endl;
    exit(1);  
  }
}

Measures * Benchmark::createTimeMeasure(MeasureInfo measureInfo){
  Time * timer = new Time();
  return timer;
}

Measures * Benchmark::createPositionErrorMeasure(MeasureInfo measureInfo,osg::Group * root){
  osg::Node * first= findRN(measureInfo.target,root);
  if(first==NULL){
    std::cerr<<"Can't find target "<<measureInfo.target<<" in PositionError measure check benchmark's XML."<<std::endl;
    exit(1);
  }
  else
    return new PositionError(first,measureInfo.position);
}

Measures * Benchmark::createDistanceMeasure(MeasureInfo measureInfo,osg::Group * root){
  osg::Node * first= findRN(measureInfo.target,root);
  if(first==NULL){
    std::cerr<<"Can't find target "<<measureInfo.target<<" in distance measure check benchmark's XML."<<std::endl;
    exit(1);
  }
  else
    return new Distance(first);
}

Measures * Benchmark::createCollisionMeasure(MeasureInfo measureInfo,BulletPhysics * physics){
  Collisions * coll = new Collisions(physics,measureInfo.target);
  return coll;
}

Measures * Benchmark::createEuclideanNormMeasure(MeasureInfo measureInfo){
  EuclideanNorm * EN = new EuclideanNorm(measureInfo.groundTruth, measureInfo.nVals,measureInfo.target);
  return EN;
}

void Benchmark::stopMeasures(){
  mu::Parser parser;
  std::vector<double> benchResult;
  benchResult.resize(numMeasures+2);
  benchResult[0]=sceneUpdater->getReference();
  int error=0;

  //std::cout<<"Num measures: "<<numMeasures<<std::endl;
  for(int i=0; i<numMeasures ;i++){
    if(measures[i]->isOn() || active[i]==1)
      measures[i]->stop();
    active[i]=0;
    benchResult[i+1]=measures[i]->getMeasure();
    parser.DefineConst(measures[i]->name, benchResult[i+1]);
    error+=measures[i]->error();
    //std::cout<<"MEASURE "<<" "<<measures[i]->name<<" "<<measures[i]->getMeasure()<<" "<<measures[i]->isOn()<<std::endl;  
  }


  if(error==0){
  //Print result??
    parser.SetExpr(function);
    try{
      benchResult[numMeasures+1]=parser.Eval();
      //std::cout<<"Benchmark stopped, result: "<<parser.Eval()<<std::endl;
    }
    catch (mu::Parser::exception_type &e)
    {
      std::cerr<<"Error on benchmark function: "<< e.GetMsg() << std::endl;
    }
    results.push_back(benchResult);
  }
  else
    std::cout<<"ERROR on measure"<<std::endl;
}

void Benchmark::updateMeasures(){
    for(int i=0; i<numMeasures ;i++){
      if(measures[i]->isOn()){
        if(active[i]==0) // It has started this iteration
          measures[i]->start();
        else 
 	  measures[i]->update();
      //Cout for debugging!
      //std::cout<<"MEASURE "<<measures[i]->getMeasure()<<" "<<measures[i]->isOn()<<" "<<active[i]<<std::endl;
      }
      else
        if(active[i]==1) //It has stopped this iteration
	  measures[i]->stop();
      active[i]=measures[i]->isOn();
    }

}

void Benchmark::reset(){
  for(int i=0; i<numMeasures ;i++)
    measures[i]->reset();
  //startOn->reset();
  //stopOn->reset();
}

void Benchmark::printResults(){
  std::vector<double> benchResult;
  std::ofstream outdata;

  outdata.open("benchmarkOutput.dat");
  if(!outdata){
    std::cerr<<"Couldn't open benchmark output file."<<std::endl;
    exit(1);
  }
  outdata<<sceneUpdater->getName()<<"\t";
  for(int i=0;i<numMeasures;i++)
    outdata<<measures[i]->name<<"\t";
  outdata<<"TOTAL"<<std::endl;

  while(results.size()>0){
    benchResult=results.front();
    for(unsigned int i=0;i<benchResult.size();i++)
      outdata<<benchResult[i]<<"\t";
    outdata<<std::endl;
    results.pop_front();
  }
}

void Benchmark::step(){

  if(startOn->isOn() && !stopOn->isOn() && !sceneUpdater->finished()){
    if(activeBenchmark==0) //Benchmark started this iteration
      sceneUpdater->start();
    activeBenchmark=1;
    updateMeasures();
  }
  else if( activeBenchmark==1){
    stopMeasures();
    activeBenchmark=0;
    sceneUpdater->updateScene();
    if(sceneUpdater->finished())
      printResults();
    else
      reset();
  }

  if(sceneUpdater->needsUpdate()){
    stopMeasures();
    activeBenchmark=0;
    sceneUpdater->updateScene();
    if(sceneUpdater->finished())
      printResults();
    else
      reset();
  }

}
