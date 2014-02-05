#include "Benchmark.h"

#include "muParser/muParser.h" //Used to evaluate mathematical expresions on benchmark scores

Benchmark::Benchmark(){
numMeasures=0;
}

Benchmark::Benchmark(BenchmarkXMLParser *bench,SceneBuilder * builder,BulletPhysics * physics){

  function=bench->function;
  MeasureInfo measureInfo;
  numMeasures= bench->measures.size();
  measures = new Measures *[numMeasures];
  active = new int[numMeasures];
  logging.resize(numMeasures);
  timeLogging.resize(numMeasures);
  iterationStart.push_back(0);
  int i=0;
  while( bench->measures.size() >0){
    measureInfo=  bench->measures.front();
    
    if(measureInfo.type == MeasureInfo::Time)
      measures[i]=createTimeMeasure(measureInfo);
    else if(measureInfo.type == MeasureInfo::Collisions)
      measures[i]=createCollisionMeasure(measureInfo,physics);
    else if(measureInfo.type == MeasureInfo::PositionError)
      measures[i]=createPositionErrorMeasure(measureInfo,builder->root);
    else if(measureInfo.type == MeasureInfo::Distance)
      measures[i]=createDistanceMeasure(measureInfo,builder->root);
    else if(measureInfo.type == MeasureInfo::EuclideanNorm)
      measures[i]=createEuclideanNormMeasure(measureInfo,builder);
    else if(measureInfo.type == MeasureInfo::ObjectCenteredOnCam)
      measures[i]=createObjectCenteredOnCam(measureInfo,builder);

    measures[i]->setTriggers(createTrigger(measureInfo.startOn,builder->root),createTrigger(measureInfo.stopOn,builder->root));
    measures[i]->setName(measureInfo.name);
    measures[i]->setLog(measureInfo.log);
    active[i]=0;
    bench->measures.pop_front();
    i++;
  }

  startOn= createTrigger(bench->startOn,builder->root);
  stopOn= createTrigger(bench->stopOn,builder->root);
  sceneUpdater= createSceneUpdater(bench->sceneUpdater,builder);
  activeBenchmark=0;
  //asd = new  ROSTopicToShapeShifter("/dataNavigator");

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

SceneUpdater * Benchmark::createSceneUpdater(SceneUpdaterInfo su, SceneBuilder * builder){
  if(su.type==SceneUpdaterInfo::None)
    return new NullSceneUpdater();
  else if(su.type==SceneUpdaterInfo::SceneFogUpdater){
    //Get cameras fog
    std::vector<osg::Fog *>  camerasFog;
    for(unsigned int i=0; i<builder->iauvFile.size();i++){
      for (unsigned int j=0; j<builder->iauvFile[i]->getNumCams(); j++) {
        camerasFog.push_back((osg::Fog *) builder->iauvFile[i]->camview[j].textureCamera->getOrCreateStateSet()->getAttribute(osg::StateAttribute::FOG));
      }
     }
    return new SceneFogUpdater(su.initialFog, su.finalFog, su.step, su.interval,camerasFog,builder->scene);
  }

 /* else if(su.type==SceneUpdaterInfo::CurrentForceUpdater){
    SimulatedIAUV * vehicle=NULL;
    for(unsigned int i=0;i<builder->iauvFile.size();i++)
      if(builder->iauvFile[i]->name==su.target)
	vehicle=builder->iauvFile[i].get();
    if(!vehicle){
      std::cerr<<"Target "<<su.target<<" for current force scene updater NOT found"<<std::endl;
      exit(1);
    }
    if(!builder->current){
      std::cerr<<"Current must be defined in order to use CurrentForceUpdater"<<std::endl;
      exit(1);
    }    
    return new CurrentForceUpdater(su.initialCurrent, su.finalCurrent, su.step, su.interval,vehicle,builder->current);
  }*/

  else if(su.type==SceneUpdaterInfo::ArmMoveUpdater){
    SimulatedIAUV * vehicle=NULL;
    for(unsigned int i=0;i<builder->iauvFile.size();i++)
      if(builder->iauvFile[i]->name==su.target)
	vehicle=builder->iauvFile[i].get();
    if(!vehicle){
      std::cerr<<"Target "<<su.target<<" for arm move scene updater NOT found"<<std::endl;
      exit(1);
    }   
    return new ArmMoveUpdater(su.armPositions, su.step, su.interval, vehicle);
  }
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

Measures * Benchmark::createEuclideanNormMeasure(MeasureInfo measureInfo, SceneBuilder * builder){
  EuclideanNorm * EN;
  if(measureInfo.subtype==MeasureInfo::Constant){
    EN = new EuclideanNorm(new EuclideanNorm::ConstantGT(measureInfo.groundTruth),measureInfo.target,"");
  }
  else if(measureInfo.subtype==MeasureInfo::CornersFromCam || measureInfo.subtype==MeasureInfo::CentroidFromCam){ //Type requires look for camera and target (cornersfromcam or centroidfromcam)
    osg::Camera *  camera=NULL;
    for(unsigned int i=0; i<builder->iauvFile.size();i++){
      for (unsigned int j=0; j<builder->iauvFile[i]->getNumCams(); j++) {
        if(builder->iauvFile[i]->camview[j].name==measureInfo.camera)
          camera=builder->iauvFile[i]->camview[j].textureCamera;
      }
    }
    if(camera==NULL){
      std::cerr<<"Camera for measure "<<measureInfo.name<<" couldn't be found."<<std::endl;
      exit(1);
    }

    osg::Node* target=NULL;
    for(unsigned int i=0; i<builder->objects.size();i++)
      if(builder->objects[i]->getName()==measureInfo.object)
        target=builder->objects[i];

    if(target==NULL){
      std::cerr<<"Object target for measure "<<measureInfo.name<<" couldn't be found."<<std::endl;
      exit(1);
    }

    if(measureInfo.subtype==MeasureInfo::CornersFromCam)
      EN = new EuclideanNorm(new EuclideanNorm::ObjectCornersInCam(camera,target),measureInfo.target,measureInfo.publishOn);
    else //CentroidFromCam{
      EN = new EuclideanNorm(new EuclideanNorm::ObjectCentroidInCam(camera,target),measureInfo.target,measureInfo.publishOn);
  }
  else{  //RelativeLocation
    osg::Node * from=findRN(measureInfo.from,builder->root);
    osg::Node * to=findRN(measureInfo.to,builder->root);
    if(from==NULL){
      std::cerr<<"'from' target for measure "<<measureInfo.name<<" couldn't be found."<<std::endl;
      exit(1);
    }
    if(to==NULL){
      std::cerr<<"'to' target for measure "<<measureInfo.name<<" couldn't be found."<<std::endl;
      exit(1);
    }
    EN = new EuclideanNorm(new EuclideanNorm::RelativeLocation(from,to),measureInfo.target,measureInfo.publishOn);
  }
  return EN;
}

Measures * Benchmark::createObjectCenteredOnCam(MeasureInfo measureInfo, SceneBuilder * builder){

  osg::Camera *  camera=NULL;
  for(unsigned int i=0; i<builder->iauvFile.size();i++){
    for (unsigned int j=0; j<builder->iauvFile[i]->getNumCams(); j++) {
      if(builder->iauvFile[i]->camview[j].name==measureInfo.camera)
        camera=builder->iauvFile[i]->camview[j].textureCamera;
    }
  }
  if(camera==NULL){
    std::cerr<<"Camera for measure "<<measureInfo.name<<" couldn't be found."<<std::endl;
    exit(1);
  }

  osg::Node * first= findRN(measureInfo.target,builder->root);
  if(first==NULL){
    std::cerr<<"Can't find target "<<measureInfo.target<<" in measure "<<measureInfo.name<<" check benchmark's XML."<<std::endl;
    exit(1);
  }

  return new ObjectCenteredOnCam(camera,first);
}

void Benchmark::stopMeasures(){
  mu::Parser parser;
  std::vector<double> benchResult;
  benchResult.resize(numMeasures+3); //measures+reference+globalresult+time
  benchResult[0]=sceneUpdater->getReference();
  benchResult[numMeasures+2]=(ros::WallTime::now()-time).toSec()+iterationStart.back();
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
      if(measures[i]->log!=-1 && (ros::WallTime::now()-time).toSec()+iterationStart.back()-timeLogging[i].back()>measures[i]->log){ //saving log
        std::vector<double> results=measures[i]->getMeasureDetails();
	for(unsigned int j=0;j<results.size();j++){
          logging[i].push_back(results[j]);
	}
        timeLogging[i].push_back((ros::WallTime::now()-time).toSec()+iterationStart.back());
      }
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

  outdata.open((std::string(getenv("HOME")) + "/.uwsim/benchmarkOutput.dat").c_str());
  if(!outdata){
    std::cerr<<"Couldn't open benchmark output file."<<std::endl;
    exit(1);
  }
  outdata<<sceneUpdater->getName()<<"\t";
  for(int i=0;i<numMeasures;i++)
    outdata<<measures[i]->name<<"\t";
  outdata<<"TOTAL\tSimTime"<<std::endl;

  while(results.size()>0){
    benchResult=results.front();
    for(unsigned int i=0;i<benchResult.size();i++)
      outdata<<benchResult[i]<<"\t";
    outdata<<std::endl;
    results.pop_front();
  }
  outdata.close();
  
  //Logging
  for (int i=0;i<numMeasures;i++){
    if(measures[i]->log!=-1){
      outdata.open((std::string(getenv("HOME")) + "/.uwsim/benchmark-"+measures[i]->name+".data").c_str());
      if(!outdata){
        std::cerr<<"Couldn't open benchmark output file."<<std::endl;
        exit(1);
      }
      outdata<<sceneUpdater->getName()<<"\t";
      std::vector<std::string> namedetails=measures[i]->getNameDetails();
      for(unsigned int j=0;j<namedetails.size();j++)
	outdata<<namedetails[j]<<"\t";
      outdata<<std::endl;
      while(logging[i].size()>0){
	outdata<<timeLogging[i].front()<<"\t";
	timeLogging[i].pop_front();
        for(unsigned int j=0;j<namedetails.size();j++){ //We assume namedetails has the same length as measuredetails.
	  outdata<<logging[i].front()<<"\t";
	  logging[i].pop_front();
	}
        outdata<<std::endl;
      }
      outdata<<std::endl;
      outdata.close();
    }
  }
}

void Benchmark::step(){

  if(startOn->isOn() && !stopOn->isOn() && !sceneUpdater->finished()){
    if(activeBenchmark==0){ //Benchmark started this iteration
      sceneUpdater->start();
      time=ros::WallTime::now();
    }
    activeBenchmark=1;
    updateMeasures();
  }
  else if( activeBenchmark==1){ //Benchmark stopped this iteration
    stopMeasures();
    activeBenchmark=0;
    iterationStart.push_back((ros::WallTime::now()-time).toSec()+iterationStart.back());
    sceneUpdater->updateScene();
    if(sceneUpdater->finished()){
      printResults();
      ROS_INFO("Benchmark finished.");
    }
    else
      reset();
  }

  if(sceneUpdater->needsUpdate()){ //Scene Updater needs update
    stopMeasures(); 
    activeBenchmark=0;
    iterationStart.push_back((ros::WallTime::now()-time).toSec()+iterationStart.back());
    sceneUpdater->updateScene();
    if(sceneUpdater->finished()){
      printResults();
      ROS_INFO("Benchmark finished.");
    }
    else
      reset();
  }

}
