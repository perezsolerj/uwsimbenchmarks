
#include "SimulatedIAUV.h"

class Current {
private:
  double module, moduleVariation, modulePeriod, randomNoise;
  double direction[2],directionVariation[2], directionPeriod[2];
  ros::WallTime last;
public:

  Current(double module, double direction[], double moduleVariation, double modulePeriod, double directionVariation[], double directionPeriod[], double randomNoise);
  void applyCurrent(std::vector<boost::shared_ptr<SimulatedIAUV> > iauvFile);
};
