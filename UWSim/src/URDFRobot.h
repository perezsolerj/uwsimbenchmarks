#ifndef URDFROBOT_H_
#define URDFROBOT_H_

#include "SimulatorConfig.h"
#include "KinematicChain.h"
#include "ConfigXMLParser.h"

#include <osgOcean/OceanScene>
#include <osg/Switch>

#include <iostream>
#include <string.h>
#include <resource_retriever/retriever.h>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>

struct membuf : std::streambuf
{
    membuf(char* begin, char* end) {
        this->setg(begin, begin, end);
    }
};


class URDFRobot: public KinematicChain {

public:

	std::vector<osg::Vec3d> joint_axis;

	osg::Node * retrieveResource(std::string name);
        osg::Node * loadGeometry(boost::shared_ptr<Geometry> geom);

	URDFRobot(osgOcean::OceanScene *oscene,Vehicle vehicle);

	~URDFRobot();

protected:
	
	void updateJoints(std::vector<double> &q);
	void updateJoints(std::vector<double> &q, int startJoint, int numJoints);

};

#endif /* URDFROBOT_H_ */
