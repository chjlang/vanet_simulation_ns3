#ifndef _INTERSECTION_H_
#define _INTERSECTION_H_

#include "public.h"
#include "ns3/core-module.h"
#include "ns3/application-module.h"

using namespace ns3;

class Intersection
{
public:
	Intersection(Position center, int eastArmLength, int southArmLength, int westArmLength, int northArmLength,
			double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate);
	~Intersection() {}

	//gernerate traffic for the whole intersection
	void generateTraffic();	

private:

	//generate traffic for a specific lane which is implied by laneNumber
	void generateTrafficForLane(int laneNumber);

	//setup flow rate for each arm
	void setupFlowRate(double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate);		

	//phisical configuratoin of intersection
	Position m_centerPosition;

	int m_eastArmLength;
	int m_southArmLength;
	int m_westArmLength;
	int m_northArmLength;	

	//traffic flow configuration of intersection
	double m_e2wFlowRate;		//flow rate of east to west direction;
	double m_w2eFlowRate;		
	double m_s2nFlowRate;
	double m_n2sFlowRate;

	Ptr<ExponentialRandomVariable> m_intervalsOfe2wVehicle;	//time interval of two succesive vehicle arriving at e2w arm
	Ptr<ExponentialRandomVariable> m_intervalsOfw2eVehicle;
	Ptr<ExponentialRandomVariable> m_intervalsOfs2nVehicle;
	Ptr<ExponentialRandomVariable> m_intervalsOfn2sVehicle;
};

#endif

