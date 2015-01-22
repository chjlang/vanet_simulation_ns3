#ifndef _INTERSECTION_H_
#define _INTERSECTION_H_

#include "public.h"
#include "ns3/core-module.h"
#include "ns3/application-module.h"

using namespace ns3;

/***********phisical layout************
				|  |  |
				| 3|  |
		  ______|  |  |______
          ______       ___2__
          ___4__       ______
                |  |  |
                |  | 1|
                |  |  |
**************************************/
 
class Intersection : public Application
{
public:
	Intersection() {}
	~Intersection() {}
	
	//setup intersection: ID, position, flow rates
	void Configure(int ID, const Vector& centerPosition, 
			double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate);

	//gernerate traffic for the whole intersection
	void GenerateTraffic();	

	int GetID() { return m_ID; }
	int GetLaneID(DIRECTION direction);
	const Vector GetPosition() { return m_mobility->GetPosition() };

	//get position of the last obstacle in lane of which the ID is laneID
	const Vector GetObstaclePosition(int laneID);

	static double size = 15;		//the size of intersection region is set to: 15m * 15m
	static double armLength = 75;	//all arms are to 75 meters long

private:

	//generate traffic for a specific lane which is implied by laneNumber
	void GenerateTrafficForLane(DIRECTION direction);

	//setup flow rate for each arm
	void SetupFlowRate(double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate);		

	//id of the intersection and its inflow lanes
	int m_ID;
	int m_eastWardLaneID;
	int m_southWardLaneID;
	int m_westWardLaneID;
	int m_nortWardhLaneID;

	//lists which are used to manage vehicles in each lane
	std::queue<Ptr<Vehicle> > m_eastWardLaneQueue;
	std::queue<Ptr<Vehicle> > m_southWardLaneQueue;
	std::queue<Ptr<Vehicle> > m_westWardLaneQueue;
	std::queue<Ptr<Vehicle> > m_northWardLaneQueue;

	//phisical configuratoin of intersection
	Ptr<ConstantPositionMobilityModel> m_mobility; //mobility model

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

