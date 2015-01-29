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
 
struct GroupInformation
{	
	std::vector<uint32_t> members;
	double expectedArrivalTime;				//the expected arrival time at the intersection for the first vehicle in group
											//(used for scheduling algorithm)
	std::map<uint32_t, double> timeStamp;	//mark when did the vehicle join group
	double GetTotalDelay(double now)
	{
		double totalDelay = 0;
		for(std::map<uint32_t, double>::iterator it = timeStamp.begin(); it != timeStamp.end(); it++)
			totalDelay += (now - it->second);

		return totalDelay;
	}

	double GetAverageDelay(double now)
	{
		return GetTotalDelay(now) / members.size();
	}
};

class Intersection : public Application
{
public:
	Intersection() {}
	~Intersection() {}
	
	//setup intersection: ID, position, flow rates
	void Configure(int ID, Ipv4Address ip, const Vector& centerPosition, 
			double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate);

	//gernerate traffic for the whole intersection
	void GenerateTraffic();	

	uint32_t GetID() { return m_ID; }
	uint32_t GetLaneID(DIRECTION direction);
	
	const Vector GetPosition() { return m_mobility->GetPosition() };
	//get position of the obstacle ahead of vehicleID in laneID
	const Vector GetObstaclePosition(int laneID, int vehicleID);

	const Ipv4Address GetIPAddress() { return m_ipAddress; }

	static double size = 15;		//the size of intersection region is set to: 15m * 15m
	static double armLength = 75;	//all arms are to 75 meters long
	static uint16_t receivePort = 8081;		//port number of receive socket

	static double minGreenTime = 5;

	static uint16_t receivePort = 8081;		//port number of receive socket
	
private:
	void StartApplication();			//this function is called by simulaiton at the start time specific by "Start"
	void StopApplication();				//this function is called by simulaiton at the start time specific by "Stop"

	void OnReceivePacket(Ptr<Socket> socket);
	void SendPacket();

	//generate traffic for a specific lane which is implied by laneNumber
	void GenerateTrafficForLane(DIRECTION direction);

	//setup flow rate for each arm
	void SetupFlowRate(double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate);		

	/***scheduling-related functions***/
	bool IsAbleJoinGroup(uint32_t vehicleID, uint32_t laneID);
	double BenefitOfJoiningGroup(uint32_t vehicleID, uint32_t laneID);
	int GetConcurrentGroupSize(uint32_t laneID);
	void Schedule();

	/***attributes of scheduling***/
	std::list<PltContent> m_plt;				//permission list
	std::map<uint32_t, std::list<GroupInformation>* > m_laneGroup;		//keep track of groups in each lane

	FNN *m_groupingFnn;

	Scheduler m_sheduler;

	friend void Scheduler::Convert2Inflow(const Intersection &intersection);


	//id of the intersection and its inflow lanes
	uint32_t m_ID;
	uint32_t m_eastWardLaneID;
	uint32_t m_southWardLaneID;
	uint32_t m_westWardLaneID;
	uint32_t m_nortWardhLaneID;

	/***network attribute***/
	Ipv4Address m_ipAddress;
	Ptr<Socket> m_sendSocket;
	Ptr<Socket> m_receiveSocket;

	//lists which are used to manage vehicles in each lane
	std::map<uint32_t, std::list<Ptr<Vehicle> >* > m_laneFlow;

	/***phisical configuration of intersection***/
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

