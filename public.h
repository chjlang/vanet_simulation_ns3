#ifndef _PUBLIC_H_
#define _PUBLIC_H_

#include <list>
#include <string>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

class Vehicle;

struct GroupInformation
{	
	std::vector<uint32_t> members;
	uint32_t laneID;
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

enum DIRECTION {EASTWARD, SOUTHWARD, WESTWARD, NORTHWARD};
enum VEHICLE_STATUS {IDLE, WAITING, PASSING, EXIT};

const double MAX_X_COORDINATE = 1000;		//maximum x coordinate of the simulated region
const double MAX_Y_COORDINATE = 1000;		//maximum y coordinate of the simulated region

const std::string GROUPING_FIS_FILE = "/home/cjl/ns3/scratch/vanet_simulation/grouping.fis";
const double SIMULATION_STEP = 1;

extern std::list<ns3::Ptr<Vehicle> > g_vehiclePool;					//record all free vehicles, used in generating traffic
extern std::map<uint32_t, ns3::Ptr<Vehicle> > g_vehicleMapping; 

#endif