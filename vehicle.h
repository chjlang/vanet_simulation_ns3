#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "ns3/constant-velocity-mobility-model.h"

enum VEHICLE_STATUS {IDLE, WAITING, PASSING, EXIT};

class Vehicle : public Application
{
public:
	Vehicle(){}
	~Vehicle() {}

	void Drive();	//drive with constant speed
	void Stop();

	//setup the initial position and velocity of vehicle, return true if configure successfully
	bool ConfigureVehicle(int laneID, const Vector &initialPosition, VEHICLE_DIRECTION direction);

	//callback function when the positon and speed of the vehicle change
	void OnCourseChanged(std::string context, Ptr<MobilityModel> mobility);

	const Vector GetPosition() { return m_mobility->GetPosition(); }
	
private:
	//while vehicle is EXIT, reset laneID if the vehicle has transfer to another lane, otherwise, do nothing 
	void TransferToOtherLane();

	Ptr<ConstantVelocityMobilityModel> m_mobility;

	//flag indicating whether the vehicle is availabe to emit
	bool m_freeFlag;

	//driving status
	int m_currentIntersectionID		//intersection ID at which the vehicle arrive
	int m_currentLaneID;			//lane ID on which the vehicle is driving
	int m_status;
	DIRECTION m_direction;

	int m_nextLaneID;				//lane ID which the vehicle is heading to 
	Vector m_lastVelocity;		//keep track of the velocity of the last callback

	//paramenters of vehicle
	static int lengthOfVehicle = 5;		//all vehicles are set to be 5 meters long 
	static double speed = 10;			//all vehicles' speed are set to be 10 m/s
	static double startUpLostTime = 3.5;	//the start up lost time
	static double minimumHeadway = 2;		//headway (in meters) between two succesive vehicles
};

#endif
