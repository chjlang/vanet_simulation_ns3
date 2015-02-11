#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"

#include "public.h"

using namespace ns3;


class Vehicle : public Application
{
public:
	Vehicle(uint16_t ID, Ipv4Address ip) { m_ID = ID; m_ipAddress = ip; }
	virtual ~Vehicle() {}

	//drive with constant speed
	void Drive();	
	void Stop();

	//setup the initial position and velocity of vehicle
	void Configure(uint32_t laneID, const Vector &initialPosition, DIRECTION direction);

	//callback function when the positon and speed of the vehicle change
	void OnCourseChanged(Ptr<const MobilityModel> mobility);

	const Vector GetPosition() { return m_mobility->GetPosition(); }
	const uint32_t GetID() { return m_ID; }

	const static uint16_t receivePort = 8081;		//port number of receive socket

	//common paramenters of all vehicles
	const static uint16_t lengthOfVehicle = 5;	//all vehicles are set to be 5 meters long 
	const static double speed = 10;				//all vehicles' speed are set to be 10 m/s
	const static double startUpLostTime = 3.5;	//the start up lost time
	const static double minimumHeadway = lengthOfVehicle + 2;		//headway (in meters) between two succesive vehicles
	
private:
	void StartApplication();			//this function is called by simulaiton at the start time specific by "Start"
	void StopApplication();				//this function is called by simulaiton at the start time specific by "Stop"

	/***wireless communication function***/
	void SendPacket(VEHICLE_STATUS status);
	void OnReceivePacket(Ptr<Socket> socket);

	/***utility functions used in vehicle status tranformation***/

	//return true if the postions of vehicle and obstacle is within minimumHeadway
	bool IsWithinHeadway(const Vector& obstaclePosition);

	//return true if the vehicle is able to follow the previous vehicle, the distance is not less than minimumHeadway + lengthOfVehicle
	bool IsAbleToFollow(const Vector &obstaclePosition);

	//reset vehicle position according to the obstacle position, maintaining the minimum headway between two succesive vehicles	
	void ResetPosition(const Vector& obstaclePosition);	

	//return true if the vehicle has the region of current intersection
	bool HasLeftIntersection();

	//return true if the vehicle has the region of current lane
	bool HasLeftCurrentLane();

	void TryToDrive();

	void OnPositionChanged();


	/***private variable declaration***/

	uint32_t m_ID;
	bool m_isInUse;	//flag indicating whether the vehicle is in use	
	
	//communication parameters
	Ipv4Address m_ipAddress;
	Ptr<Socket> m_sendSocket;
	Ptr<Socket> m_receiveSocket;

	//driving status
	Ptr<ConstantVelocityMobilityModel> m_mobility;			//constant velocity mobility model
	uint32_t m_currentIntersectionID;		//intersection ID at which the vehicle arrive
	uint32_t m_currentLaneID;				//lane ID on which the vehicle is driving
	VEHICLE_STATUS m_status;
	DIRECTION m_direction;
	bool m_isPaused;

	uint32_t m_nextLaneID;				//lane ID which the vehicle is heading to 
	Vector m_lastVelocity;				//keep track of the velocity of the last callback

};

const double POSITION_UPDATE_INTERVAL = (Vehicle::minimumHeadway - 1) / Vehicle::speed;		//MUST BE less than minimumHeadway / speed
																							//to prevent invalid overtaking from happening
const double QUERY_INTERVAL = 0.01;				//interval for the vehicle to query the position of obstacle, should be small enough

#endif
