#include "constant-velocity-mobility-model.h"

enum DIRECTION {EASTWARD, SOUTHWARD, WESTWARD, NORTHWARD};

class Vehicle
{
public:
	Vehicle() : mobility () {}
	~Vehicle() {}

	void Drive();	//drive with constant speed
	void Stop();

	//setup the initial position and velocity of vehicle
	void ConfigureVehicle(const Vector &initialPosition, DIRECTION direction);
	
private:
	ConstantVelocityMobilityModel m_mobility;
	DIRECTION m_drivingDirection;

	static int lengthOfVehicle = 5;		//all vehicles are set to be 5 meters long 
	static double speed = 10;			//all vehicles' speed are set to be 10 m/s
	static double startUpLostTime = 3.5;	//the start up lost time
};
