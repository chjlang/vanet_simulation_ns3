#include "simulator.h"

#include "vehicle.h"

void Vehicle::ConfigureVehicle(const Vector &initialPosition, DIRECTION direction)
{
	m_mobility.SetPositon(initialPosition);
	m_drivingDirection = direction;
}

void Drive()
{
	Vector velocity;
	//set velocity according to driving direction
	switch (m_drivingDirection)
	{
		case EASTWARD:
			velocity += Vector(speed, 0, 0);
			break;
		case SOUTHWARD:
			velocity += Vector(0, -speed, 0);
			break;
		case WESTWARD:
			velocity += Vector(-speed, 0, 0);
			break;
		case NORTHWARD:
			velocity += Vector(0, speed, 0);
			break;
	}
	m_mobility.SetVelocity(velocity);
}

void Vehicle::Stop()
{	
	//setting velocity to (0, 0, 0) makes vehicle to stop driving
	m_mobility.SetVelocity(Vector(0, 0, 0));
}