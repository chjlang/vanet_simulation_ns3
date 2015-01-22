#include "simulator.h"

#include "vehicle.h"

void Vehicle::OnCourseChanged(std::string context, Ptr<MobilityModel> mobility)
{
	//if the callback is invoked by a velocity update, just mark this velocity and skip the rest of this function
	Vector currentVelocity = mobility->GetVelocity();
	if(currentVelocity.x != m_lastVelocity.x || currentVelocity.y != m_lastVelocity.y)
	{
		m_lastVelocity = currentVelocity;
		return;
	}

	NS_LOG_DEBUG(context + " x: " + mobility->GetPosition().x + " y: " + mobility->GetPosition().y);

	//check if the vehicle is still available
	if(m_currentLaneID == -1)
	{
		m_freeFlag = true;
		g_vehiclePool.push(this); 
		return;
	}

	//state transformation
	switch(m_status)
	{
		case IDLE: //stop and send ENTER messeage if the vehicle encouters an obstacle
		{
			Ptr<Intersection> currentIntersection = Topology::GetIntersection(m_currentIntersectionID);
			Vector obstaclePosition = currentIntersection->GetObstaclePosition(m_currentLaneID);

			if(IsWithinHeadway(obstaclePosition) == true)
			{
				Stop();		
				SendMsg(ENTER);
				ResetPosition(obstaclePosition);
				m_status = WAITING;
			}			
			break;
		}
		case PASSING:
		{
			if(HasLeftIntersection() == true)
			{
				m_nextLaneID = Topology::GetInstance()->GetDownStream(m_currentLaneID);
				m_status = EXIT;
				SendMsg(EXIT);
			}
			break;
		}
		case EXIT:
		{
			if(HasLeftCurrentLane() == true)
			{
				m_currentLaneID = m_nextLaneID;
				m_currentIntersectionID = Topology::GetInstance()->GetIntersectionIDFromLane(m_currentLaneID);
				m_status = IDLE;
			}
			break;
		}
	}
}

bool Vehicle::IsWithinHeadway(const Vector& obstaclePosition)
{
	if(m_direction == EASTWARD || m_direction == WESTWARD)
		return (fabs(m_mobility->GetPosition().x, obstaclePosition.x) < Vehicle::minimumHeadway);
	else
		return (fabs(m_mobility->GetPosition().y, obstaclePosition.y) < Vehicle::minimumHeadway);
}

//reset vehicle position according to the obstacle position, maintaining the minimum headway between two succesive vehicles
void Vehicle::ResetPosition(const Vector& obstaclePosition)
{
	NS_LOG_DEBUG("Reset position");

	Vector newPosition(obstaclePosition);
	switch(m_direction)
	{
		case EASTWARD:
			newPosition.x = obstaclePosition.x - Vehicle::minimumHeadway;
			break;
		case SOUTHWARD:
			newPosition.y = obstaclePosition.y + Vehicle::minimumHeadway;
			break;
		case WESTWARD:
			newPosition.x = obstaclePosition.x + Vehicle::minimumHeadway;
			break;
		case NORTHWARD:
			newPosition.y = obstaclePosition.y - Vehicle::minimumHeadway;
			break;
	}
	m_mobility->SetPositon(newPosition);
}

bool Vehicle::HasLeftIntersection()
{
	const Vector& intersectionPosition = GetIntersection();
	const Vector& vehiclePosition = m_mobility->GetPosition();

	return (fabs(intersectionPosition.x, vehiclePosition.x) < Intersection::size / 2) &&
			(fabs(intersectionPosition.y, vehiclePosition.y) < Intersection::size / 2)
}

bool Vehicle::HasLeftCurrentLane()
{
	Ptr<Intersection> intersection = Topology::GetInstance()->GetIntersection(m_currentIntersection);
	if(m_direction == EASTWARD || m_direction == WESTWARD)
		return fabs(GetPosition().x, intersection->GetPosition().x) > (Intersection::armLength + Intersection::size / 2);
	else
		return fabs(GetPosition().y, intersection->GetPosition().y) > (Intersection::armLength + Intersection::size / 2);
}

bool Vehicle::ConfigureVehicle(int laneID, const Vector &initialPosition, DIRECTION direction)
{
	m_freeFlag = false;		//the vehicle is now in use 

	m_currentIntersectionID = Topology::GetIntersectionIDFromLane(laneID);
	m_currentLaneID = laneID;
	m_direction = direction;
	m_status = IDLE;

	m_mobility = GetNode()->GetObject<MobilityModel> ();
	if(m_mobility != 0)
		m_mobility->SetPositon(initialPosition);
	else
	{
		NS_LOG_ERROR("Fail to et mobility model from node");
		return false;
	}
	return true;
}

void Drive()
{
	Vector velocity;
	//set velocity according to driving direction
	switch (m_direction)
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
	m_mobility->SetVelocity(velocity);
}

void Vehicle::Stop()
{	
	//setting velocity to (0, 0, 0) makes vehicle to stop driving
	m_mobility->SetVelocity(Vector(0, 0, 0));
}