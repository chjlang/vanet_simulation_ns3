#include "vehicle.h"
#include "topology.h"

#include <cmath>

NS_LOG_COMPONENT_DEFINE("VehicleLog");

void Vehicle::Configure(uint32_t laneID, const Vector &initialPosition, DIRECTION direction)
{
	m_isInUse = true;		//the vehicle is now in use 

	m_currentLaneID = laneID;
	m_direction = direction;
	m_status = IDLE;
	m_currentIntersectionID = Topology::GetInstance()->GetIntersectionIDFromLane(laneID);

	if(m_mobility != 0)
		m_mobility->SetPosition(initialPosition);
	else
	{
		NS_LOG_ERROR("Fail to et mobility model from node");
	}
}

void Vehicle::StartApplication()
{
	NS_LOG_DEBUG("vehicle_" << m_ID <<" start to run");
	//create socket
	TypeId typeID = TypeId::LookupByName("ns3::UdpSocketFactory");
	m_sendSocket = Socket::CreateSocket(GetNode(), typeID);
	m_receiveSocket = Socket::CreateSocket(GetNode(), typeID);

	//setup receive socket
	m_receiveSocket->Bind(InetSocketAddress(m_ipAddress, receivePort));		
	m_receiveSocket->SetRecvCallback(MakeCallback(&Vehicle::OnReceivePacket, this));	//setup callback function

	//setup course-changed callback
	m_mobility = GetNode()->GetObject<ConstantVelocityMobilityModel>();
	m_mobility->TraceConnectWithoutContext("CourseChange", MakeCallback(&Vehicle::OnCourseChanged, this));
}

void Vehicle::StopApplication()
{
	if(m_sendSocket)
		m_sendSocket->Close();
	if(m_receiveSocket)
		m_receiveSocket->Close();
}

//send packet to intersection, telling that the vehicle is going to tranform to "status"
void Vehicle::SendPacket(VEHICLE_STATUS status)
{
	//get intersection ip address
	Ipv4Address serverIP = Topology::GetInstance()->GetIntersection(m_currentIntersectionID)->GetIPAddress();
	InetSocketAddress serverAddress(serverIP, receivePort);

	//not like packet in real network, in ns3, the payload of packet is carried by header
	Ptr<Packet> packet = new Packet();
	SeqTsHeader header;

	header.SetSeq(m_ID);
	packet->AddHeader(header);
	header.SetSeq(m_currentLaneID);
	packet->AddHeader(header);
	header.SetSeq(status);
	packet->AddHeader(header);

	if(m_sendSocket->SendTo(packet, 0, serverAddress) == -1)
		NS_LOG_ERROR("Fail to send packet");
	else
		NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " seconds vehicle_" << m_ID << " send packet with status: " << status);
}

//callback is invoked when a packet is arrived
//check whether the vehicle is in permission list(plt), start driving if so 
void Vehicle::OnReceivePacket(Ptr<Socket> socket)
{
	Ptr<Packet> packet = socket->Recv();
	PltHeader header;

	packet->RemoveHeader(header);
	std::list<PltContent> plt = header.GetData();

	for(std::list<PltContent>::iterator it = plt.begin(); it != plt.end(); it++)
	{
		if(m_ID == it->vehicleID)
		{
			NS_LOG_DEBUG("vehicle_" << m_ID << " get permission to pass intersection_" << m_currentIntersectionID);

			m_status = PASSING;
			if(m_isPaused == true)
				Simulator::Schedule(Seconds(Vehicle::startUpLostTime), &Vehicle::Drive, this);

			break;
		}
	}
}

//callback function is invoked when the positon and speed of the vehicle change
void Vehicle::OnCourseChanged(Ptr<const MobilityModel> mobility)
{
	//if the callback is invoked by a velocity update, just mark this velocity and return 
	Vector currentVelocity = mobility->GetVelocity();
	if(currentVelocity.x != m_lastVelocity.x || currentVelocity.y != m_lastVelocity.y)
	{
		NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " seconds velocity changed for vehicle_" << m_ID << " (" << m_lastVelocity.x << ", "<< m_lastVelocity.y <<") --> (" << currentVelocity.x << ", " << currentVelocity.y <<")");
		m_lastVelocity = currentVelocity;
	}
}

//return true if the postions of vehicle and obstacle is within minimumHeadway
bool Vehicle::IsWithinHeadway(const Vector& obstaclePosition)
{
	NS_LOG_DEBUG("Vehicle::IsWithinHeadway() vehicle_" << m_ID << ":(" << m_mobility->GetPosition().x << ", " << m_mobility->GetPosition().y << ") obstacle:(" << obstaclePosition.x << ", " << obstaclePosition.y <<")");

	if(m_direction == EASTWARD || m_direction == WESTWARD)
	{
		NS_LOG_DEBUG("E/W distance: " << fabs(m_mobility->GetPosition().x - obstaclePosition.x));
		return (fabs(m_mobility->GetPosition().x - obstaclePosition.x) < Vehicle::minimumHeadway);
	}
	else
	{
		NS_LOG_DEBUG("S/N distance: " << fabs(m_mobility->GetPosition().y - obstaclePosition.y));
		return (fabs(m_mobility->GetPosition().y - obstaclePosition.y) < Vehicle::minimumHeadway);
	}
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

	NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " seconds position changed for vehicle_" << m_ID << " (" << m_mobility->GetPosition().x << ", "<< m_mobility->GetPosition().y << ") --> (" << newPosition.x << ", " << newPosition.y << ")");
	m_mobility->SetPosition(newPosition);
}

//return true if the vehicle has the region of current intersection
bool Vehicle::HasLeftIntersection()
{
	const Vector& intersectionPosition = Topology::GetInstance()->GetIntersection(m_currentIntersectionID)->GetPosition();
	const Vector& vehiclePosition = GetPosition();

	if(m_direction == EASTWARD || m_direction == WESTWARD)
		return (fabs(intersectionPosition.x - vehiclePosition.x) > Intersection::size / 2);
	else
		return (fabs(intersectionPosition.y - vehiclePosition.y) > Intersection::size / 2);
}

//return true if the vehicle has the region of current lane
bool Vehicle::HasLeftCurrentLane()
{
	Ptr<Intersection> intersection = Topology::GetInstance()->GetIntersection(m_currentIntersectionID);
	const Vector& vehiclePosition = GetPosition();
	const Vector& intersectionPosition = intersection->GetPosition();

	if(m_direction == EASTWARD || m_direction == WESTWARD)
		return fabs(vehiclePosition.x - intersectionPosition.x) > (Intersection::armLength + Intersection::size / 2);
	else
		return fabs(vehiclePosition.y - intersectionPosition.y) > (Intersection::armLength + Intersection::size / 2);
}

//periodically call this function to update vehicle's position
//status tranformation according to the current postion of vehicle
void Vehicle::OnPositionChanged()
{
	//check if the vehicle is still available
	if(m_currentLaneID == 0)
	{
		if(m_isInUse == true)
		{
			NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " vehicle_" << m_ID << " push back in vehicle pool");
			m_isInUse = false;
			g_vehiclePool.push_back(this); 
			
			fprintf(stderr, "g_vehiclePool: ");
	        for(std::list<Ptr<Vehicle> >::iterator it = g_vehiclePool.begin(); it != g_vehiclePool.end(); it++)
	            fprintf(stderr, "%u  ", (*it)->GetID());
	        fprintf(stderr, "\n");
	    }
		
		//Stop();
		return;
	}
	//if the vehicle is stop, there is no position change
	if(m_isPaused == true)
		return;

	//state transformation
	switch(m_status)
	{
		case IDLE: //stop and send ENTER messeage if the vehicle encouters an obstacle
		{
			Ptr<Intersection> currentIntersection = Topology::GetInstance()->GetIntersection(m_currentIntersectionID);
			Vector obstaclePosition = currentIntersection->GetObstaclePosition(m_currentLaneID, m_ID);

			NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " vehicle_" << m_ID << " (" << m_mobility->GetPosition().x << ", " << m_mobility->GetPosition().y << ")" " IDLE obstaclePosition:(" << obstaclePosition.x << ", " << obstaclePosition.y << ")");
			if(IsWithinHeadway(obstaclePosition) == true)
			{
				NS_LOG_DEBUG("vehicle_" << m_ID << " is within headway to obstacle: (" << obstaclePosition.x << ", " << obstaclePosition.y << ")");
				Stop();		
				SendPacket(WAITING);
				m_status = WAITING;
				ResetPosition(obstaclePosition);		
			}			
			break;
		}
		case WAITING:	//happen when vehicle is following
		{
			Ptr<Intersection> currentIntersection = Topology::GetInstance()->GetIntersection(m_currentIntersectionID);
			Vector obstaclePosition = currentIntersection->GetObstaclePosition(m_currentLaneID, m_ID);

			NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " vehicle_" << m_ID << " (" << m_mobility->GetPosition().x << ", " << m_mobility->GetPosition().y << ")" << " WAITING obstaclePosition:(" << obstaclePosition.x << ", " << obstaclePosition.y << ")");
			if(IsWithinHeadway(obstaclePosition) == true)
			{
				NS_LOG_DEBUG("vehicle_" << m_ID << " is within headway to obstacle: (" << obstaclePosition.x << ", " << obstaclePosition.y << ")");
				Stop();
				ResetPosition(obstaclePosition);
			}			
			break;
		}
		case PASSING:
		{
			if(HasLeftIntersection() == true)
			{
				m_nextLaneID = Topology::GetInstance()->GetDownstream(m_currentLaneID);
				m_status = EXIT;
				SendPacket(EXIT);
				NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " vehicle_" << m_ID << " (" << m_mobility->GetPosition().x << ", " << m_mobility->GetPosition().y << ")" << " PASSING intersection_" << m_currentLaneID);
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
				NS_LOG_DEBUG("At " << Simulator::Now().GetSeconds() << " vehicle_" << m_ID << " (" << m_mobility->GetPosition().x << ", " << m_mobility->GetPosition().y << ")" << " EXIT intersection_" << m_currentLaneID);
			}
			break;
		}
	}	
	Simulator::Schedule(Seconds(POSITION_UPDATE_INTERVAL), &Vehicle::OnPositionChanged, this);
}

void Vehicle::Drive()
{
	NS_LOG_DEBUG("vehicle_" << m_ID <<" begin to drive on lane_" << m_currentLaneID << " with position : (" << m_mobility->GetPosition().x << ", " << m_mobility->GetPosition().y <<")");
	//set velocity according to driving direction
	switch (m_direction)
	{
		case EASTWARD:
			m_mobility->SetVelocity(Vector(Vehicle::speed, 0, 0));
			break;
		case SOUTHWARD:
			m_mobility->SetVelocity(Vector(0, -Vehicle::speed, 0));
			break;
		case WESTWARD:
			m_mobility->SetVelocity(Vector(-Vehicle::speed, 0, 0));
			break;
		case NORTHWARD:
			m_mobility->SetVelocity(Vector(0, Vehicle::speed, 0));
			break;
	}
	m_isPaused = false;
	OnPositionChanged();
}

void Vehicle::Stop()
{	
	NS_LOG_DEBUG("vehicle_" << m_ID << " stop at ( " << m_mobility->GetPosition().x <<", " << m_mobility->GetPosition().y << " )");
	//setting velocity to (0, 0, 0) makes vehicle to stop driving
	m_isPaused = true;
	m_mobility->SetVelocity(Vector(0, 0, 0));

	Simulator::Schedule(Seconds(QUERY_INTERVAL), &Vehicle::TryToDrive, this);
}

//periodically query the obstacle position, move forward to make it as close as possible to the obstacle (trying to simulate the car-following)
void Vehicle::TryToDrive()
{
	if(m_isPaused == false || m_isInUse == false)	//since the vehicle is driving or it is in vehicle pool, there is no need to "TryToDrive"
		return;
	else
	{
		Ptr<Intersection> currentIntersection = Topology::GetInstance()->GetIntersection(m_currentIntersectionID);
		if(currentIntersection == NULL)
			return;
		Vector obstaclePosition = currentIntersection->GetObstaclePosition(m_currentLaneID, m_ID);

		//drive if the distance to obstacle is large enough, else TryToDrive in QUERY_INTERVAL
		if(IsAbleToFollow(obstaclePosition) == true)
		{
			NS_LOG_DEBUG("Vehicle::TryToDrive() vehicle_" << m_ID << " get permission to drive");
			Simulator::Schedule(Seconds(Vehicle::startUpLostTime), &Vehicle::Drive, this);
		}
		else
			Simulator::Schedule(Seconds(QUERY_INTERVAL), &Vehicle::TryToDrive, this);
	}
}

bool Vehicle::IsAbleToFollow(const Vector &obstaclePosition)
{
	if(m_direction == EASTWARD || m_direction == WESTWARD)
	{
		return (fabs(m_mobility->GetPosition().x - obstaclePosition.x) >= (Vehicle::minimumHeadway + Vehicle::lengthOfVehicle));
	}
	else
	{
		return (fabs(m_mobility->GetPosition().y - obstaclePosition.y) >= (Vehicle::minimumHeadway + Vehicle::lengthOfVehicle));
	}
}