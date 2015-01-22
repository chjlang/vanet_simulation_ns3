#include "intersection.h"
#include "vehicle.h"
#include "public.h"

void Intersection::Configure(int ID, const Vector& centerPosition
                    double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate)
{
    //setup IDs of intersection and inflow lanes
    m_ID = ID;
    m_eastWardLaneID = m_ID * 10 + 4;
    m_southWardLaneID = m_ID * 10 + 3;
    m_westWardLaneID = m_ID * 10 + 2;
    m_northWardLaneID = m_ID * 10 + 1;

    //setup center position of intersection
    m_mobility = GetNode()->GetObject<MobilityModel> ();
    if(m_mobility != 0)
        m_mobility->SetPosition(centerPosition);
    else
        NS_LOG_ERROR("Fail to get mobility model");

    //setup flow rates of the inflow lanes
    SetupFlowRate(e2wFlowRate, w2eFlowRate, s2nFlowRate, n2sFlowRate);
}

void Intersection::SetupFlowRate(double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate)
{
    //set arrival rate
    m_e2wFlowRate = e2wFlowRate;
    m_w2eFlowRate = w2eFlowRate;
    m_s2nFlowRate = s2nFlowRate;
    m_n2sFlowRate = n2sFlowRate;

    //setup random variables which are used to generate the time intervals of the arriving vehicles
    m_intervalsOfe2wVehicle = CreateObject<ExponentialRandomVariable> ();
    m_intervalsOfe2wVehicle->SetStream(1);
    m_intervalsOfe2wVehicle->SetAttribute("Mean", DoubleValue(60/e2wFlowRate));

    m_intervalsOfw2eVehicle = CreateObject<ExponentialRandomVariable> ();
    m_intervalsOfw2eVehicle->SetStream(2);
    m_intervalsOfw2eVehicle->SetAttribute("Mean", DoubleValue(60/w2eFlowRate));

    m_intervalsOfs2nVehicle = CreateObject<ExponentialRandomVariable> ();
    m_intervalsOfs2nVehicle->SetStream(3);
    m_intervalsOfs2nVehicle->SetAttribute("Mean", DoubleValue(60/s2nFlowRate));

    m_intervalsOfn2sVehicle = CreateObject<ExponentialRandomVariable> ();
    m_intervalsOfn2sVehicle->SetStream(4);
    m_intervalsOfn2sVehicle->SetAttribute("Mean", DoubleValue(60/n2sFlowRate));
}

void GenerateTraffic()    
{
    for(DIRECTION direction = EASTWARD; direction <= NORTHWARD; direction++)
        GenerateTrafficForLane(direction);
}

void GenerateTrafficForLane(DIRECTION direction)
{
    Vector initialPosition;     //initial position of the newly-generated vehicle

    switch(lane)
    {
        case NORTHWARD:
            //the newly-generated vehicle lacated at the start point of south arm initially
            initialPosition = Vector(m_centerPosition.x, m_centerPosition.y - armLength - intersectionSize / 2, 0);
            //the time interval of two successive generated vehicle is m_intervalsOfs2nVehicle->GetValue()
            Simulator::Schedule(Seconds(m_intervalsOfs2nVehicle->GetValue()), &Intersection::GenerateTrafficForLane, NORTHWARD);
            break;

        case WESTWARD:
            //the newly-generated vehicle lacated at the start point of east arm initially
            initialPosition = Position(m_centerPosition.x + armLength + intersectionSize / 2, m_centerPosition.y, 0);
            Simulator::Schedule(Seconds(m_intervalsOfe2wVehicle->GetValue()), &Intersection::GenerateTrafficForLane, WESTWARD);
            break;

        case SOUTHWARD:
            //the newly-generated vehicle lacated at the start point of north arm initially
            initialPosition = Position(m_centerPosition.x, m_centerPosition.y + armLength + intersectionSize / 2, 0);
            Simulator::Schedule(Seconds(m_intervalsOfn2sVehicle->GetValue()), &Intersection::GenerateTrafficForLane, SOUTHWARD);
            break;

        case EASTWARD:            
            //the newly-generated vehicle lacated at the start point of west arm initially
            initialPosition = Position(m_centerPosition.x - armLength - intersectionSize / 2, m_centerPosition.y, 0);    
            Simulator::Schedule(Seconds(m_intervalsOfw2eVehicle->GetValue()), &Intersection::GenerateTrafficForLane, EASTWARD);
            break;
    }

    if(g_vehiclePool.empty() == true)
        NS_LOG_ERROR("g_vehiclePool is empty, unable to generate new vehicle");
    else
    {
        Ptr<Vehicle> vehicle = g_vehiclePool.front();
        g.pop();
        //emit vehicles: configure the vehicle with proper laneID, initial position and driving direction
        vehicle->Configure(GetLaneID(direction), initialPosition, direction);
        vehicle->Drive();
    }
}

int Intersection::GetLaneID(DIRECTION direction)
{
    switch(direction)
    {
        case EASTWARD:
            return m_eastWardLaneID;
        case SOUTHWARD:
            return m_southWardLaneID;
        case WESTWARD:
            return m_westWardLaneID;
        case NORTHWARD:
            return m_northWardLaneID;
    }
}

//return the last obstacle position of the a specific lane
const Vector Intersection::GetObstaclePosition(int laneNumber)
{
    switch(laneNumber)
    {
        case m_eastWardLaneID:
        {
            if(m_eastWardLaneQueue.empty() == true)     //if there is no vehicle waiting in this lane, return the edge of intersetion region
                return m_mobility->GetPosition() + Vector(-Intersection::size / 2, 0, 0);
            else                                    
                return m_eastLaneWardQueue.back()->GetPosition();   
        }
        case m_southWardLaneID:
        {
            if(m_southWardLaneQueue.empty() == true)
                return m_mobility->GetPosition() + Vector(0, Intersection::size / 2, 0);
            else
                return m_southWardLaneQueue.back()->GetPosition();
        }
        case m_westWardLaneID:
        {
            if(m_westWardLaneQueue.empty() == true)
                return m_mobility->GetPosition() + Vector(Intersection::size / 2, 0, 0);
            else
                return m_westWardLaneQueue.back()->GetPosition();
        }
        case m_northWardLaneID:
        {
            if(m_northWardLaneQueue.emtpy() == true)
                return m_mobility->GetPosition() + Vector(0, -Intersection::size / 2, 0);
            else
                return m_northWardLaneQueue.back()->GetPosition();
        }
    }
}