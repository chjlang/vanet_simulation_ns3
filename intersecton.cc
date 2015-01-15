#include "intersection.h"

Intersection::Intersection(Position center, int eastArmLength, int southArmLength, int westArmLength, int northArmLength
                    double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate)
{
    m_centerPosition = center;

    m_eastArmLength = eastArmLength;
    m_southArmLength = southArmLength;
    m_westArmLength = westArmLength;
    m_northArmLength = northArmLength;

    setupFlowRate(e2wFlowRate, w2eFlowRate, s2nFlowRate, n2sFlowRate);
}


void setupFlowRate(double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate)
{
    m_e2wFlowRate = e2wFlowRate;
    m_w2eFlowRate = w2eFlowRate;
    m_s2nFlowRate = s2nFlowRate;
    m_n2sFlowRate = n2sFlowRate;

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

void generateTraffic()    
{
    for(int i = 1; i <= 4; i++)
        generateTrafficForLane(i);    
}

void generateTrafficForLane(int laneNumber)
{
    Position initialPosition(0, 0);     //initial position of the newly-generated vehicle

    switch(laneNumber)
    {
        case 1:
            //the newly-generated vehicle lacated at the start point of south arm initially
            initialPosition = Position(m_centerPosition.m_x, m_centerPosition.m_y - m_southArmLength);
            //the time interval of two successive generated vehicle is m_intervalsOfs2nVehicle->GetValue()
            Simulator::Schedule(Seconds(m_intervalsOfs2nVehicle->GetValue()), &Intersection::generateTrafficForLane, 1);
            break;

        case 2:
            //the newly-generated vehicle lacated at the start point of east arm initially
            initialPosition = Position(m_centerPosition.m_x + m_eastArmLength, m_centerPosition.m_y);
            Simulator::Schedule(Seconds(m_intervalsOfe2wVehicle->GetValue()), &Intersection::generateTrafficForLane, 2);
            break;

        case 3:
            //the newly-generated vehicle lacated at the start point of north arm initially
            initialPosition = Position(m_centerPosition.m_x, m_centerPosition.m_y + m_northArmLength);
            Simulator::Schedule(Seconds(m_intervalsOfn2sVehicle->GetValue()), &Intersection::generateTrafficForLane, 3);
            break;

        case 4:            
            //the newly-generated vehicle lacated at the start point of west arm initially
            initialPosition = Position(m_centerPosition.m_x - m_westArmLength, m_centerPosition.m_y);
            Simulator::Schedule(Seconds(m_intervalsOfw2eVehicle->GetValue()), &Intersection::generateTrafficForLane, 4);
            break;

        default:
            LogError("Invalid laneNumber");
            return;
    }

    Ptr<Vehicle> vehicle = getVehicleFromPool(list<Ptr<Vehicle> > vehiclePool);
    if(vehicle == NULL)
        LogError("getVehicleFromPool failed");
    else
    {
        vehicle->setPosition(initialPosition);
        vehicle->startDriving();
    }
}