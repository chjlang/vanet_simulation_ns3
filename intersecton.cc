#include "intersection.h"
#include "vehicle.h"
#include "public.h"

void Intersection::Configure(int ID, Ipv4Address ip, const Vector& centerPosition
                    double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate)
{
    //setup IDs of intersection and inflow lanes
    m_ID = ID;
    m_eastWardLaneID = m_ID * 10 + 4;
    m_southWardLaneID = m_ID * 10 + 3;
    m_westWardLaneID = m_ID * 10 + 2;
    m_northWardLaneID = m_ID * 10 + 1;

    //set mapping from laneID to m_laneGroup and m_laneQueue
    for(DIRECTION direction = EASTWARD; direction <= NORTHWARD; direction++)
    {
        uint32_t laneID = GetLaneID(direction);
        m_laneFlow[laneID] = new std::list<Ptr<Vehicle> >;
        m_laneGroup[laneID] = new std::list<GroupInformation>;
    }

    m_ipAddress = ip;

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

    switch(direction)
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
const Vector Intersection::GetObstaclePosition(int laneNumber, int vehicleID)
{
    if(m_laneFlow[laneNumber]->empty() == true)
    {
        switch(laneNumber)
        {
            case m_eastWardLaneID:
                return m_mobility->GetPosition() + Vector(-Intersection::size / 2, 0, 0);
            case m_southWardLaneID:
                return m_mobility->GetPosition() + Vector(0, Intersection::size / 2, 0);
            case m_westWardLaneID:
                return m_mobility->GetPosition() + Vector(Intersection::size / 2, 0, 0);
            case m_northWardLaneID:
                return m_mobility->GetPosition() + Vector(0, -Intersection::size / 2, 0);
        }
    }
    else
    {
        std::list<Ptr<Vehicle> > *laneFlow = m_laneFlow[laneNumber];
        std::list<Ptr<Vehicle> >::iterator prev = laneFlow.begin(), cur = prev + 1;
        while(cur != laneFlow.end() && (*cur)->GetID() == vehicleID)
        {
            prev = cur;
            cur++;
        }
        return (*prev)->GetPosition();
    }
}

void Intersection::StartApplication()
{
    //setup send socket
    TypeId typeID = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_sendSocket = Socket::CreateSocket (GetNode (), typeID);
    m_sendSocket->SetAllowBroadcast(true);      //the controller brocast the plt_list to all vehicles

    //setup receive socket
    m_receiveSocket = Socket::CreateSocket(GetNode(), typeID);
    m_receiveSocket->Bind(InetSocketAddress(m_ipAddress, receivePort));

    //setup fnn
    m_groupingFnn = new FNN(3);
    m_groupingFnn->Initialize(GROUPING_FIS_FILE);

    Schedule();
}

void Intersection::StopApplication()
{
    if(m_sendSocket)
        m_sendSocket->Close();
    if(m_receiveSocket)
        m_receiveSocket->Close();
}

void Intersection::SendPacket()
{
    NS_LOG_DEBUG("At "<<Simulator::Now().GetSeconds()<<" seconds");
    NS_LOG_DEBUG("Plt:");
    for(std::list<PltContent>::iterator it = plt.begin(); it != plt.end(); it++)
        NS_LOG_DEBUG(it->vehicleID);

    //brocast the plt list to all vehicles
    Ptr<Packet> packet = new Packet();
    PltHeader header;
    header.SetData(plt);
    packet->AddHeader(header);

    InetSocketAddress brocastAddress(Ipv4Address("255.255.255.255"), Vehicle::receivePort);

    m_sendSocket->SendTo(packet, 0, brocastAddress);    
}

void Intersection::OnReceivePacket(Ptr<Socket> socket)
{
    //extract packet and its content
    Ptr<Packet> packet = socket->Recv();

    SeqTsHeader header;
    uint32_t vehicleID, vehicleLaneID;
    VEHICLE_STATUS vehicleStatus;

    packet->RemoveHeader(header);
    status = header.GetSeq();
    packet->RemoveHeader(header);
    vehicleLaneID = header.GetSeq();
    packet->RemoveHeader(header);
    vehicleID = header.GetSeq();

    switch(vehicleStatus)
    {
        case WAITING:       //when the vehicle is entering WAITING status, do grouping
        {
            NS_LOG_DEBUG(Simulator::Now().GetSeconds()<<" second intersection_" <<m_ID<<" receive WAITING packet from "<<vehicleID<<"\n");

            m_laneQueue[vehicleLaneID]->push(vehicleID);

            //if there is no group in lane or the vehicle is not permitted to join tailing group, form a new one
            if(m_laneGroup[vehicleLaneID]->empty() == true || IsAbleJoinGroup(vehicleID, vehicleLaneID) == false)  
            {
                GroupInformation newGroup;
                newGroup.members.push_back(vehicleID);
                newGroup.expectedArrivalTime = Simulator::Now().GetSeconds() + armLength / Vehicle::speed;
                newGroup.timeStamp[vehicleID] = Simulator::Now().GetSeconds();

                m_laneGroup[vehicleLaneID]->push_back(newGroup);
            }
            else    //the vehicle is going to join the tailing group
            {
                //update information of the tailing group
                std::list<GroupInformation>::iterator itLastGroup = m_laneGroup[vehicleLaneID]->rbegin();
                itLastGroup->members.push_back(vehicleID);
                itLastGroup->timeStamp[vehicleID] = Simulator::Now().GetSeconds();
            }

            break;
        }
        case EXIT:  //when the vehicle has left intersection region, delete it from plt
        {
            NS_LOG_DEBUG(Simulator::Now().GetSeconds()<<" second intersection_" <<m_ID<<" receive EXIT packet from "<<vehicleID<<"\n");

            for(std::list<PltContent>::iterator it = plt.begin(); it != plt.end(); it++)
            {
                if(vehicleID == it->vehicleID)
                {
                    plt.erase(it);
                    break;
                }
            }
            break;
        }
    }
}

bool Intersection::IsAbleJoinGroup(uint32_t vehicleID, uint32_t laneID)
{
    double benefit = BenefitOfJoiningGroup(vehicleID, laneID);

    if(benefit >= 0.5)
    {
        NS_LOG_DEBUG(vehicleID<<" join group");
        return true;
    }
    else
    {
        NS_LOG_DEBUG(vehicleID<<" setup new group");
        return false;
    }
}

double Intersection::BenefitOfJoiningGroup(uint32_t vehicleID, uint32_t laneID)
{
    GroupInformation tailGroup = m_laneGroup[laneID]->back();
    double queueLength =  tailGroup.members.size();
    double averageDelay = tailGroup.GetAverageDelay(Simulator::Now().GetSeconds());
    int diffWithConcurrentLane = queueLength - GetConcurrentGroupSize();

    std::vector<double> fnnInput;
    fnnInput.push_back(queueLength);
    fnnInput.push_back(averageDelay);
    fnnInput.push_back(diffWithConcurrentLane);
    double benefit = m_groupingFnn->Forward(input); 

    NS_LOG_DEBUG("Benefit of joining group for vehicle "<<vehicleID<<" : "<<benefit);
    return benefit;
}

int Intersection::GetConcurrentGroupSize(uint32_t laneID)
{
    std::list<GroupInformation> *groups = NULL;
    switch(laneID)
    {
        case EASTWARD:
        {
            groups = m_laneGroup[m_westWardLaneID];
            break;
        }
        case SOUTHWARD:
        {
            groups = m_laneGroup[m_northWardLaneID];
            break;
        }
        case WESTWARD:
        {
            groups = m_laneGroup[m_eastWardLaneID];
            break;
        }
        case NORTHWARD:
        {
            groups = m_laneGroup[m_southWardLaneID];
            break;
        }
    }    
    
    if(groups->empty() == true)
        return 0;
    else
        return groups->back().members.size();
}

void Intersection::Schedule()
{
    if(m_plt.empty() == true)
    {
        ConstructPlt();
        SendPacket();
        Simulator::Schedule(Seconds(SIMULATION_STEP), &Intersection::Schedule, this);
    }
}
