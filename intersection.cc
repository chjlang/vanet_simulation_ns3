#include "intersection.h"
#include "traffic_scheduler.h"
#include "vehicle.h"
#include "public.h"

#include "ns3/vector.h"
#include <stdio.h>

NS_LOG_COMPONENT_DEFINE("IntersectionLog");

void Intersection::Configure(uint32_t ID, Ipv4Address ip, const Vector& centerPosition,
                    double e2wFlowRate, double w2eFlowRate, double s2nFlowRate, double n2sFlowRate)
{
    //setup IDs of intersection and inflow lanes
    m_ID = ID;
    m_eastWardLaneID = m_ID * 10 + 4;
    m_southWardLaneID = m_ID * 10 + 3;
    m_westWardLaneID = m_ID * 10 + 2;
    m_northWardLaneID = m_ID * 10 + 1;

    //set mapping from laneID to m_laneGroup and m_laneQueue
    for(DIRECTION direction = EASTWARD; direction <= NORTHWARD; direction = DIRECTION((uint32_t)direction + 1))
    {
        uint32_t laneID = GetLaneID(direction);
        m_laneFlow[laneID] = new std::list<uint32_t>;
        m_laneGroup[laneID] = new std::list<GroupInformation>;
    }

    m_ipAddress = ip;

    //setup center position of intersection
    m_centerPosition = centerPosition;
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

void Intersection::GenerateTraffic()    
{
    for(DIRECTION direction = EASTWARD; direction <= NORTHWARD; direction = DIRECTION((uint32_t)direction + 1))
        GenerateTrafficForLane(direction);
}

void Intersection::GenerateTrafficForLane(DIRECTION direction)
{
    Vector initialPosition;     //initial position of the newly-generated vehicle

    switch(direction)
    {
        case NORTHWARD:
            if(m_s2nFlowRate == 0)
                return;
            //the newly-generated vehicle lacated at the start point of south arm initially
            initialPosition = Vector(m_centerPosition.x, m_centerPosition.y - armLength - Intersection::size / 2, 0);
            //the time interval of two successive generated vehicle is m_intervalsOfs2nVehicle->GetValue()
            Simulator::Schedule(Seconds(m_intervalsOfs2nVehicle->GetValue()), &Intersection::GenerateTrafficForLane, this, NORTHWARD);
            break;

        case WESTWARD:
            if(m_e2wFlowRate == 0)
                return;
            //the newly-generated vehicle lacated at the start point of east arm initially
            initialPosition = Vector(m_centerPosition.x + armLength + Intersection::size / 2, m_centerPosition.y, 0);
            Simulator::Schedule(Seconds(m_intervalsOfe2wVehicle->GetValue()), &Intersection::GenerateTrafficForLane, this, WESTWARD);
            break;

        case SOUTHWARD:
            if(m_n2sFlowRate == 0)
                return;
            //the newly-generated vehicle lacated at the start point of north arm initially
            initialPosition = Vector(m_centerPosition.x, m_centerPosition.y + armLength + Intersection::size / 2, 0);
            Simulator::Schedule(Seconds(m_intervalsOfn2sVehicle->GetValue()), &Intersection::GenerateTrafficForLane, this, SOUTHWARD);
            break;

        case EASTWARD:   
            if(m_w2eFlowRate == 0)
                return;         
            //the newly-generated vehicle lacated at the start point of west arm initially
            initialPosition = Vector(m_centerPosition.x - armLength - Intersection::size / 2, m_centerPosition.y, 0);    
            Simulator::Schedule(Seconds(m_intervalsOfw2eVehicle->GetValue()), &Intersection::GenerateTrafficForLane, this, EASTWARD);
            break;
    }

    if(g_vehiclePool.empty() == true)
        NS_LOG_ERROR("g_vehiclePool is empty, unable to generate new vehicle");
    else
    {
        fprintf(stderr, "g_vehiclePool: ");
        for(std::list<Ptr<Vehicle> >::iterator it = g_vehiclePool.begin(); it != g_vehiclePool.end(); it++)
            fprintf(stderr, "%u  ", (*it)->GetID());
        fprintf(stderr, "\n");
        
        NS_LOG_DEBUG("Generating new vehicle on lane_" << GetLaneID(direction) << " : vehicle_" << g_vehiclePool.front()->GetID());
        Ptr<Vehicle> vehicle = g_vehiclePool.front();
        g_vehiclePool.pop_front();
        //emit vehicles: configure the vehicle with proper laneID, initial position and driving direction
        vehicle->Configure(GetLaneID(direction), initialPosition, direction);
        vehicle->Drive();
    }
}

uint32_t Intersection::GetLaneID(DIRECTION direction)
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
const Vector Intersection::GetObstaclePosition(uint32_t laneNumber, uint32_t vehicleID)
{
    if(m_laneFlow[laneNumber]->empty() == true || m_laneFlow[laneNumber]->front() == vehicleID)
    {
        if(laneNumber == m_eastWardLaneID)
            return Vector(m_centerPosition.x - Intersection::size / 2, m_centerPosition.y, 0);
        else if (laneNumber == m_southWardLaneID)
            return Vector(m_centerPosition.x, m_centerPosition.y + Intersection::size / 2, 0);
        else if(laneNumber == m_westWardLaneID)
            return Vector(m_centerPosition.x + Intersection::size / 2, m_centerPosition.y, 0);
        else if(m_northWardLaneID)
            return Vector(m_centerPosition.x, m_centerPosition.y - Intersection::size / 2, 0);
        else
        {
            NS_LOG_ERROR("Invalid laneNumber in GetObstaclePosition");
            return Vector(0, 0, 0);
        }
    }
    else
    {
        std::list<uint32_t> *laneFlow = m_laneFlow[laneNumber];
        std::list<uint32_t>::iterator prev = laneFlow->begin(), cur = laneFlow->begin();
        cur++;
        while(cur != laneFlow->end() && (*cur) != vehicleID)
        {
            prev = cur;
            cur++;
        }
        return g_vehicleMapping[(*prev)]->GetPosition();
    }
}

void Intersection::StartApplication()
{
    NS_LOG_DEBUG("Intersection_" << m_ID <<" start to run");
    //setup send socket
    TypeId typeID = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_sendSocket = Socket::CreateSocket (GetNode(), typeID);
    m_sendSocket->SetAllowBroadcast(true);      //the controller brocast the plt_list to all vehicles

    //setup receive socket
    m_receiveSocket = Socket::CreateSocket(GetNode(), typeID);
    m_receiveSocket->Bind(InetSocketAddress(m_ipAddress, receivePort));
    m_receiveSocket->SetRecvCallback(MakeCallback(&Intersection::OnReceivePacket, this));

    //setup fnn
    m_groupingFnn = new FNN(3);
    m_groupingFnn->Initialize(GROUPING_FIS_FILE);

    //setup traffic scheduler
    m_trafficScheduler = new TrafficScheduler;

    GenerateTraffic();
    Schedule();
}

void Intersection::StopApplication()
{
    //these two lines cause run time error, check it out later (2015.2.6)
    // if(m_groupingFnn != NULL)    
    //     delete m_groupingFnn;
    if(m_trafficScheduler != NULL)
        delete m_trafficScheduler;
    
    if(m_sendSocket)
        m_sendSocket->Close();
    if(m_receiveSocket)
        m_receiveSocket->Close();
}

void Intersection::SendPacket()
{
    NS_LOG_DEBUG("Intersection_" << m_ID <<" SendPacket()");
    NS_LOG_DEBUG("At "<<Simulator::Now().GetSeconds()<<" seconds Plt:");
    for(std::list<PltContent>::iterator it = m_plt.begin(); it != m_plt.end(); it++)
        NS_LOG_DEBUG(it->vehicleID);

    //brocast the plt list to all vehicles
    if(m_plt.empty() == false)
    {
        Ptr<Packet> packet = new Packet();
        PltHeader header;
        header.SetData(m_plt);
        packet->AddHeader(header);

        InetSocketAddress brocastAddress(Ipv4Address("255.255.255.255"), Vehicle::receivePort);

        m_sendSocket->SendTo(packet, 0, brocastAddress);
    }
    else
        NS_LOG_DEBUG("m_plt is empty, no packet is sent");    
}

void Intersection::OnReceivePacket(Ptr<Socket> socket)
{
    //extract packet and its content
    Ptr<Packet> packet = socket->Recv();

    SeqTsHeader header;
    uint32_t vehicleID, vehicleLaneID;
    VEHICLE_STATUS vehicleStatus;

    packet->RemoveHeader(header);
    vehicleStatus = (VEHICLE_STATUS)header.GetSeq();
    packet->RemoveHeader(header);
    vehicleLaneID = header.GetSeq();
    packet->RemoveHeader(header);
    vehicleID = header.GetSeq();

    switch(vehicleStatus)
    {
        case WAITING:       //when the vehicle is entering WAITING status, do grouping
        {
            NS_LOG_DEBUG(Simulator::Now().GetSeconds()<<" second intersection_" <<m_ID<<" receive WAITING packet from vehicle_"<<vehicleID<<" lane_"<<vehicleLaneID);

            m_laneFlow[vehicleLaneID]->push_back(vehicleID);
            PrintLane();

            //if there is no group in lane or the vehicle is not permitted to join tailing group, form a new one
            if(m_laneGroup[vehicleLaneID]->empty() == true || IsAbleJoinGroup(vehicleID, vehicleLaneID) == false)  
            {
                NS_LOG_DEBUG("Setting up a new grroup for vehicle_" << vehicleID << " on lane_" << vehicleLaneID);

                GroupInformation newGroup;
                newGroup.members.push_back(vehicleID);
                newGroup.laneID = vehicleLaneID;
                newGroup.expectedArrivalTime = Simulator::Now().GetSeconds() + armLength / Vehicle::speed;
                newGroup.timeStamp[vehicleID] = Simulator::Now().GetSeconds();

                m_laneGroup[vehicleLaneID]->push_back(newGroup);
            }
            else    //the vehicle is going to join the tailing group
            {
                NS_LOG_DEBUG("vehicle_" << vehicleID << " joining group");

                //update information of the tailing group
                std::list<GroupInformation>::reverse_iterator itLastGroup = m_laneGroup[vehicleLaneID]->rbegin();
                itLastGroup->members.push_back(vehicleID);
                itLastGroup->timeStamp[vehicleID] = Simulator::Now().GetSeconds();
            }

            NS_LOG_DEBUG("After grouping: ");
            PrintLane();

            break;
        }
        case EXIT:  //when the vehicle has left intersection region, delete it from plt
        {
            NS_LOG_DEBUG(Simulator::Now().GetSeconds()<<" second intersection_" <<m_ID<<" receive EXIT packet from "<<vehicleID<<" lane_"<<vehicleLaneID<<"\n");

            m_laneFlow[vehicleLaneID]->pop_front();

            for(std::list<PltContent>::iterator it = m_plt.begin(); it != m_plt.end(); it++)
            {
                if(vehicleID == it->vehicleID)
                {
                    m_plt.erase(it);
                    NS_LOG_DEBUG("Remove vehicle_" << vehicleID << " from m_plt");
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
        return true;
    }
    else
    {
        return false;
    }
}

double Intersection::BenefitOfJoiningGroup(uint32_t vehicleID, uint32_t laneID)
{
    NS_LOG_DEBUG("Intersection::BenefitOfJoiningGroup()");

    GroupInformation tailGroup = m_laneGroup[laneID]->back();
    NS_LOG_DEBUG("test");
    double queueLength =  tailGroup.members.size();
    NS_LOG_DEBUG("test1");
    double averageDelay = tailGroup.GetAverageDelay(Simulator::Now().GetSeconds());
    NS_LOG_DEBUG("test2");
    int diffWithConcurrentLane = queueLength - GetConcurrentGroupSize(laneID);

    NS_LOG_DEBUG("queueLength: " << queueLength << " averageDelay: " << averageDelay << " diff: " << diffWithConcurrentLane);

    std::vector<double> fnnInput;
    fnnInput.push_back(queueLength);
    fnnInput.push_back(averageDelay);
    fnnInput.push_back(diffWithConcurrentLane);
    double benefit = m_groupingFnn->Forward(fnnInput); 

    NS_LOG_DEBUG("Benefit of joining group for vehicle_"<<vehicleID<<" : "<<benefit);
    return benefit;
}

uint32_t Intersection::GetConcurrentGroupSize(uint32_t laneID)
{
    std::list<GroupInformation> *groups = NULL;
      
    if(laneID == m_eastWardLaneID)
        groups = m_laneGroup[m_westWardLaneID];
    else if(laneID == m_westWardLaneID)
        groups = m_laneGroup[m_eastWardLaneID];
    else if(laneID == m_southWardLaneID)
        groups = m_laneGroup[m_northWardLaneID];
    else if(laneID == m_northWardLaneID)
        groups = m_laneGroup[m_southWardLaneID];
    else
    {
        NS_LOG_ERROR("Intersection::GetConcurrentGroupSize() invalid laneID: " << laneID);
        return 0;
    }  
    
    if(groups->empty() == true)
        return 0;
    else
        return groups->back().members.size();
}

void Intersection::Schedule()
{
    NS_LOG_DEBUG("Intersection_" << m_ID <<" Schedule()");
    if(m_plt.empty() == true)
    {
        ConstructPlt();
        SendPacket();
    }
    Simulator::Schedule(Seconds(SIMULATION_STEP), &Intersection::Schedule, this);
}

void Intersection::ConstructPlt()
{
    NS_LOG_DEBUG("Intersection_" << m_ID << " ConstructPlt()");

    Job job = m_trafficScheduler->GetNextScheduleJob(*this);

    m_plt.clear();
    GroupInformation *group = job.group1;
    if(group != NULL)
    {
        for(uint32_t i = 0; i < group->members.size(); i++)
        {
            PltContent content(group->members[i]);
            m_plt.push_back(content);
        }
        //delete group information from m_laneGroup since it has been scheduled
        m_laneGroup[group->laneID]->pop_front();
    }

    group = job.group2;
    if(group != NULL)
    {
        for(uint32_t i = 0; i < group->members.size(); i++)
        {
            PltContent content(group->members[i]);
            m_plt.push_back(content);
        }
        m_laneGroup[group->laneID]->pop_front();
    }
}

void Intersection::PrintLane()
{
    fprintf(stderr, "Printing lane information of intersection_%u:\n", m_ID);
    for(DIRECTION direction = EASTWARD; direction <= NORTHWARD; direction = DIRECTION((uint32_t)direction + 1))
    {
        uint32_t laneID = GetLaneID(direction);
        std::list<GroupInformation> *laneGroup = m_laneGroup[laneID];

        fprintf(stderr, "lane_%u: ", laneID);
        for(std::list<GroupInformation>::iterator it = laneGroup->begin(); it != laneGroup->end(); it++)
        {
            fprintf(stderr, "(");
            for(uint32_t i = 0; i < it->members.size(); i++)
                fprintf(stderr, " %u,", it->members[i]);
            fprintf(stderr, " )");
        }
        fprintf(stderr, "\n");
    }
}