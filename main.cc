#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/timer.h"
#include "ns3/socket-factory.h"
#include "ns3/mobility-module.h"
#include "ns3/nstime.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-address.h"
#include "ns3/wifi-module.h"
#include "src/core/model/timer.h"
#include "src/network/utils/queue.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/config-store-module.h"

#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include <list>

#include "vehicle.h"
#include "intersection.h"
#include "topology.h"

using namespace ns3;

const double VEHICLE_START_TIME = 1;
const double INTERSECTION_START_TIME = 2;
const uint16_t MAXIMUN_NUM_VEHICLE = 10;
const uint16_t NUMBER_INTERSECTIONS = 1;

std::list<Ptr<Vehicle> > g_vehiclePool;					//record all free vehicles, used in generating traffic
std::map<uint32_t, Ptr<Vehicle> > g_vehicleMapping;

int main(int argc, char* argv[])
{
	/*****setup command line parameters*****/
	uint32_t runTime = 300;		//runing time of this simulation
	double arrivalRate = 5;		//average number of vehicles generated for each lane
	bool verbose = false;		//flag of showing verbose information

	CommandLine cmd;
	cmd.AddValue("runTime", "running time", runTime);
	cmd.AddValue("arrivalRate", "average number of vehicles generated for each lane", arrivalRate);
	cmd.AddValue("verbose", "flag of showing verbose information (0--off, 1--on)", verbose);
	cmd.Parse(argc, argv);

	/*****setup logging level*****/
	if(verbose)
	{
		LogComponentEnable("VehicleLog", LOG_LEVEL_DEBUG);
		LogComponentEnable("IntersectionLog", LOG_LEVEL_DEBUG);
		LogComponentEnable("SchedulerLog", LOG_LEVEL_DEBUG);
		LogComponentEnable("TopologyLog", LOG_LEVEL_DEBUG);
	}
	else
	{
		LogComponentEnable("VehicleLog", LOG_LEVEL_ERROR);
		LogComponentEnable("IntersectionLog", LOG_LEVEL_ERROR);
		LogComponentEnable("SchedulerLog", LOG_LEVEL_ERROR);
		LogComponentEnable("TopologyLog", LOG_LEVEL_ERROR);
	}

	/***** setup 802.11p service *****/
	NodeContainer vehicleNodes, intersectionNodes;
	vehicleNodes.Create(MAXIMUN_NUM_VEHICLE);
	intersectionNodes.Create(NUMBER_INTERSECTIONS);

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	wifiPhy.SetChannel (wifiChannel.Create ());
	NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default();
	Wifi80211pHelper wifi80211pHelper = Wifi80211pHelper::Default ();
	NetDeviceContainer vehicleDevices = wifi80211pHelper.Install (wifiPhy, wifi80211pMac, vehicleNodes);
	NetDeviceContainer intersectionDevices = wifi80211pHelper.Install(wifiPhy, wifi80211pMac, intersectionNodes);

	/*****setup mobility model and topology of the road network*****/
	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
	mobility.Install(vehicleNodes);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(intersectionNodes);

	/***** setup IP address *****/
	InternetStackHelper internet;
	internet.Install(vehicleNodes);
	internet.Install(intersectionNodes);
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.0.0", "255.255.0.0");
	Ipv4InterfaceContainer intersectionAddress = ipv4.Assign(intersectionDevices);
	Ipv4InterfaceContainer vehicleAddress = ipv4.Assign(vehicleDevices);

	for(uint32_t i = 0; i < MAXIMUN_NUM_VEHICLE; i++)
	{
		Ptr<Vehicle> vehicleApp = CreateObject<Vehicle>(i, vehicleAddress.GetAddress(i));
		g_vehiclePool.push_back(vehicleApp);
		g_vehicleMapping[i] = vehicleApp;

		vehicleNodes.Get(i)->AddApplication(vehicleApp);
		vehicleApp->SetStartTime(Seconds(VEHICLE_START_TIME));
		vehicleApp->SetStopTime(Seconds(runTime));
	}

	for(uint32_t i = 0; i < NUMBER_INTERSECTIONS; i++)
	{
		Ptr<Intersection> intersectionApp = CreateObject<Intersection>();
		intersectionNodes.Get(i)->AddApplication(intersectionApp);
		intersectionApp->Configure(i, intersectionAddress.GetAddress(i), Vector(100, 100, 0), arrivalRate, arrivalRate, arrivalRate, arrivalRate);
		intersectionApp->SetStartTime(Seconds(INTERSECTION_START_TIME));
		intersectionApp->SetStopTime(Seconds(runTime));

		Topology::GetInstance()->ConnectIntersections(intersectionApp, NULL, HORIZONTAL);	//temporally added for testing isolated intersection
								 															//should be replaced by function CreateTopology() for multiple intersections
	}

	Simulator::Stop(Seconds(runTime + 1));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}