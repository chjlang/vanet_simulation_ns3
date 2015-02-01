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

using namespace ns3;

const double VEHICLE_START_TIME = 2;
const double INTERSECTION_START_TIME = 1;
const uint16_t MAXIMUN_NUM_VEHICLE = 500;
const uint16_t NUMBER_INTERSECTIONS = 1;

std::list<Ptr<Vehicle> > g_vehiclePool;					//record all free vehicles, used in generating traffic

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
		LogComponentEnable("VehcileLog", LOG_DEBUG);
		LogComponentEnable("IntersectionLog", LOG_DEBUG);
		LogComponentEnable("SchedulerLog", LOG_DEBUG);
	}
	else
	{
		LogComponentEnable("VehcileLog", LOG_ERROR);
		LogComponentEnable("IntersectionLog", LOG_ERROR);
		LogComponentEnable("SchedulerLog", LOG_ERROR);
	}

	/***** setup 802.11p service *****/
	NodeCotainer vehicleNodes, intersectionNodes;
	vehicleNodes.Create(MAXIMUN_NUM_VEHICLE);
	intersectionNodes.Create(NUMBER_INTERSECTIONS);

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
	wifiPhy.SetChannel (wifiChannel.Create ());
	NqosWave80211pMacHelper wifi80211pMac = NqosWaveMacHelper::Default();
	Wifi80211pHelper 80211pHelper = Wifi80211pHelper::Default ();
	NetDeviceCotainer vehicleDevices = 80211pHelper.Install (wifiPhy, wifi80211pMac, vehicleNodes);
	NetDeviceCotainer intersectionDevices = 80211pHelper.Install(wifiPhy, wifi80211pMac, intersectionNodes);

	/*****setup mobility model and topology of the road network*****/
	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
	mobility.install(vehicleNodes);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.install(intersectionNodes);

	/***** setup IP address *****/
	InternetStackHelper internet;
	internet.Install(nodes);
	Ipv4AddressHelper ipv4;
	ipv4.SetBase("10.1.1.0", "255.255.200.0");
	Ipv4InterfaceContainer intersectionAddress = ipv4.Assign(IntersectionDevices);
	Ipv4InterfaceContainer vehicleAddress = ipv4.Assign(vehicleDevices);

	for(uint32_t i = 0; i < MAXIMUN_NUM_VEHICLE; i++)
	{
		Ptr<Vehicle> vehicleApp = CreateObject<Vehicle>(i, vehicleAddress.GetAddress(i));
		g_vehiclePool.push_back(vehicleApp);
		
		vehicleApp->SetStartTime(Seconds(VEHICLE_START_TIME));
		vehicleApp->SetStopTime(Seconds(runTime));
		vehicleNodes.Get(i)->AddApplication(vehicleApp);
	}

	for(uint32_t i = 0; i < NUMBER_INTERSECTIONS; i++)
	{
		Ptr<Intersection> intersectionApp = CreateObject<Intersection>();
		intersectionApp->Configure(i, intersectionAddress.GetAddress(i), Vector(100, 100, 0), arrivalRate, arrivalRate, arrivalRate, arrivalRate);
		intersectionApp->SetStartTime(INTERSECTION_START_TIME);
		intersectionApp->SetStopTime(runTime);
		intersectionNodes.Get(i)->AddApplication(intersectionApp);
	}

	Simulator::Stop(Seconds(runTime + 1));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}