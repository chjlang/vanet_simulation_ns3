#ifndef _PUBLIC_H_
#define _PUBLIC_H_

#include <list>
##include <string>

enum DIRECTION {EASTWARD, SOUTHWARD, WESTWARD, NORTHWARD};
enum VEHICLE_STATUS {IDLE, WAITING, PASSING, EXIT};

const double MAX_X_COORDINATE = 1000;		//maximum x coordinate of the simulated region
const double MAX_Y_COORDINATE = 1000;		//maximum y coordinate of the simulated region

const std::string GROUPING_FIS_FILE = "/home/cjl...";
const double SIMULATION_STEP = 1;

extern std::map<int, std::list<MobilityModel> > g_laneMapping;
extern std::list<Ptr<Vehicle> > g_vehiclePool;

#endif