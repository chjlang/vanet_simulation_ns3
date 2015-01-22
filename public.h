#ifndef _PUBLIC_H_
#define _PUBLIC_H_

#include <list>

enum DIRECTION {EASTWARD, SOUTHWARD, WESTWARD, NORTHWARD};

const double MAX_X_COORDINATE = 1000;		//maximum x coordinate of the simulated region
const double MAX_Y_COORDINATE = 1000;		//maximum y coordinate of the simulated region


extern std::map<int, std::list<MobilityModel> > g_laneMapping;
extern std::queue<Ptr<Vehicle> > g_vehiclePool;

#endif