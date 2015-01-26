#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "intersection.h"

struct Job
{
	GroupInformation *group1, *group2;
	Job()
};

class Scheduler
{
public:
	Scheduler();
	~Scheduler();

	void Convert2Inflow();	//convert vehicles of lanes into vehicles of inflows
							//the east and west lanes forms inflow1, the rest is inflow2

private: 
	std::queue<Job> inflow1, inflow2;
};

#endif