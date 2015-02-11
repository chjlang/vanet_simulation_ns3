#ifndef _TRAFFIC_SCHEDULER_H_
#define _TRAFFIC_SCHEDULER_H_

#include <vector>
#include <list>
#include <map>

#include "public.h"

class Intersection;

struct Job
{
	GroupInformation *group1, *group2;
	double expectArrivalTime;

	double TimeToCrossIntersection();
	Job(GroupInformation *_group1 = NULL, GroupInformation *_group2 = NULL, double _expectArrivalTime = 0);
};

struct SchedulerStatus
{
	struct XS
	{
		uint32_t x1, x2;		//each element xi counts the number of groups that had been serviced in phase i
		uint32_t theLastPhase;

		XS(uint32_t _x1 = 0, uint32_t _x2= 0, uint32_t _theLastPhase = 0) 
		{ 
			x1 = _x1;
			x2 = _x2;
			theLastPhase = _theLastPhase; 
		}
		
		XS(const XS& other)
		{
			x1 = other.x1;
			x2 = other.x2;
			theLastPhase = other.theLastPhase;
		}

		bool operator < (const XS &other) const
		{
			if(x1 < other.x1)
				return true;
			else if (x1 > other.x1)
				return false;
			else
			{
				if(x2 < other.x2)
					return true;
				else if (x2 > other.x2)
					return false;
				else
					return theLastPhase < other.theLastPhase;
			}
		}
	} xs;

	double finishTime;		
	double cumulativeDelay;
	uint32_t prevPhaseIndex;

	SchedulerStatus()
	{
		xs.x1 = xs.x2 = 0;
		xs.theLastPhase = 0;
		finishTime = cumulativeDelay = prevPhaseIndex = 0;
	}
};

class TrafficScheduler
{
public:
	TrafficScheduler(){}
	~TrafficScheduler(){}


	Job GetNextScheduleJob(Intersection& intersection);	//return the next job in scheduleFlow
																//if scheduleFlow is empty, this function call Schedule to contruct a new scheduleFlow
	void ConvertIntoInflow(Intersection &intersection);	//convert vehicles of lanes into vehicles of inflows
																//the east and west lanes forms inflow1, the rest is inflow2 
private:

	//merge groups in "groups1" & "groups2" into "inflow"
	void MergeToInflow(std::vector<Job> &inflow, std::list<GroupInformation> *groups1, std::list<GroupInformation> *groups2);

	//scheduling algorithm
	void Schedule(Intersection& intersection);

	//calculate and return the next scheduler status given the old status and next phase index
	SchedulerStatus CalculateStatus(const SchedulerStatus& oldStatus, uint32_t phaseIndex);	

	//a naive way to collect all possible state groups
	std::vector<SchedulerStatus::XS> GetCombination(uint32_t sum);

	//update state using forward recursion technique (refer to algorithm3)
	void ForwardRecusion(SchedulerStatus::XS statusIndex);

	//retrieve solution and get scheduleFlow
	void RetrieveSolution();

	std::vector<Job> inflow1, inflow2;

	std::map<SchedulerStatus::XS, SchedulerStatus> m_status;

	std::list<Job> scheduleFlow;
};

#endif