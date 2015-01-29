#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "intersection.h"

struct Job
{
	GroupInformation *group1, *group2;
	double expectArrivalTime;

	double TimeToCrossIntersection()
	{
		int jobSize = group1->members.size() > group2->members.size() ? group1->members.size() : group2->members.size();
		double distance = Intersection::size + (jobSize - 1) * Vehicle::minimumHeadway;
		return distance / Vehicle::speed;
	}
	Job(GroupInformation *_group1, GroupInformation *_group2, double _expectArrivalTime)
	{
		group1 = _group1;
		group2 = _group2;
		expectArrivalTime = _expectArrivalTime;
	}
};

struct SchedulerStatus
{
	struct XS
	{
		int x1, x2;		//each element xi counts the number of groups that had been serviced in phase i
		int theLastPhase;

		XS(int _x1 = 0, int _x2= 0, int _theLastPhase = 0) 
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

		bool opertor < (const XS &other) const
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
	int prevPhaseIndex;

	SchedulerStatus()
	{
		xs.x1 = xs.x2 = 0;
		xs.theLastPhase = 0;
		finishTime = cumulativeDelay = prevPhaseIndex = 0;
	}
};

class Scheduler
{
public:
	Scheduler();
	~Scheduler();

	void ConvertIntoInflow(const Intersection &intersection);	//convert vehicles of lanes into vehicles of inflows
																//the east and west lanes forms inflow1, the rest is inflow2

	void Schedule();

private: 
	//merge groups in "groups1" & "groups2" into "inflow"
	void MergeToInflow(std::vector<Job> &inflow, std::list<GroupInformation> *groups1, std::list<GroupInformation> *groups2);

	//calculate and return the next scheduler status given the old status and next phase index
	SchedulerStatus CalculateStatus(SchedulerStatus oldStatus, int phaseIndex);	

	//a naive way to collect all possible state groups
	std::vector<SchedulerStatus::XS> GetCombination(int sum);

	//update state using forward recursion technique (refer to algorithm3)
	void ForwardRecusion(SchedulerStatus::XS statusIndex);

	//retrieve solution and get scheduleFlow
	void RetrieveSolution();

	std::vector<Job> inflow1, inflow2;

	std::map<SchedulerStatus::XS, SchedulerStatus> m_status;

	std::list<Job> scheduleFlow;
};

#endif