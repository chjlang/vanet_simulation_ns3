#include "traffic_scheduler.h"
#include "intersection.h"
#include "vehicle.h"

#include <math.h>

NS_LOG_COMPONENT_DEFINE("SchedulerLog");

double Job::TimeToCrossIntersection()
{
	int jobSize = group1->members.size() > group2->members.size() ? group1->members.size() : group2->members.size();
	double distance = Intersection::size + (jobSize - 1) * Vehicle::minimumHeadway;
	return distance / Vehicle::speed;
}

Job::Job(GroupInformation *_group1, GroupInformation *_group2, double _expectArrivalTime)
{
	group1 = _group1;
	group2 = _group2;
	expectArrivalTime = _expectArrivalTime;
}

void TrafficScheduler::ConvertIntoInflow(Intersection &intersection)
{
	MergeToInflow(inflow1, intersection.m_laneGroup[intersection.m_eastWardLaneID], intersection.m_laneGroup[intersection.m_westWardLaneID]);
	MergeToInflow(inflow2, intersection.m_laneGroup[intersection.m_southWardLaneID], intersection.m_laneGroup[intersection.m_northWardLaneID]);
}

void TrafficScheduler::MergeToInflow(std::vector<Job> &inflow, std::list<GroupInformation> *groups1, std::list<GroupInformation> *groups2)
{
	std::list<GroupInformation>::iterator it1 = groups1->begin(), it2 = groups2->begin();
	while(it1 != groups1->end() && it2 != groups2->end())
	{
		double expectedArrivalTime = (it1->expectedArrivalTime > it2->expectedArrivalTime ? it1->expectedArrivalTime : it2->expectedArrivalTime);
		inflow.push_back(Job(&(*it1), &(*it2), expectedArrivalTime));
		it1++;
		it2++;
	}
	while(it1 != groups1->end())
	{
		inflow.push_back(Job(&(*it1), NULL, it1->expectedArrivalTime));
		it1++;
	}
	while(it2 != groups2->end())
	{
		inflow.push_back(Job(NULL, &(*it2), it2->expectedArrivalTime));
		it2++;
	}	
}

//calculate the next scheduler status given the old status and next phase index
SchedulerStatus TrafficScheduler::CalculateStatus(const SchedulerStatus& oldStatus, int phaseIndex)	
{
	SchedulerStatus newStatus;
	Job job;
	if(phaseIndex == 1)
		job = inflow1[oldStatus.xs.x1];
	else
		job = inflow2[oldStatus.xs.x2];

	if(phaseIndex == oldStatus.xs.theLastPhase)
		newStatus.finishTime = oldStatus.finishTime + job.TimeToCrossIntersection();
	else
		newStatus.finishTime = oldStatus.finishTime + job.TimeToCrossIntersection() + Vehicle::startUpLostTime;

	newStatus.cumulativeDelay = oldStatus.cumulativeDelay + job.group1->GetTotalDelay(newStatus.finishTime) + job.group2->GetTotalDelay(newStatus.finishTime);
	
	return newStatus;
}

void TrafficScheduler::Schedule(Intersection& intersection)
{
	ConvertIntoInflow(intersection);

	m_status.clear();
	SchedulerStatus initialStatus;
	m_status.insert(make_pair(initialStatus.xs, initialStatus));

	int numOfJobs = inflow1.size() + inflow2.size();

	for(int i = 1; i <= numOfJobs; i++)
	{
		std::vector<SchedulerStatus::XS> setOfx = GetCombination(i);
		for(int j = 0; j < setOfx.size(); j++)
		{
			if(setOfx[j].x1 > 0)
			{
				setOfx[j].theLastPhase = 1;
				ForwardRecusion(setOfx[j]);
			}
			if(setOfx[j].x2 > 0)
			{
				setOfx[j].theLastPhase = 2;
				ForwardRecusion(setOfx[j]);
			}
		}
	}
	RetrieveSolution();
}

std::vector<SchedulerStatus::XS> TrafficScheduler::GetCombination(int sum)
{
	std::vector<SchedulerStatus::XS> result;
	for(int i = 0; i <= sum; i++)
	{
		for(int j = 0; j <=sum; j++)
		{
			if(i + j == sum)
				result.push_back(SchedulerStatus::XS(i, j));
		}
	}
	return result;
}

void TrafficScheduler::ForwardRecusion(SchedulerStatus::XS statusIndex)
{
	double minDelay = 10000000;

	SchedulerStatus::XS prevXS(statusIndex);
	if(statusIndex.theLastPhase = 1)
		prevXS.x1--;
	else
		prevXS.x2--;

	prevXS.theLastPhase = 1;
	if(m_status.find(prevXS) != m_status.end())
	{
		SchedulerStatus newStatus = CalculateStatus(m_status[prevXS], 1);
		if(newStatus.cumulativeDelay < minDelay)
		{
			minDelay = newStatus.cumulativeDelay;
			newStatus.prevPhaseIndex = 1;
			m_status[statusIndex] = newStatus;
		}
	}

	prevXS.theLastPhase = 2;
	if(m_status.find(prevXS) != m_status.end())
	{
		SchedulerStatus newStatus = CalculateStatus(m_status[prevXS], 2);
		if(newStatus.cumulativeDelay < minDelay)
		{
			minDelay = newStatus.cumulativeDelay;
			newStatus.prevPhaseIndex = 2;
			m_status[statusIndex] = newStatus;
		}
	}
}

void TrafficScheduler::RetrieveSolution()
{
	scheduleFlow.clear();
	SchedulerStatus::XS fullX1(inflow1.size(), inflow2.size(), 1), fullX2(inflow1.size(), inflow2.size(), 2);

	SchedulerStatus::XS trace;
	if(m_status[fullX1].cumulativeDelay < m_status[fullX2].cumulativeDelay)
		trace = fullX1;
	else
		trace = fullX2;

	for(int i = inflow1.size() + inflow2.size(); i >= 1; i--)
	{
		if(trace.theLastPhase == 1)
		{
			scheduleFlow.push_front(inflow1.back());
			inflow1.pop_back();
		}
		else
		{
			scheduleFlow.push_front(inflow2.back());
			inflow2.pop_back();
		}

		int prevIndex = m_status[trace].prevPhaseIndex;
		if(trace.theLastPhase == 1)
			trace.x1--;
		else
			trace.x2--;
		trace.theLastPhase = prevIndex;
	}
}

Job TrafficScheduler::GetNextScheduleJob(Intersection& intersection)
{
	if(scheduleFlow.empty() == true)
		Schedule(intersection);

	Job nextJob = scheduleFlow.front();
	scheduleFlow.pop_front();

	return nextJob;
}