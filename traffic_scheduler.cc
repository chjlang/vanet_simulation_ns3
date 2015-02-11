#include "traffic_scheduler.h"
#include "intersection.h"
#include "vehicle.h"

#include <math.h>

NS_LOG_COMPONENT_DEFINE("SchedulerLog");

double Job::TimeToCrossIntersection()
{
	uint32_t jobSize;
	if(group1 != NULL && group2 != NULL)
		jobSize = group1->members.size() > group2->members.size() ? group1->members.size() : group2->members.size();
	else if(group1 != NULL)
		jobSize = group1->members.size();
	else
		jobSize = group2->members.size();

	if(jobSize > 0)
	{
		double distance = Intersection::size + (jobSize - 1) * Vehicle::minimumHeadway;
		return distance / Vehicle::speed;
	}
	else
		return 0;
}

Job::Job(GroupInformation *_group1, GroupInformation *_group2, double _expectArrivalTime)
{
	group1 = _group1;
	group2 = _group2;
	expectArrivalTime = _expectArrivalTime;
}

void TrafficScheduler::ConvertIntoInflow(Intersection &intersection)
{
	inflow1.clear();
	MergeToInflow(inflow1, intersection.m_laneGroup[intersection.m_eastWardLaneID], intersection.m_laneGroup[intersection.m_westWardLaneID]);

	inflow2.clear();
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

void TrafficScheduler::Schedule(Intersection& intersection)
{
	NS_LOG_DEBUG("TrafficScheduler::Schedule()");

	ConvertIntoInflow(intersection);

	m_status.clear();
	SchedulerStatus initialStatus;
	initialStatus.finishTime = Simulator::Now().GetSeconds();
	initialStatus.xs.theLastPhase = 1;
	m_status.insert(make_pair(initialStatus.xs, initialStatus));
	initialStatus.xs.theLastPhase = 2;
	m_status.insert(make_pair(initialStatus.xs, initialStatus));

	uint32_t numOfJobs = inflow1.size() + inflow2.size();
	NS_LOG_DEBUG("TrafficScheduler::Schedule() of intersection_" << intersection.GetID() << " number of jobs: " << numOfJobs);

	for(uint32_t i = 1; i <= numOfJobs; i++)
	{
		std::vector<SchedulerStatus::XS> setOfx = GetCombination(i);

		for(uint32_t j = 0; j < setOfx.size(); j++)
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

std::vector<SchedulerStatus::XS> TrafficScheduler::GetCombination(uint32_t sum)
{
	std::vector<SchedulerStatus::XS> result;
	for(uint32_t i = 0; i <= inflow1.size(); i++)
	{
		for(uint32_t j = 0; j <= inflow2.size(); j++)
		{
			if(i + j == sum)
				result.push_back(SchedulerStatus::XS(i, j));
		}
	}
	return result;
}

void TrafficScheduler::ForwardRecusion(SchedulerStatus::XS statusIndex)
{
	NS_LOG_DEBUG("TrafficScheduler::ForwardRecusion() statusIndex: " << statusIndex.x1 << " " << statusIndex.x2 << " " << statusIndex.theLastPhase);

	double minDelay = 10000000;

	SchedulerStatus::XS prevXS(statusIndex);
	if(statusIndex.theLastPhase == 1)
		prevXS.x1--;
	else
		prevXS.x2--;

	prevXS.theLastPhase = 1;
	if(m_status.find(prevXS) != m_status.end())
	{
		SchedulerStatus newStatus = CalculateStatus(m_status[prevXS], statusIndex.theLastPhase);
		if(newStatus.cumulativeDelay < minDelay)
		{
			minDelay = newStatus.cumulativeDelay;
			newStatus.prevPhaseIndex = 1;
			newStatus.xs = statusIndex;
			m_status[statusIndex] = newStatus;
		}
	}

	prevXS.theLastPhase = 2;
	if(m_status.find(prevXS) != m_status.end())
	{
		SchedulerStatus newStatus = CalculateStatus(m_status[prevXS], statusIndex.theLastPhase);
		if(newStatus.cumulativeDelay < minDelay)
		{
			minDelay = newStatus.cumulativeDelay;
			newStatus.prevPhaseIndex = 2;
			newStatus.xs = statusIndex;
			m_status[statusIndex] = newStatus;
		}
	}
	NS_LOG_DEBUG("minDelay: " << minDelay);
}

//calculate the next scheduler status given the old status and next phase index
SchedulerStatus TrafficScheduler::CalculateStatus(const SchedulerStatus& oldStatus, uint32_t phaseIndex)	
{
	NS_LOG_DEBUG("TrafficScheduler::CalculateStatus() oldStatus: " << oldStatus.xs.x1 << " " << oldStatus.xs.x2 << " " << oldStatus.xs.theLastPhase << " phaseIndex: " << phaseIndex);

	SchedulerStatus newStatus;
	Job job;
	if(phaseIndex == 1)
		job = inflow1[oldStatus.xs.x1];
	else if(phaseIndex == 2)
		job = inflow2[oldStatus.xs.x2];

	if(phaseIndex == oldStatus.xs.theLastPhase)
		newStatus.finishTime = oldStatus.finishTime + job.TimeToCrossIntersection();
	else
		newStatus.finishTime = oldStatus.finishTime + job.TimeToCrossIntersection() + Vehicle::startUpLostTime;

	newStatus.cumulativeDelay = oldStatus.cumulativeDelay;
	if(job.group1 != NULL)
		newStatus.cumulativeDelay += job.group1->GetTotalDelay(newStatus.finishTime);
	if(job.group2 != NULL)
		newStatus.cumulativeDelay += job.group2->GetTotalDelay(newStatus.finishTime);
	
	return newStatus;
}

void TrafficScheduler::RetrieveSolution()
{
	scheduleFlow.clear();
	SchedulerStatus::XS fullX1(inflow1.size(), inflow2.size(), 1), fullX2(inflow1.size(), inflow2.size(), 2);

	double minDelay = 1000000;
	SchedulerStatus::XS trace;
	if(m_status.find(fullX1) != m_status.end() && m_status[fullX1].cumulativeDelay < minDelay)
	{
		minDelay = m_status[fullX1].cumulativeDelay;
		trace = fullX1;
	}
	if(m_status.find(fullX2) != m_status.end() && m_status[fullX2].cumulativeDelay < minDelay)
	{
		minDelay = m_status[fullX2].cumulativeDelay;
		trace = fullX2;
	}
	NS_LOG_DEBUG("TrafficScheduler::RetrieveSolution() minimum delay: " << minDelay);

	for(uint32_t i = inflow1.size() + inflow2.size(); i >= 1; i--)
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

		uint32_t prevIndex = m_status[trace].prevPhaseIndex;
		if(trace.theLastPhase == 1)
			trace.x1--;
		else
			trace.x2--;
		trace.theLastPhase = prevIndex;
	}
}

Job TrafficScheduler::GetNextScheduleJob(Intersection& intersection)
{
	NS_LOG_DEBUG("TrafficScheduler::GetNextScheduleJob()");

	if(scheduleFlow.empty() == true)
		Schedule(intersection);

	if(scheduleFlow.empty() == false)
	{
		Job nextJob = scheduleFlow.front();
		scheduleFlow.pop_front();
		return nextJob;
	}
	else	//if Schedule() did not get a schedule flow (because there is no vehicle in inflow)
	{		//return Job(NULL, NULL, 0), the calling funciton should check the return
		return Job(NULL, NULL, 0);
	}		
}