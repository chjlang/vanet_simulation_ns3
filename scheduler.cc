#include "scheduler.h"

void Scheduler::ConvertIntoInflow(const Intersection &intersection)
{
	MergeToInflow(inflow1, intersection.m_laneGroup[EASTWARD], intersection.m_laneGroup[WESTWARD]);
	MergeToInflow(inflow2, intersection.m_laneGroup[SOUTHWARD], intersection.m_laneGroup[NORTHWARD]);
}

void Scheduler::MergeToInflow(std::vector<Job> &inflow, std::list<GroupInformation> *groups1, std::list<GroupInformation> *groups2)
{
	std::list::iterator it1 = groups1.begin(), it2 = groups2.begin();
	while(it1 != groups1.end() && it2 != groups2.end())
	{
		double expectedArrivalTime = (it1->expectedArrivalTime > it2->expectedArrivalTime ? it1->expectedArrivalTime : it2->expectedArrivalTime);
		inflow.push_back(Job(it1, it2, expectedArrivalTime));
		it1++;
		it2++;
	}
	while(it1 != groups1.end())
	{
		inflow.push_back(Job(it1, NULL, it1->expectedArrivalTime));
		it1++;
	}
	while(it2 != groups2.end())
	{
		inflow.push_back(Job(NULL, it2, it2->expectedArrivalTime));
		it2++;
	}	
}

//calculate the next scheduler status given the old status and next phase index
SchedulerStatus Scheduler::CalculateStatus(SchedulerStatus oldStatus, int phaseIndex);	
{
	SchedulerStatus newStatus;
	Job job;
	if(phaseIndex == 1)
		job = oldStatus.inflow1[x1];
	else
		job = oldStatus.inflow2[x2];

	if(phaseIndex == oldStatus.xs.theLastPhase)
		newStatus.finishTime = oldStatus.finishTime + job.TimeToCrossIntersection();
	else
		newStatus.finishTime = oldStatus.finishTime + job.TimeToCrossIntersection() + Vehicle::startUpLostTime;

	newStatus.cumulativeDelay = oldStatus.cumulativeDelay + job.group1->GetTotalDelay(newStatus.finishTime) + job.group2->GetTotalDelay(newStatus.finishTime);
	
	return newStatus;
}

void Scheduler::Schedule()
{
	m_status.clear();
	SchedulerStatus initialStatus;
	m_status.inset(make_pair(initialStatus.xs, initialStatus));

	int numOfJobs = inflow1.size() + inflow2.size();

	for(int i = 1; i <= numOfJobs; i++)
	{
		std::vector<SchedulerStatus::XS> setOfx = GetCombination(i);
		for(int j = 0; j < setOfx.size(); j++)
		{
			if(setOfx[j].x1 > 0)
			{
				setOfx[j].theLastPhase = 1;
				StateUpdate(setOfx[j]);
			}
			if(setOfx[j].x2 > 0)
			{
				setOfx[j].theLastPhase = 2;
				StateUpdate(setOfx[j]);
			}
		}
	}
	RetrieveSolution();
}

std::vector<SchedulerStatus::XS> Scheduler::GetCombination(int sum)
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

void Scheduler::ForwardRecusion(SchedulerStatus::XS statusIndex)
{
	double minDelay = 1000000;

	SchedulerStatus::XS prevXS(statusIndex);
	if(statusIndex.xs.theLastPhase = 1)
		prevXS.xs.x1--;
	else
		prevXS.xs.x2--;

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

void Scheduler::RetrieveSolution()
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
		scheduleFlow.push_front(trace.xs.theLastPhase);

		int prevIndex = m_status[trace].prevPhaseIndex;
		if(trace.xs.theLastPhase == 1)
			trace.xs.x1--;
		else
			trace.x2.x2--;
		trace.xs.theLastPhase = prevIndex;
	}
}