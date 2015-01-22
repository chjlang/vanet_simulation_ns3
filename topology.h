#ifndef _TOPOLOGY_H_
#define _TOPOLOGY_H_

#include <set>
#include <map>
#include "ns/core-module.h"
#include "intersection.h"

enum Pattern { HORIZONTAL, VERTICAL };

class Topology
{
public:
	
	static Topology* GetInstance()
	{
		if(ptr == NULL)
		{
			ptr = new Topology();
			return ptr;
		}
		else
			return ptr;
	}

	int GetUpstream(int laneID)	{ return m_upstream[laneID]; }
	int GetDownstream(int laneID) { return m_downstream[laneID]; }
	int GetIntersectionFromLane(int laneID);
	Ptr<Intersection> GetIntersection(int ID);

	void ConnectInteresctions(Ptr<Intersection> intersection_1, Ptr<Intersection> intersection_2, Pattern pattern);

private:
	Topology() {}

	static Topology *ptr;

	std::set<Ptr<Intersection> > m_intersections;
	std::map<int, int> m_upstream, m_downstream;
};

#endif