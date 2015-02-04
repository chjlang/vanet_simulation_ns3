#ifndef _TOPOLOGY_H_
#define _TOPOLOGY_H_

#include <set>
#include <map>
#include "ns3/core-module.h"
#include "intersection.h"

enum Pattern { HORIZONTAL, VERTICAL };

class Topology
{
public:
	
	static Topology* GetInstance()
	{
		if(Topology::ptr == NULL)
		{
			Topology::ptr = new Topology();
			return Topology::ptr;
		}
		else
			return Topology::ptr;
	}

	uint32_t GetUpstream(uint32_t laneID)	{ return m_upstream[laneID]; }
	uint32_t GetDownstream(uint32_t laneID) { return m_downstream[laneID]; }
	uint32_t GetIntersectionIDFromLane(uint32_t laneID);
	Ptr<Intersection> GetIntersection(uint32_t ID);

	void ConnectIntersections(Ptr<Intersection> intersection_1, Ptr<Intersection> intersection_2, Pattern pattern);

private:
	Topology() {}

	static Topology *ptr;

	std::set<Ptr<Intersection> > m_intersections;
	std::map<uint32_t, uint32_t> m_upstream, m_downstream;
};

#endif