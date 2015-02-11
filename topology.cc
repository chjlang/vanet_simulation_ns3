#include "topology.h"

NS_LOG_COMPONENT_DEFINE("TopologyLog");

Topology* Topology::ptr = NULL;

//setup connections between two intersections, the upstream/downstream is set to -1 if there is no upstream/downstream available 
void Topology::ConnectIntersections(Ptr<Intersection> intersection_1, Ptr<Intersection> intersection_2, Pattern pattern)
{
	m_intersections[intersection_1->GetID()] = intersection_1;

	if(intersection_2 != NULL)
	{
		m_intersections[intersection_2->GetID()] = intersection_2;

		if(pattern == HORIZONTAL)	//intersection_1 is on the left-hand side of intersection_2
		{
			m_upstream[intersection_1->GetLaneID(WESTWARD)] = intersection_2->GetLaneID(WESTWARD);
			m_downstream[intersection_2->GetLaneID(WESTWARD)] = intersection_1->GetLaneID(WESTWARD);

			m_upstream[intersection_2->GetLaneID(EASTWARD)] = intersection_1->GetLaneID(EASTWARD);
			m_downstream[intersection_1->GetLaneID(EASTWARD)] = intersection_2->GetLaneID(EASTWARD);

			m_downstream[intersection_1->GetLaneID(WESTWARD)] = m_downstream[intersection_2->GetLaneID(EASTWARD)] = 0;
			m_upstream[intersection_1->GetLaneID(EASTWARD)] = m_upstream[intersection_2->GetLaneID(WESTWARD)] = 0;
			
			m_upstream[intersection_1->GetLaneID(NORTHWARD)] = m_downstream[intersection_1->GetLaneID(NORTHWARD)] = 0;
			m_upstream[intersection_1->GetLaneID(SOUTHWARD)] = m_downstream[intersection_1->GetLaneID(SOUTHWARD)] = 0;

			m_upstream[intersection_2->GetLaneID(NORTHWARD)] = m_downstream[intersection_2->GetLaneID(NORTHWARD)] = 0;
			m_upstream[intersection_2->GetLaneID(SOUTHWARD)] = m_downstream[intersection_2->GetLaneID(SOUTHWARD)] = 0;
		}
		else	//intersection_1 is on the upper side of intersection_2
		{
			m_upstream[intersection_1->GetLaneID(NORTHWARD)] = intersection_2->GetLaneID(NORTHWARD);
			m_downstream[intersection_2->GetLaneID(NORTHWARD)] = intersection_1->GetLaneID(NORTHWARD);

			m_upstream[intersection_2->GetLaneID(SOUTHWARD)] = intersection_1->GetLaneID(SOUTHWARD);
			m_downstream[intersection_1->GetLaneID(SOUTHWARD)] = intersection_2->GetLaneID(SOUTHWARD);

			m_downstream[intersection_1->GetLaneID(NORTHWARD)] = m_downstream[intersection_2->GetLaneID(SOUTHWARD)] = 0;
			m_upstream[intersection_1->GetLaneID(SOUTHWARD)] = m_upstream[intersection_2->GetLaneID(NORTHWARD)] = 0;
			
			m_upstream[intersection_1->GetLaneID(WESTWARD)] = m_downstream[intersection_1->GetLaneID(WESTWARD)] = 0;
			m_upstream[intersection_1->GetLaneID(EASTWARD)] = m_downstream[intersection_1->GetLaneID(EASTWARD)] = 0;

			m_upstream[intersection_2->GetLaneID(WESTWARD)] = m_downstream[intersection_2->GetLaneID(WESTWARD)] = 0;
			m_upstream[intersection_2->GetLaneID(EASTWARD)] = m_downstream[intersection_2->GetLaneID(EASTWARD)] = 0;
		}
	}
	else		//isolated intersection, no connection
	{
		m_upstream[intersection_1->GetLaneID(EASTWARD)] = m_downstream[intersection_1->GetLaneID(EASTWARD)] = 0;
		m_upstream[intersection_1->GetLaneID(WESTWARD)] = m_downstream[intersection_1->GetLaneID(WESTWARD)] = 0;
		m_upstream[intersection_1->GetLaneID(SOUTHWARD)] = m_downstream[intersection_1->GetLaneID(SOUTHWARD)] = 0;
		m_upstream[intersection_1->GetLaneID(NORTHWARD)] = m_downstream[intersection_1->GetLaneID(NORTHWARD)] = 0;
	}
}

//get intersection ID for the specific laneID
uint32_t Topology::GetIntersectionIDFromLane(uint32_t laneID)
{
	for(std::map<uint32_t, Ptr<Intersection> >::iterator it = m_intersections.begin(); it != m_intersections.end(); it++)
	{
		for(DIRECTION direction = EASTWARD; direction <= NORTHWARD; direction = DIRECTION((int)direction + 1))
			if( it->second->GetLaneID(direction) == laneID)
				return it->first;
	}
}

//get pointer to the intersection from ID
Ptr<Intersection> Topology::GetIntersection(uint32_t ID)
{
	if(m_intersections.find(ID) != m_intersections.end())
		return m_intersections[ID];
	else
	{
		NS_LOG_ERROR("Fail to get intersection from ID, check ID value: " + ID);
		return NULL;
	}
}