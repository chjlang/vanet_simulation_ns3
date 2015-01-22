#include "topology.h"

//setup connections between two intersections, the upstream/downstream is set to -1 if there is no upstream/downstream available 
void Topology::ConnectInteresctions(Ptr<Intersection> intersection_1, Ptr<Intersection> intersection_2, Pattern pattern)
{
	m_intersections.insert(intersection_1);
	m_intersections.insert(intersection_2);

	if(pattern == HORIZONTAL)	//intersection_1 is on the left-hand side of intersection_2
	{
		m_upstream[intersection_1->GetLaneID(WESTWARD)] = intersection_2->GetLaneID(WESTWARD);
		m_downstream[intersection_2->GetLaneID(WESTWARD)] = intersection_1->GetLaneID(WESTWARD);

		m_upstream[intersection_2->GetLaneID(EASTWARD)] = intersection_1->GetLaneID(EASTWARD);
		m_downstream[intersection_1->GetLaneID(EASTWARD)] = intersection_2->GetLaneID(EASTWARD);

		m_downstream[intersection_1->GetLaneID(WESTWARD)] = m_downstream[intersection_2->GetLaneID(EASTWARD)] = -1;
		m_upstream[intersection_1->GetLaneID(EASTWARD)] = m_upstream[intersection_2->GetLaneID(WESTWARD)] = -1;
		
		m_upstream[intersection_1->GetLaneID(NORTHWARD)] = m_downstream[intersection_1->GetLaneID(NORTHWARD)] = -1;
		m_upstream[intersection_1->GetLaneID(SOUTHWARD)] = m_downstream[intersection_1->GetLaneID(SOUTHWARD)] = -1;

		m_upstream[intersection_2->GetLaneID(NORTHWARD)] = m_downstream[intersection_2->GetLaneID(NORTHWARD)] = -1;
		m_upstream[intersection_2->GetLaneID(SOUTHWARD)] = m_downstream[intersection_2->GetLaneID(SOUTHWARD)] = -1;
	}
	else	//intersection_1 is on the upper side of intersection_2
	{
		m_upstream[intersection_1->GetLaneID(NORTHWARD)] = intersection_2->GetLaneID(NORTHWARD);
		m_downstream[intersection_2->GetLaneID(NORTHWARD)] = intersection_1->GetLaneID(NORTHWARD);

		m_upstream[intersection_2->GetLaneID(SOUTHWARD)] = intersection_1->GetLaneID(SOUTHWARD);
		m_downstream[intersection_1->GetLaneID(SOUTHWARD)] = intersection_2->GetLaneID(SOUTHWARD);

		m_downstream[intersection_1->GetLaneID(NORTHWARD)] = m_downstream[intersection_2->GetLaneID(SOUTHWARD)] = -1;
		m_upstream[intersection_1->GetLaneID(SOUTHWARD)] = m_upstream[intersection_2->GetLaneID(NORTHWARD)] = -1;
		
		m_upstream[intersection_1->GetLaneID(WESTWARD)] = m_downstream[intersection_1->GetLaneID(WESTWARD)] = -1;
		m_upstream[intersection_1->GetLaneID(EASTWARD)] = m_downstream[intersection_1->GetLaneID(EASTWARD)] = -1;

		m_upstream[intersection_2->GetLaneID(WESTWARD)] = m_downstream[intersection_2->GetLaneID(WESTWARD)] = -1;
		m_upstream[intersection_2->GetLaneID(EASTWARD)] = m_downstream[intersection_2->GetLaneID(EASTWARD)] = -1;
	}
}

//get intersection ID for the specific laneID
int Topology::GetIntersectionIDFromLane(int laneID)
{
	for(std::set<Ptr<Intersection> >::iterator it = m_intersections.begin(); it != m_intersections.end(); it++)
	{
		for(DIRECTION direction = EASTWARD; direction <= NORTHWARD; direction++)
			if( (*it)->GetLaneID(direction) == laneID)
				return (*it)->GetID();
	}

	NS_LOG_ERROR("Fail to get intersection from laneID, check laneID value: "<<laneID);
	return -1;
}

//get pointer to the intersection from ID
Ptr<Intersection> Topology::GetIntersection(int ID)
{
	for(std::set<Ptr<Intersection> >::iterator it = m_intersections.begin(); it != m_intersections.end(); it++)
	{
		if((*it)->GetID() == ID)
			return it;
	}

	NS_LOG_ERROR("Fail to get intersection from ID, check ID value: "<<ID);
	return -1;
}