/*
 * plt_header.cc
 *
 *  Created on: 4 Jul, 2013
 *      Author: along
 *  Modified on 2015.01.23 by along
 *    remove veihcle_postion in PltContent
 */
#include "plt_header.h"
#include "ns3/core-module.h"

PltHeader::PltHeader ()
{
  // we must provide a public default constructor,
  // implicit or explicit, but never private.
}

PltHeader::~PltHeader ()
{
}

TypeId PltHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PltHeader")
    .SetParent<Header> ()
    .AddConstructor<PltHeader> ()
  ;
  return tid;
}

TypeId PltHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

void PltHeader::Print (std::ostream &os) const
{
  // This method is invoked by the packet printing
  // routines to print the content of my header.
  //os << "data=" << m_data << std::endl;
  for(std::list<PltContent>::iteator it = plt.begin(); it != plt.end(); it++)
    os<<it->vehicleID<<" ";
  os<<std::endl;
}

uint32_t PltHeader::GetSerializedSize (void) const
{
  //the size of plt_list is added at the first 4 bytes of header
  return plt.size() * sizeof(PltContent) + 4;
}

void PltHeader::Serialize (Buffer::Iterator start) const
{
  // we can serialize four bytes at the start of the buffer.
  // we write them in network byte order.
	start.WriteHtonU32(plt.size());
	for(std::list<PltContent>::iteator it = plt.begin(); it != plt.end(); it++)
	{
		start.WriteHtonU32(it->vehicleID);
	}
	//NS_LOG_UNCOND("serializing finished");
}

uint32_t PltHeader::Deserialize (Buffer::Iterator start)
{
  // we can deserialize 4 bytes from the start of the buffer.
  // we read them in network byte order and store them
  // in host byte order.
	plt.clear();
	uint32_t size = start.ReadNtohU32();
	for(uint32_t i = 0; i < size; i++)
	{
		PltContent content;
		content.vehicleID = start.ReadNtohU32();
		plt.push_back(content);
	}

  // we return the number of bytes effectively read.
   return plt.size()*sizeof(PltContent) + 4;
}

void PltHeader::SetData (std::list<PltContent> l)
{
  plt = l;
}

std::list<PltContent> PltHeader::GetData (void) const
{
  return plt;
}
