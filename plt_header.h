/*
 * plt_header.h
 *
 *  Created on: 3 Jul, 2013
 *      Author: along

 *  Modified on 2015.01.23 by along
 *    remove veihcle_postion in PltContent
 */

#ifndef PLT_HEADER_H_
#define PLT_HEADER_H_

#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/header.h"
#include <iostream>
#include <list>

using namespace ns3;

struct PltContent
{
	uint32_t vehicleID;
};

class PltHeader : public Header
{
public:

  PltHeader ();
  virtual ~PltHeader ();

  void SetData (std::list<PltContent> data);
  std::list<PltContent> GetData (void) const;

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual uint32_t GetSerializedSize (void) const;
private:
  std::list<PltContent> plt;
};

#endif /* PLT_HEADER_H_ */
