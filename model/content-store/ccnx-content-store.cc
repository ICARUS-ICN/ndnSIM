/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011,2012 University of California, Los Angeles
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Alexander Afanasyev <alexander.afanasyev@ucla.edu>
 *         Ilya Moiseenko <iliamo@cs.ucla.edu>
 *         
 */

#include "ccnx-content-store.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/ccnx-name-components.h"
#include "ns3/ccnx-interest-header.h"
#include "ns3/ccnx-content-object-header.h"

NS_LOG_COMPONENT_DEFINE ("CcnxContentStore");

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED (CcnxContentStore);

TypeId 
CcnxContentStore::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::CcnxContentStore")
    .SetGroupName ("Ccnx")
    .SetParent<Object> ()

    .AddTraceSource ("CacheHits", "Trace called every time there is a cache hit",
                     MakeTraceSourceAccessor (&CcnxContentStore::m_cacheHitsTrace))

    .AddTraceSource ("CacheMisses", "Trace called every time there is a cache miss",
                     MakeTraceSourceAccessor (&CcnxContentStore::m_cacheMissesTrace))
    ;

  return tid;
}


CcnxContentStore::~CcnxContentStore () 
{
}

//////////////////////////////////////////////////////////////////////

CcnxContentStoreEntry::CcnxContentStoreEntry (Ptr<CcnxContentObjectHeader> header, Ptr<const Packet> packet)
  : m_header (header)
  , m_packet (packet->Copy ())
{
}

Ptr<Packet>
CcnxContentStoreEntry::GetFullyFormedCcnxPacket () const
{
  static CcnxContentObjectTail tail; ///< \internal for optimization purposes

  Ptr<Packet> packet = m_packet->Copy ();
  packet->AddHeader (*m_header);
  packet->AddTrailer (tail);
  return packet;
}

const CcnxNameComponents&
CcnxContentStoreEntry::GetName () const
{
  return m_header->GetName ();
}

Ptr<const CcnxContentObjectHeader>
CcnxContentStoreEntry::GetHeader () const
{
  return m_header;
}

Ptr<const Packet>
CcnxContentStoreEntry::GetPacket () const
{
  return m_packet;
}

} // namespace ns3