/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "offload-wifi-mac.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "mac-rx-middle.h"
#include "mac-tx-middle.h"
#include "mac-low.h"
#include "dcf.h"
#include "dcf-manager.h"
#include "wifi-phy.h"
#include "msdu-standard-aggregator.h"
#include "mpdu-standard-aggregator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("OffloadWifiMac");

NS_OBJECT_ENSURE_REGISTERED (OffloadWifiMac);

OffloadWifiMac::OffloadWifiMac () :
  m_htSupported (0),
  m_vhtSupported (0),
  m_erpSupported (0),
  m_dsssSupported (0)
{
  NS_LOG_FUNCTION (this);
  m_rxMiddle = new MacRxMiddle ();
  m_rxMiddle->SetForwardCallback (MakeCallback (&OffloadWifiMac::Receive, this));

  m_txMiddle = new MacTxMiddle ();

  m_low = CreateObject<MacLow> ();
  m_low->SetRxCallback (MakeCallback (&MacRxMiddle::Receive, m_rxMiddle));

  m_dcfManager = new DcfManager ();
  m_dcfManager->SetupLowListener (m_low);

  m_dca = CreateObject<DcaTxop> ();
  m_dca->SetLow (m_low);
  m_dca->SetManager (m_dcfManager);
  m_dca->SetTxMiddle (m_txMiddle);
  m_dca->SetTxOkCallback (MakeCallback (&OffloadWifiMac::TxOk, this));
  m_dca->SetTxFailedCallback (MakeCallback (&OffloadWifiMac::TxFailed, this));

  //Construct the EDCAFs. The ordering is important - highest
  //priority (Table 9-1 UP-to-AC mapping; IEEE 802.11-2012) must be created
  //first.
  SetupEdcaQueue (AC_VO);
  SetupEdcaQueue (AC_VI);
  SetupEdcaQueue (AC_BE);
  SetupEdcaQueue (AC_BK);
}

OffloadWifiMac::~OffloadWifiMac ()
{
  NS_LOG_FUNCTION (this);
}

void
OffloadWifiMac::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  m_dca->Initialize ();

  for (EdcaQueues::iterator i = m_edca.begin (); i != m_edca.end (); ++i)
    {
      i->second->Initialize ();
    }
}

void
OffloadWifiMac::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  delete m_rxMiddle;
  m_rxMiddle = 0;

  delete m_txMiddle;
  m_txMiddle = 0;

  delete m_dcfManager;
  m_dcfManager = 0;

  m_low->Dispose ();
  m_low = 0;

  m_phy = 0;
  m_stationManager = 0;

  m_dca->Dispose ();
  m_dca = 0;

  for (EdcaQueues::iterator i = m_edca.begin (); i != m_edca.end (); ++i)
    {
      i->second = 0;
    }
}

void
OffloadWifiMac::SetWifiRemoteStationManager (Ptr<WifiRemoteStationManager> stationManager)
{
  NS_LOG_FUNCTION (this << stationManager);
  m_stationManager = stationManager;
  m_stationManager->SetHtSupported (GetHtSupported ());
  m_stationManager->SetVhtSupported (GetVhtSupported ());
  m_low->SetWifiRemoteStationManager (stationManager);

  m_dca->SetWifiRemoteStationManager (stationManager);

  for (EdcaQueues::iterator i = m_edca.begin (); i != m_edca.end (); ++i)
    {
      i->second->SetWifiRemoteStationManager (stationManager);
    }
}

Ptr<WifiRemoteStationManager>
OffloadWifiMac::GetWifiRemoteStationManager () const
{
  return m_stationManager;
}

HtCapabilities
OffloadWifiMac::GetHtCapabilities (void) const
{
  HtCapabilities capabilities;
  capabilities.SetHtSupported (1);
  if (m_htSupported)
    {
      capabilities.SetLdpc (m_phy->GetLdpc ());
      capabilities.SetSupportedChannelWidth (m_phy->GetChannelWidth () == 40);
      capabilities.SetShortGuardInterval20 (m_phy->GetGuardInterval ());
      capabilities.SetShortGuardInterval40 (m_phy->GetChannelWidth () == 40 && m_phy->GetGuardInterval ());
      capabilities.SetGreenfield (m_phy->GetGreenfield ());
      capabilities.SetMaxAmsduLength (1); //hardcoded for now (TBD)
      capabilities.SetLSigProtectionSupport (!m_phy->GetGreenfield ());
      capabilities.SetMaxAmpduLength (3); //hardcoded for now (TBD)
      uint64_t maxSupportedRate = 0; //in bit/s
      for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
        {
          WifiMode mcs = m_phy->GetMcs (i);
          if (mcs.GetModulationClass () != WIFI_MOD_CLASS_HT)
            {
              continue;
            }
          capabilities.SetRxMcsBitmask (mcs.GetMcsValue ());
          uint8_t nss = (mcs.GetMcsValue () / 8) + 1;
          NS_ASSERT (nss > 0 && nss < 5);
          if (mcs.GetDataRate (m_phy->GetChannelWidth (), m_phy->GetGuardInterval (), nss) > maxSupportedRate)
            {
              maxSupportedRate = mcs.GetDataRate (m_phy->GetChannelWidth (), m_phy->GetGuardInterval (), nss);
              NS_LOG_DEBUG ("Updating maxSupportedRate to " << maxSupportedRate);
            }
        }
      capabilities.SetRxHighestSupportedDataRate (maxSupportedRate / 1e6); //in Mbit/s
      capabilities.SetTxMcsSetDefined (m_phy->GetNMcs () > 0);
      capabilities.SetTxMaxNSpatialStreams (m_phy->GetSupportedTxSpatialStreams ());
    }
  return capabilities;
}

VhtCapabilities
OffloadWifiMac::GetVhtCapabilities (void) const
{
  VhtCapabilities capabilities;
  capabilities.SetVhtSupported (1);
  if (m_vhtSupported)
    {
      if (m_phy->GetChannelWidth () == 160)
        {
          capabilities.SetSupportedChannelWidthSet (1);
        }
      else
        {
          capabilities.SetSupportedChannelWidthSet (0);
        }
      capabilities.SetMaxMpduLength (2); //hardcoded for now (TBD)
      capabilities.SetRxLdpc (m_phy->GetLdpc ());
      capabilities.SetShortGuardIntervalFor80Mhz ((m_phy->GetChannelWidth () == 80) && m_phy->GetGuardInterval ());
      capabilities.SetShortGuardIntervalFor160Mhz ((m_phy->GetChannelWidth () == 160) && m_phy->GetGuardInterval ());
      capabilities.SetMaxAmpduLengthExponent (7); //hardcoded for now (TBD)
      uint8_t maxMcs = 0;
      for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
        {
          WifiMode mcs = m_phy->GetMcs (i);
          if ((mcs.GetModulationClass () == WIFI_MOD_CLASS_VHT)
              && (mcs.GetMcsValue () > maxMcs))
            {
              maxMcs = mcs.GetMcsValue ();
            }
        }
      // Support same MaxMCS for each spatial stream
      for (uint8_t nss = 1; nss <= m_phy->GetSupportedRxSpatialStreams (); nss++)
        {
          capabilities.SetRxMcsMap (maxMcs, nss);
        }
      for (uint8_t nss = 1; nss <= m_phy->GetSupportedTxSpatialStreams (); nss++)
        {
          capabilities.SetTxMcsMap (maxMcs, nss);
        }
    }
  return capabilities;
}

void
OffloadWifiMac::SetVoMaxAmsduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_voMaxAmsduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetViMaxAmsduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_viMaxAmsduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetBeMaxAmsduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_beMaxAmsduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetBkMaxAmsduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_bkMaxAmsduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetVoMaxAmpduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_voMaxAmpduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetViMaxAmpduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_viMaxAmpduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetBeMaxAmpduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_beMaxAmpduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetBkMaxAmpduSize (uint32_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_bkMaxAmpduSize = size;
  ConfigureAggregation ();
}

void
OffloadWifiMac::SetVoBlockAckThreshold (uint8_t threshold)
{
  NS_LOG_FUNCTION (this << (uint16_t) threshold);
  GetVOQueue ()->SetBlockAckThreshold (threshold);
}

void
OffloadWifiMac::SetViBlockAckThreshold (uint8_t threshold)
{
  NS_LOG_FUNCTION (this << (uint16_t) threshold);
  GetVIQueue ()->SetBlockAckThreshold (threshold);
}

void
OffloadWifiMac::SetBeBlockAckThreshold (uint8_t threshold)
{
  NS_LOG_FUNCTION (this << (uint16_t) threshold);
  GetBEQueue ()->SetBlockAckThreshold (threshold);
}

void
OffloadWifiMac::SetBkBlockAckThreshold (uint8_t threshold)
{
  NS_LOG_FUNCTION (this << (uint16_t) threshold);
  GetBKQueue ()->SetBlockAckThreshold (threshold);
}

void
OffloadWifiMac::SetVoBlockAckInactivityTimeout (uint16_t timeout)
{
  NS_LOG_FUNCTION (this << timeout);
  GetVOQueue ()->SetBlockAckInactivityTimeout (timeout);
}

void
OffloadWifiMac::SetViBlockAckInactivityTimeout (uint16_t timeout)
{
  NS_LOG_FUNCTION (this << timeout);
  GetVIQueue ()->SetBlockAckInactivityTimeout (timeout);
}

void
OffloadWifiMac::SetBeBlockAckInactivityTimeout (uint16_t timeout)
{
  NS_LOG_FUNCTION (this << timeout);
  GetBEQueue ()->SetBlockAckInactivityTimeout (timeout);
}

void
OffloadWifiMac::SetBkBlockAckInactivityTimeout (uint16_t timeout)
{
  NS_LOG_FUNCTION (this << timeout);
  GetBKQueue ()->SetBlockAckInactivityTimeout (timeout);
}

void
OffloadWifiMac::SetupEdcaQueue (enum AcIndex ac)
{
  NS_LOG_FUNCTION (this << ac);

  //Our caller shouldn't be attempting to setup a queue that is
  //already configured.
  NS_ASSERT (m_edca.find (ac) == m_edca.end ());

  Ptr<EdcaTxopN> edca = CreateObject<EdcaTxopN> ();
  edca->SetLow (m_low);
  edca->SetManager (m_dcfManager);
  edca->SetTxMiddle (m_txMiddle);
  edca->SetTxOkCallback (MakeCallback (&OffloadWifiMac::TxOk, this));
  edca->SetTxFailedCallback (MakeCallback (&OffloadWifiMac::TxFailed, this));
  edca->SetAccessCategory (ac);
  edca->CompleteConfig ();

  m_edca.insert (std::make_pair (ac, edca));
}

void
OffloadWifiMac::SetTypeOfStation (TypeOfStation type)
{
  NS_LOG_FUNCTION (this << type);
  for (EdcaQueues::iterator i = m_edca.begin (); i != m_edca.end (); ++i)
    {
      i->second->SetTypeOfStation (type);
    }
}

Ptr<DcaTxop>
OffloadWifiMac::GetDcaTxop () const
{
  return m_dca;
}

Ptr<EdcaTxopN>
OffloadWifiMac::GetVOQueue () const
{
  return m_edca.find (AC_VO)->second;
}

Ptr<EdcaTxopN>
OffloadWifiMac::GetVIQueue () const
{
  return m_edca.find (AC_VI)->second;
}

Ptr<EdcaTxopN>
OffloadWifiMac::GetBEQueue () const
{
  return m_edca.find (AC_BE)->second;
}

Ptr<EdcaTxopN>
OffloadWifiMac::GetBKQueue () const
{
  return m_edca.find (AC_BK)->second;
}

void
OffloadWifiMac::SetWifiPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);
  m_phy = phy;
  m_dcfManager->SetupPhyListener (phy);
  m_low->SetPhy (phy);
}

Ptr<WifiPhy>
OffloadWifiMac::GetWifiPhy (void) const
{
  NS_LOG_FUNCTION (this);
  return m_phy;
}

void
OffloadWifiMac::ResetWifiPhy (void)
{
  NS_LOG_FUNCTION (this);
  m_low->ResetPhy ();
  m_dcfManager->RemovePhyListener (m_phy);
  m_phy = 0;
}

void
OffloadWifiMac::SetForwardUpCallback (ForwardUpCallback upCallback)
{
  NS_LOG_FUNCTION (this);
  m_forwardUp = upCallback;
}

void
OffloadWifiMac::SetLinkUpCallback (Callback<void> linkUp)
{
  NS_LOG_FUNCTION (this);
  m_linkUp = linkUp;
}

void
OffloadWifiMac::SetLinkDownCallback (Callback<void> linkDown)
{
  NS_LOG_FUNCTION (this);
  m_linkDown = linkDown;
}

void
OffloadWifiMac::SetQosSupported (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  m_qosSupported = enable;
}

bool
OffloadWifiMac::GetQosSupported () const
{
  return m_qosSupported;
}

void
OffloadWifiMac::SetHtSupported (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  m_htSupported = enable;
  if (enable)
    {
      SetQosSupported (true);
    }
  if (!enable && !m_vhtSupported)
    {
      DisableAggregation ();
    }
  else
    {
      EnableAggregation ();
    }
}

bool
OffloadWifiMac::GetVhtSupported () const
{
  return m_vhtSupported;
}

void
OffloadWifiMac::SetVhtSupported (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  m_vhtSupported = enable;
  if (enable)
    {
      SetQosSupported (true);
    }
  if (!enable && !m_htSupported)
    {
      DisableAggregation ();
    }
  else
    {
      EnableAggregation ();
    }
}

bool
OffloadWifiMac::GetHtSupported () const
{
  return m_htSupported;
}

bool
OffloadWifiMac::GetErpSupported () const
{
  return m_erpSupported;
}

void
OffloadWifiMac::SetErpSupported (bool enable)
{
  NS_LOG_FUNCTION (this);
  if (enable)
    {
      SetDsssSupported (true);
    }
  m_erpSupported = enable;
}

void
OffloadWifiMac::SetDsssSupported (bool enable)
{
  NS_LOG_FUNCTION (this);
  m_dsssSupported = enable;
}

bool
OffloadWifiMac::GetDsssSupported () const
{
  return m_dsssSupported;
}

void
OffloadWifiMac::SetCtsToSelfSupported (bool enable)
{
  NS_LOG_FUNCTION (this);
  m_low->SetCtsToSelfSupported (enable);
}

bool
OffloadWifiMac::GetCtsToSelfSupported () const
{
  return m_low->GetCtsToSelfSupported ();
}

void
OffloadWifiMac::SetSlot (Time slotTime)
{
  NS_LOG_FUNCTION (this << slotTime);
  m_dcfManager->SetSlot (slotTime);
  m_low->SetSlotTime (slotTime);
}

Time
OffloadWifiMac::GetSlot (void) const
{
  return m_low->GetSlotTime ();
}

void
OffloadWifiMac::SetSifs (Time sifs)
{
  NS_LOG_FUNCTION (this << sifs);
  m_dcfManager->SetSifs (sifs);
  m_low->SetSifs (sifs);
}

Time
OffloadWifiMac::GetSifs (void) const
{
  return m_low->GetSifs ();
}

void
OffloadWifiMac::SetEifsNoDifs (Time eifsNoDifs)
{
  NS_LOG_FUNCTION (this << eifsNoDifs);
  m_dcfManager->SetEifsNoDifs (eifsNoDifs);
}

Time
OffloadWifiMac::GetEifsNoDifs (void) const
{
  return m_dcfManager->GetEifsNoDifs ();
}

void
OffloadWifiMac::SetRifs (Time rifs)
{
  NS_LOG_FUNCTION (this << rifs);
  m_low->SetRifs (rifs);
}

Time
OffloadWifiMac::GetRifs (void) const
{
  return m_low->GetRifs ();
}

void
OffloadWifiMac::SetPifs (Time pifs)
{
  NS_LOG_FUNCTION (this << pifs);
  m_low->SetPifs (pifs);
}

Time
OffloadWifiMac::GetPifs (void) const
{
  return m_low->GetPifs ();
}

void
OffloadWifiMac::SetAckTimeout (Time ackTimeout)
{
  NS_LOG_FUNCTION (this << ackTimeout);
  m_low->SetAckTimeout (ackTimeout);
}

Time
OffloadWifiMac::GetAckTimeout (void) const
{
  return m_low->GetAckTimeout ();
}

void
OffloadWifiMac::SetCtsTimeout (Time ctsTimeout)
{
  NS_LOG_FUNCTION (this << ctsTimeout);
  m_low->SetCtsTimeout (ctsTimeout);
}

Time
OffloadWifiMac::GetCtsTimeout (void) const
{
  return m_low->GetCtsTimeout ();
}

void
OffloadWifiMac::SetBasicBlockAckTimeout (Time blockAckTimeout)
{
  NS_LOG_FUNCTION (this << blockAckTimeout);
  m_low->SetBasicBlockAckTimeout (blockAckTimeout);
}

Time
OffloadWifiMac::GetBasicBlockAckTimeout (void) const
{
  return m_low->GetBasicBlockAckTimeout ();
}

void
OffloadWifiMac::SetCompressedBlockAckTimeout (Time blockAckTimeout)
{
  NS_LOG_FUNCTION (this << blockAckTimeout);
  m_low->SetCompressedBlockAckTimeout (blockAckTimeout);
}

Time
OffloadWifiMac::GetCompressedBlockAckTimeout (void) const
{
  return m_low->GetCompressedBlockAckTimeout ();
}

void
OffloadWifiMac::SetAddress (Mac48Address address)
{
  NS_LOG_FUNCTION (this << address);
  m_low->SetAddress (address);
}

Mac48Address
OffloadWifiMac::GetAddress (void) const
{
  return m_low->GetAddress ();
}

void
OffloadWifiMac::SetSsid (Ssid ssid)
{
  NS_LOG_FUNCTION (this << ssid);
  m_ssid = ssid;
}

Ssid
OffloadWifiMac::GetSsid (void) const
{
  return m_ssid;
}

void
OffloadWifiMac::SetBssid (Mac48Address bssid)
{
  NS_LOG_FUNCTION (this << bssid);
  m_low->SetBssid (bssid);
}

Mac48Address
OffloadWifiMac::GetBssid (void) const
{
  return m_low->GetBssid ();
}

void
OffloadWifiMac::SetPromisc (void)
{
  m_low->SetPromisc ();
}

void
OffloadWifiMac::SetShortSlotTimeSupported (bool enable)
{
  NS_LOG_FUNCTION (this << enable);
  m_shortSlotTimeSupported = enable;
}

bool
OffloadWifiMac::GetShortSlotTimeSupported (void) const
{
  return m_shortSlotTimeSupported;
}

void
OffloadWifiMac::Enqueue (Ptr<const Packet> packet,
                         Mac48Address to, Mac48Address from)
{
  //We expect OffloadWifiMac subclasses which do support forwarding (e.g.,
  //AP) to override this method. Therefore, we throw a fatal error if
  //someone tries to invoke this method on a class which has not done
  //this.
  NS_FATAL_ERROR ("This MAC entity (" << this << ", " << GetAddress ()
                                      << ") does not support Enqueue() with from address");
}

bool
OffloadWifiMac::SupportsSendFrom (void) const
{
  return false;
}

void
OffloadWifiMac::ForwardUp (Ptr<Packet> packet, Mac48Address from, Mac48Address to)
{
  NS_LOG_FUNCTION (this << packet << from);
  m_forwardUp (packet, from, to);
}

void
OffloadWifiMac::Receive (Ptr<Packet> packet, const WifiMacHeader *hdr)
{
  NS_LOG_FUNCTION (this << packet << hdr);

  Mac48Address to = hdr->GetAddr1 ();
  Mac48Address from = hdr->GetAddr2 ();

  //We don't know how to deal with any frame that is not addressed to
  //us (and odds are there is nothing sensible we could do anyway),
  //so we ignore such frames.
  //
  //The derived class may also do some such filtering, but it doesn't
  //hurt to have it here too as a backstop.
  if (to != GetAddress ())
    {
      return;
    }

  if (hdr->IsMgt () && hdr->IsAction ())
    {
      //There is currently only any reason for Management Action
      //frames to be flying about if we are a QoS STA.
      NS_ASSERT (m_qosSupported);

      WifiActionHeader actionHdr;
      packet->RemoveHeader (actionHdr);

      switch (actionHdr.GetCategory ())
        {
        case WifiActionHeader::BLOCK_ACK:

          switch (actionHdr.GetAction ().blockAck)
            {
            case WifiActionHeader::BLOCK_ACK_ADDBA_REQUEST:
              {
                MgtAddBaRequestHeader reqHdr;
                packet->RemoveHeader (reqHdr);

                //We've received an ADDBA Request. Our policy here is
                //to automatically accept it, so we get the ADDBA
                //Response on it's way immediately.
                SendAddBaResponse (&reqHdr, from);
                //This frame is now completely dealt with, so we're done.
                return;
              }
            case WifiActionHeader::BLOCK_ACK_ADDBA_RESPONSE:
              {
                MgtAddBaResponseHeader respHdr;
                packet->RemoveHeader (respHdr);

                //We've received an ADDBA Response. We assume that it
                //indicates success after an ADDBA Request we have
                //sent (we could, in principle, check this, but it
                //seems a waste given the level of the current model)
                //and act by locally establishing the agreement on
                //the appropriate queue.
                AcIndex ac = QosUtilsMapTidToAc (respHdr.GetTid ());
                m_edca[ac]->GotAddBaResponse (&respHdr, from);
                //This frame is now completely dealt with, so we're done.
                return;
              }
            case WifiActionHeader::BLOCK_ACK_DELBA:
              {
                MgtDelBaHeader delBaHdr;
                packet->RemoveHeader (delBaHdr);

                if (delBaHdr.IsByOriginator ())
                  {
                    //This DELBA frame was sent by the originator, so
                    //this means that an ingoing established
                    //agreement exists in MacLow and we need to
                    //destroy it.
                    m_low->DestroyBlockAckAgreement (from, delBaHdr.GetTid ());
                  }
                else
                  {
                    //We must have been the originator. We need to
                    //tell the correct queue that the agreement has
                    //been torn down
                    AcIndex ac = QosUtilsMapTidToAc (delBaHdr.GetTid ());
                    m_edca[ac]->GotDelBaFrame (&delBaHdr, from);
                  }
                //This frame is now completely dealt with, so we're done.
                return;
              }
            default:
              NS_FATAL_ERROR ("Unsupported Action field in Block Ack Action frame");
              return;
            }
        default:
          NS_FATAL_ERROR ("Unsupported Action frame received");
          return;
        }
    }
  NS_FATAL_ERROR ("Don't know how to handle frame (type=" << hdr->GetType ());
}

void
OffloadWifiMac::DeaggregateAmsduAndForward (Ptr<Packet> aggregatedPacket,
                                            const WifiMacHeader *hdr)
{
  MsduAggregator::DeaggregatedMsdus packets =
    MsduAggregator::Deaggregate (aggregatedPacket);

  for (MsduAggregator::DeaggregatedMsdusCI i = packets.begin ();
       i != packets.end (); ++i)
    {
      ForwardUp ((*i).first, (*i).second.GetSourceAddr (),
                 (*i).second.GetDestinationAddr ());
    }
}

void
OffloadWifiMac::SendAddBaResponse (const MgtAddBaRequestHeader *reqHdr,
                                   Mac48Address originator)
{
  NS_LOG_FUNCTION (this);
  WifiMacHeader hdr;
  hdr.SetAction ();
  hdr.SetAddr1 (originator);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetAddr3 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();

  MgtAddBaResponseHeader respHdr;
  StatusCode code;
  code.SetSuccess ();
  respHdr.SetStatusCode (code);
  //Here a control about queues type?
  respHdr.SetAmsduSupport (reqHdr->IsAmsduSupported ());

  if (reqHdr->IsImmediateBlockAck ())
    {
      respHdr.SetImmediateBlockAck ();
    }
  else
    {
      respHdr.SetDelayedBlockAck ();
    }
  respHdr.SetTid (reqHdr->GetTid ());
  //For now there's not no control about limit of reception. We
  //assume that receiver has no limit on reception. However we assume
  //that a receiver sets a bufferSize in order to satisfy next
  //equation: (bufferSize + 1) % 16 = 0 So if a recipient is able to
  //buffer a packet, it should be also able to buffer all possible
  //packet's fragments. See section 7.3.1.14 in IEEE802.11e for more details.
  respHdr.SetBufferSize (1023);
  respHdr.SetTimeout (reqHdr->GetTimeout ());

  WifiActionHeader actionHdr;
  WifiActionHeader::ActionValue action;
  action.blockAck = WifiActionHeader::BLOCK_ACK_ADDBA_RESPONSE;
  actionHdr.SetAction (WifiActionHeader::BLOCK_ACK, action);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (respHdr);
  packet->AddHeader (actionHdr);

  //We need to notify our MacLow object as it will have to buffer all
  //correctly received packets for this Block Ack session
  m_low->CreateBlockAckAgreement (&respHdr, originator,
                                  reqHdr->GetStartingSequence ());

  //It is unclear which queue this frame should go into. For now we
  //bung it into the queue corresponding to the TID for which we are
  //establishing an agreement, and push it to the head.
  m_edca[QosUtilsMapTidToAc (reqHdr->GetTid ())]->PushFront (packet, hdr);
}

TypeId
OffloadWifiMac::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::OffloadWifiMac")
    .SetParent<WifiMac> ()
    .SetGroupName ("Wifi")
    .AddAttribute ("QosSupported",
                   "This Boolean attribute is set to enable 802.11e/WMM-style QoS support at this STA.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&OffloadWifiMac::SetQosSupported,
                                        &OffloadWifiMac::GetQosSupported),
                   MakeBooleanChecker ())
    .AddAttribute ("HtSupported",
                   "This Boolean attribute is set to enable 802.11n support at this STA.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&OffloadWifiMac::SetHtSupported,
                                        &OffloadWifiMac::GetHtSupported),
                   MakeBooleanChecker ())
    .AddAttribute ("VhtSupported",
                   "This Boolean attribute is set to enable 802.11ac support at this STA.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&OffloadWifiMac::SetVhtSupported,
                                        &OffloadWifiMac::GetVhtSupported),
                   MakeBooleanChecker ())
    .AddAttribute ("CtsToSelfSupported",
                   "Use CTS to Self when using a rate that is not in the basic rate set.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&OffloadWifiMac::SetCtsToSelfSupported,
                                        &OffloadWifiMac::GetCtsToSelfSupported),
                   MakeBooleanChecker ())
    .AddAttribute ("VO_MaxAmsduSize",
                   "Maximum length in bytes of an A-MSDU for AC_VO access class.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetVoMaxAmsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("VI_MaxAmsduSize",
                   "Maximum length in bytes of an A-MSDU for AC_VI access class.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetViMaxAmsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("BE_MaxAmsduSize",
                   "Maximum length in bytes of an A-MSDU for AC_BE access class.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBeMaxAmsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("BK_MaxAmsduSize",
                   "Maximum length in bytes of an A-MSDU for AC_BK access class.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBkMaxAmsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("VO_MaxAmpduSize",
                   "Maximum length in bytes of an A-MPDU for AC_VO access class.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetVoMaxAmpduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("VI_MaxAmpduSize",
                   "Maximum length in bytes of an A-MPDU for AC_VI access class.",
                   UintegerValue (65535),
                   MakeUintegerAccessor (&OffloadWifiMac::SetViMaxAmpduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("BE_MaxAmpduSize",
                   "Maximum length in bytes of an A-MPDU for AC_BE access class.",
                   UintegerValue (65535),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBeMaxAmpduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("BK_MaxAmpduSize",
                   "Maximum length in bytes of an A-MPDU for AC_BK access class.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBkMaxAmpduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("VO_BlockAckThreshold",
                   "If number of packets in VO queue reaches this value, "
                   "block ack mechanism is used. If this value is 0, block ack is never used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetVoBlockAckThreshold),
                   MakeUintegerChecker<uint8_t> (0, 64))
    .AddAttribute ("VI_BlockAckThreshold",
                   "If number of packets in VI queue reaches this value, "
                   "block ack mechanism is used. If this value is 0, block ack is never used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetViBlockAckThreshold),
                   MakeUintegerChecker<uint8_t> (0, 64))
    .AddAttribute ("BE_BlockAckThreshold",
                   "If number of packets in BE queue reaches this value, "
                   "block ack mechanism is used. If this value is 0, block ack is never used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBeBlockAckThreshold),
                   MakeUintegerChecker<uint8_t> (0, 64))
    .AddAttribute ("BK_BlockAckThreshold",
                   "If number of packets in BK queue reaches this value, "
                   "block ack mechanism is used. If this value is 0, block ack is never used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBkBlockAckThreshold),
                   MakeUintegerChecker<uint8_t> (0, 64))
    .AddAttribute ("VO_BlockAckInactivityTimeout",
                   "Represents max time (blocks of 1024 micro seconds) allowed for block ack"
                   "inactivity for AC_VO. If this value isn't equal to 0 a timer start after that a"
                   "block ack setup is completed and will be reset every time that a block ack"
                   "frame is received. If this value is 0, block ack inactivity timeout won't be used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetVoBlockAckInactivityTimeout),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("VI_BlockAckInactivityTimeout",
                   "Represents max time (blocks of 1024 micro seconds) allowed for block ack"
                   "inactivity for AC_VI. If this value isn't equal to 0 a timer start after that a"
                   "block ack setup is completed and will be reset every time that a block ack"
                   "frame is received. If this value is 0, block ack inactivity timeout won't be used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetViBlockAckInactivityTimeout),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("BE_BlockAckInactivityTimeout",
                   "Represents max time (blocks of 1024 micro seconds) allowed for block ack"
                   "inactivity for AC_BE. If this value isn't equal to 0 a timer start after that a"
                   "block ack setup is completed and will be reset every time that a block ack"
                   "frame is received. If this value is 0, block ack inactivity timeout won't be used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBeBlockAckInactivityTimeout),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("BK_BlockAckInactivityTimeout",
                   "Represents max time (blocks of 1024 micro seconds) allowed for block ack"
                   "inactivity for AC_BK. If this value isn't equal to 0 a timer start after that a"
                   "block ack setup is completed and will be reset every time that a block ack"
                   "frame is received. If this value is 0, block ack inactivity timeout won't be used.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&OffloadWifiMac::SetBkBlockAckInactivityTimeout),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("ShortSlotTimeSupported",
                   "Whether or not short slot time is supported (only used by ERP APs or STAs).",
                   BooleanValue (true),
                   MakeBooleanAccessor (&WifiMac::GetShortSlotTimeSupported,
                                        &WifiMac::SetShortSlotTimeSupported),
                   MakeBooleanChecker ())
    .AddAttribute ("DcaTxop",
                   "The DcaTxop object.",
                   PointerValue (),
                   MakePointerAccessor (&OffloadWifiMac::GetDcaTxop),
                   MakePointerChecker<DcaTxop> ())
    .AddAttribute ("VO_EdcaTxopN",
                   "Queue that manages packets belonging to AC_VO access class.",
                   PointerValue (),
                   MakePointerAccessor (&OffloadWifiMac::GetVOQueue),
                   MakePointerChecker<EdcaTxopN> ())
    .AddAttribute ("VI_EdcaTxopN",
                   "Queue that manages packets belonging to AC_VI access class.",
                   PointerValue (),
                   MakePointerAccessor (&OffloadWifiMac::GetVIQueue),
                   MakePointerChecker<EdcaTxopN> ())
    .AddAttribute ("BE_EdcaTxopN",
                   "Queue that manages packets belonging to AC_BE access class.",
                   PointerValue (),
                   MakePointerAccessor (&OffloadWifiMac::GetBEQueue),
                   MakePointerChecker<EdcaTxopN> ())
    .AddAttribute ("BK_EdcaTxopN",
                   "Queue that manages packets belonging to AC_BK access class.",
                   PointerValue (),
                   MakePointerAccessor (&OffloadWifiMac::GetBKQueue),
                   MakePointerChecker<EdcaTxopN> ())
    .AddTraceSource ("TxOkHeader",
                     "The header of successfully transmitted packet.",
                     MakeTraceSourceAccessor (&OffloadWifiMac::m_txOkCallback),
                     "ns3::WifiMacHeader::TracedCallback")
    .AddTraceSource ("TxErrHeader",
                     "The header of unsuccessfully transmitted packet.",
                     MakeTraceSourceAccessor (&OffloadWifiMac::m_txErrCallback),
                     "ns3::WifiMacHeader::TracedCallback")
  ;
  return tid;
}

void
OffloadWifiMac::FinishConfigureStandard (enum WifiPhyStandard standard)
{
  NS_LOG_FUNCTION (this << standard);
  uint32_t cwmin = 0;
  uint32_t cwmax = 0;
  switch (standard)
    {
    case WIFI_PHY_STANDARD_80211ac:
      SetVhtSupported (true);
    case WIFI_PHY_STANDARD_80211n_5GHZ:
      SetHtSupported (true);
      cwmin = 15;
      cwmax = 1023;
      break;
    case WIFI_PHY_STANDARD_80211n_2_4GHZ:
      SetHtSupported (true);
    case WIFI_PHY_STANDARD_80211g:
      SetErpSupported (true);
    case WIFI_PHY_STANDARD_holland:
    case WIFI_PHY_STANDARD_80211a:
    case WIFI_PHY_STANDARD_80211_10MHZ:
    case WIFI_PHY_STANDARD_80211_5MHZ:
      cwmin = 15;
      cwmax = 1023;
      break;
    case WIFI_PHY_STANDARD_80211b:
      SetDsssSupported (true);
      cwmin = 31;
      cwmax = 1023;
      break;
    default:
      NS_FATAL_ERROR ("Unsupported WifiPhyStandard in OffloadWifiMac::FinishConfigureStandard ()");
    }

  ConfigureContentionWindow (cwmin, cwmax);
}

void
OffloadWifiMac::ConfigureContentionWindow (uint32_t cwMin, uint32_t cwMax)
{
  bool isDsssOnly = m_dsssSupported && !m_erpSupported;
  //The special value of AC_BE_NQOS which exists in the Access
  //Category enumeration allows us to configure plain old DCF.
  ConfigureDcf (m_dca, cwMin, cwMax, isDsssOnly, AC_BE_NQOS);

  //Now we configure the EDCA functions
  for (EdcaQueues::iterator i = m_edca.begin (); i != m_edca.end (); ++i)
  {
    ConfigureDcf (i->second, cwMin, cwMax, isDsssOnly, i->first);
  }
}

void
OffloadWifiMac::TxOk (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << hdr);
  m_txOkCallback (hdr);
}

void
OffloadWifiMac::TxFailed (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << hdr);
  m_txErrCallback (hdr);
}

void
OffloadWifiMac::ConfigureAggregation (void)
{
  NS_LOG_FUNCTION (this);
  if (GetVOQueue ()->GetMsduAggregator () != 0)
    {
      GetVOQueue ()->GetMsduAggregator ()->SetMaxAmsduSize (m_voMaxAmsduSize);
    }
  if (GetVIQueue ()->GetMsduAggregator () != 0)
    {
      GetVIQueue ()->GetMsduAggregator ()->SetMaxAmsduSize (m_viMaxAmsduSize);
    }
  if (GetBEQueue ()->GetMsduAggregator () != 0)
    {
      GetBEQueue ()->GetMsduAggregator ()->SetMaxAmsduSize (m_beMaxAmsduSize);
    }
  if (GetBKQueue ()->GetMsduAggregator () != 0)
    {
      GetBKQueue ()->GetMsduAggregator ()->SetMaxAmsduSize (m_bkMaxAmsduSize);
    }
  if (GetVOQueue ()->GetMpduAggregator () != 0)
    {
      GetVOQueue ()->GetMpduAggregator ()->SetMaxAmpduSize (m_voMaxAmpduSize);
    }
  if (GetVIQueue ()->GetMpduAggregator () != 0)
    {
      GetVIQueue ()->GetMpduAggregator ()->SetMaxAmpduSize (m_viMaxAmpduSize);
    }
  if (GetBEQueue ()->GetMpduAggregator () != 0)
    {
      GetBEQueue ()->GetMpduAggregator ()->SetMaxAmpduSize (m_beMaxAmpduSize);
    }
  if (GetBKQueue ()->GetMpduAggregator () != 0)
    {
      GetBKQueue ()->GetMpduAggregator ()->SetMaxAmpduSize (m_bkMaxAmpduSize);
    }
}

void
OffloadWifiMac::EnableAggregation (void)
{
  NS_LOG_FUNCTION (this);
  for (EdcaQueues::iterator i = m_edca.begin (); i != m_edca.end (); ++i)
  {
    if (i->second->GetMsduAggregator () == 0)
      {
        Ptr<MsduStandardAggregator> msduAggregator = CreateObject<MsduStandardAggregator> ();
        i->second->SetMsduAggregator (msduAggregator);
      }
    if (i->second->GetMpduAggregator () == 0)
      {
        Ptr<MpduStandardAggregator> mpduAggregator = CreateObject<MpduStandardAggregator> ();
        i->second->SetMpduAggregator (mpduAggregator);
      }
  }
  ConfigureAggregation ();
}

void
OffloadWifiMac::DisableAggregation (void)
{
  NS_LOG_FUNCTION (this);
  for (EdcaQueues::iterator i = m_edca.begin (); i != m_edca.end (); ++i)
  {
    i->second->SetMsduAggregator (0);
    i->second->SetMpduAggregator (0);
  }
}

} //namespace ns3
