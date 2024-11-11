// MCSモジュール
#include "artery/application/ManeuverCoordinationService.h"
#include "artery/application/ManeuverCoordinationMessage.h"
// ユーティリティモジュール
#include "artery/utility/IdentityRegistry.h"
#include "artery/utility/InitStages.h"
// ommnetppモジュール
#include <ommnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
// vanetzaモジュール
#include <vanetza/btp/ports.hpp>
// boostモジュール
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

namespace artery
{

Define_Module(ManeuverCoordinationService)

int ManeuverCoordinationService::numInitStages() const
{
    return InitStages::Total;
} 

void ManeuverCoordinationService::initialize(int stage)
{
    if (stage == InitStages::Prepare) {
        ItsG5Service::initialize();
    } else if (stage == InitStaeg::Self) {
        mTraciId = getFacilities().get_const<Identity>().traci;
    }
}

void ManeuverCoordinationService::triger()
{
    using namespace vanetza;
    btp::DataRequestB req;
    req.destination_port = host_cast<PortNumber>(getPortNumber());
    req.gn.transport_type = geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(0);
    req.gn.communucation_profile = geonet::CommunicationProfile::ITS_G5;

    // 車両IDとバイト長を設定
    auto packet = new ManeuverCoordinationMessage();
    packet->setTraciId(mTraciId);
    packet->setByteLength(100); 
    request(req, packet);
}

void ManeuverCoordinationService::indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket* packet)
{
    auto mcm = omnetpp::check_and_cast<ManeuverCoordinationMessage*>(packet);
    delete packet;
}

} // namespace artery