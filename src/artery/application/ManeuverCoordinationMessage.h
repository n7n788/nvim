/*
* miyata 2024/07/01
*/

#ifndef ARTERY_MANEUVERCOORDINATIONMESSAGE_H
#define ARTERY_COLLECTIVEPERCEPTIONMOCKMESSAGE_H

#include <omnetpp/cpacket.h>
#include <string>
#include <vector>

namespace artery
{
class FrenetPath;

class ManeuverCoordinationMessage : public omnetpp::cPacket
{
public:

    ManeuverCoordinationMessage();
    ManeuverCoordinationMessage(const ManeuverCoordinationMessage&) = default;
    ManeuverCoordinationMessage& operator=(const ManeuverCoordinationMessage&) = default;

    // 各メンバ変数のgetとset
    const std::string& getTraciId() const { return mTraciId; }
    void setTraciId(std::string traciId) { mTraciId = traciId; }

    const FrenetPath& getPlannedPath() const { return *mPlannedPath; }
    void setPlannedPath(const FrenetPath&&);

    const FrenetPath& getRequestedPath() const { return *mRequestedPath; }
    void setRequestedPath(const FrenetPath&&);

private:
    std::string mTraciId; // 車両ID
    FrenetPath mPlannedPath; // 予定経路
    FrenetPath mRequestedPath; // 要求経路
};

}
#endif ARTERY_MANEUVERCOORDINATIONMESSAGE_H
