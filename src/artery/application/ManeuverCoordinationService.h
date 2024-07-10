/*
* miyata 2024/07/01
*/

#ifndef ARTERY_MANEUVERCOORDINATIONSERVICE_H_
#define ARTERY_MANEUVERCOORDINATIONSERVICE_H_

#include "artery/application/ManeuverCoordinationMessage.h"
#include "artery/application/ItsG5Service.h"
#include "artery/application/FrenetPlanning.h"
#include <map>
#include <string>

namespace artery
{
class ItsG5Service;
class FrenetPathPlannning;
class FrenetPath;

class ManeuverCoordinationService : public ItsG5Service
{
public:

    ManeuverCoordinationService();
    void initialize() override; // 初期化
    void indicate(const vantza::btp::DataIndication&, omnetpp::cPacket*) override; // 受信処理
    void trigger() override; // 送信処理

    virtual ~ManeuverCoordinationService();

private:

    // 他車両から受信した希望経路を基に交渉受け入れの有無を判断
    std::map<std::string, bool> checkNegotiation(); 
    //　リクエストを出す必要性を判断
    bool checkNeedRequest();

    std::map<std::string, FrenetPath> mReceivedPlannedPaths; // 他車両から受信した予定経路
    std::map<std::string, FrenetPath> mReceivedRequestedPaths; // 他車両から受信した希望経路
    std::map<std::string, bool> mNegotiation; // 他車両への交渉結果
};

}
#endif /* ARTERY_MANEUVERCOORDINATIONSERVICE_H_ */