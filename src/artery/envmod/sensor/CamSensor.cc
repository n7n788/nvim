/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/CamSensor.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/CaObject.h"
#include "artery/application/Middleware.h"
#include "artery/utility/IdentityRegistry.h"
#include <inet/common/ModuleAccess.h>
#include <math.h> 

using namespace omnetpp;

namespace artery
{

static const simsignal_t CamReceivedSignal = cComponent::registerSignal("CamReceived");

Define_Module(CamSensor);

void CamSensor::initialize()
{
    // mValidityPeriod = par("validityPeriod"); // 追加
    BaseSensor::initialize();
    mIdentityRegistry = inet::getModuleFromPar<IdentityRegistry>(par("identityRegistryModule"), this);
    getMiddleware().subscribe(CamReceivedSignal, this);

    // 追加 sumoのIDを取得
	mTraciId = getFacilities().get_const<Identity>().traci; 
    mReceiveCount = 0;

    // ミドルウェアへのポインタを初期化
    // auto vehicle = inet::findContainingNode(this);
    // mMiddleware = inet::getModuleFromPar<Middleware>(par("middlewareModule"), vehicle);
    // mTimer = &mMiddleware->getFacilities().get_const<Timer>();
    mTimer = &getFacilities().get_const<Timer>();

    // シグナルを登録
    camReceivedGenerationTimeSignal = registerSignal("camReceivedGenerationTime");
	camReceivedSourceIdSignal = registerSignal("camReceivedSourceId");
    // camReceivedByteSignal = registerSignal("camReceivedByte");
}

void CamSensor::finish()
{
    // std::cout << mTraciId << ": " << mReceiveCount << "\n";
    getMiddleware().unsubscribe(CamReceivedSignal, this);
    BaseSensor::finish();
}

void CamSensor::measurement()
{
    Enter_Method("measurement");
}

void CamSensor::receiveSignal(cComponent*, simsignal_t signal, cObject *obj, cObject*)
{
    if (signal == CamReceivedSignal) {
        auto* cam = dynamic_cast<CaObject*>(obj);
        if (cam) {
            uint32_t stationID = cam->asn1()->header.stationID;
            auto identity = mIdentityRegistry->lookup<IdentityRegistry::application>(stationID);
            if (identity) {
                // sumoのIDを使用して、グローバル環境モデルからオブジェクトを獲得
                auto object = mGlobalEnvironmentModel->getObject(identity->traci);
                auto measurement = mTimer->getTimeFor(mTimer->reconstructMilliseconds(cam->asn1()->cam.generationDeltaTime));

                SensorDetection detection;
                detection.objects.push_back(object);
                detection.measurements.push_back(measurement);
                detection.posXs.push_back(cam->asn1()->cam.camParameters.basicContainer.posX);
                detection.posYs.push_back(cam->asn1()->cam.camParameters.basicContainer.posY);
                detection.speeds.push_back((double) cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue / 100.0);
                detection.headings.push_back((double) cam->asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue * M_PI / 1800);
                // detection.risks.push_back(-1);
                mLocalEnvironmentModel->complementObjects(detection, *this);
                // 受信したCAMの送信元とメッセージIDをプリント
                // if (mTraciId == "2.0" && identity->traci == "26.0") std::cout << "  cam " << mTraciId << " received from " << identity->traci << " " << measurement << "\n";
                // std::cout << mTraciId << ": " << omnetpp::simTime() << " receive cam\n";
                emit(camReceivedGenerationTimeSignal, measurement);
                // emit(camReceivedSourceIdSignal, std::hash<std::string>()(identity->traci));
                emit(camReceivedSourceIdSignal, std::stol(identity->traci));
            } else {
                EV_WARN << "Unknown identity for station ID " << stationID;
                // std::cout << "Unknown identity for station ID " << stationID;
            }
        } else {
            EV_ERROR << "received signal has no CaObject";
            // std::cout << "received signal has no CaObject";
        }
        mReceiveCount++;
    }
}

omnetpp::SimTime CamSensor::getValidityPeriod() const
{
    return omnetpp::SimTime { 1100, SIMTIME_MS };
    // return mValidityPeriod; // 追加
}

const std::string& CamSensor::getSensorCategory() const
{
    static const std::string category = "CA";
    return category;
}

// 追加
// SensorDetection CamSensor::detectObjects() const
// {
//     // return empty sensor detection because CAM omod/CMakeFiles/envmod.dir/sensor/CamSensor.cc.o
//     SensorDetection detection;
//     return detection;
// }

// const std::string CamSensor::getSensorName() const
// {
//     return mSensorName;
// }

// void CamSensor::setSensorName(const std::string& name)
// {
//     mSensorName = name;
// }

} // namespace artery
