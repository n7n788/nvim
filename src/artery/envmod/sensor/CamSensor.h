/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ENVMOD_CAMSENSOR_H_
#define ENVMOD_CAMSENSOR_H_

#include "artery/envmod/sensor/BaseSensor.h"
#include <vanetza/asn1/cam.hpp>
#include <omnetpp/clistener.h>
#include <string>

namespace artery
{

class IdentityRegistry;
class Middleware;
class Timer;

class CamSensor : public BaseSensor, public omnetpp::cListener
{
public:
    void measurement() override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject* = nullptr) override;
    void setVisualization(const SensorVisualizationConfig&) override {}
    const FieldOfView* getFieldOfView() const override { return nullptr; }
    omnetpp::SimTime getValidityPeriod() const override;
    SensorPosition position() const override { return SensorPosition::VIRTUAL; }
    const std::string& getSensorCategory() const override;

    // 追加
    // const std::string getSensorName() const;
    // void setSensorName(const std::string& name);

    // std::string mSensorName = "Cam"; // センサー名
    std::string mTraciId; // 追加 sumoのID
    int mReceiveCount; //追加 受信したcam数

    //追加
    // const std::string getSensorName() const override {  return mSensorName; }
    // void setSensorName(const std::string& name) override { mSensorName = name; }
    // SensorDetection detectObjects() const override;

protected:
    void initialize() override;
    void finish() override;

private:
    IdentityRegistry* mIdentityRegistry;
    Middleware* mMiddleware;
    const Timer* mTimer; // 追加
    // omnetpp::SimTime mValidityPeriod;
    // std::string mSensorName;

    // シグナル
    omnetpp::simsignal_t camReceivedGenerationTimeSignal;
	omnetpp::simsignal_t camReceivedSourceIdSignal;
    // omnetpp::simsignal_t camReceivedByteSignal;
};

} // namespace artery

#endif /* ENVMOD_CAMSENSOR_H_ */
