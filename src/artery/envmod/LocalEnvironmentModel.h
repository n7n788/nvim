/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef LOCALENVIRONMENTMODEL_H_
#define LOCALENVIRONMENTMODEL_H_

#include <boost/iterator/filter_iterator.hpp>
#include <boost/range/iterator_range.hpp>
#include <omnetpp/clistener.h>
#include "artery/utility/IdentityRegistry.h"
#include <omnetpp/csimplemodule.h>
#include <omnetpp/simtime.h>
#include <functional>
#include <map>
#include <memory>
#include <vector>
#include <string>
// 追加

namespace artery
{

class EnvironmentModelObject;
class GlobalEnvironmentModel;
class Middleware;
class Sensor;
class SensorDetection;
class VehicleDataProvider; // 追加
class Identity; // 追加
class IdentityRegistry; // 追加

/**
 * Local representation of the global environment model
 *
 * LocalEnvironmentModel tracks the GlobalEnvironmentModel's objects
 * visible by the local sensors feeding this local model.
 */
class LocalEnvironmentModel : public omnetpp::cSimpleModule, public omnetpp::cListener
{
public:
    using Object = std::weak_ptr<EnvironmentModelObject>;

    class TrackingTime
    {
    public:
        TrackingTime();
        TrackingTime(omnetpp::SimTime time);
        void tap(omnetpp::SimTime time);

        omnetpp::SimTime first() const { return mFirst; }
        omnetpp::SimTime last() const { return mLast; }

    private:
        omnetpp::SimTime mFirst;
        omnetpp::SimTime mLast;
    };

    class Tracking
    {
    public:
        using TrackingMap = std::map<const Sensor*, TrackingTime>;

        // 車両id, 2次元位置、速度、進行方向、受信センサ、取得時間から、トラッキング情報を生成
        Tracking(std::string mTraciId, double mPosX, double mPosY, double mSpeed, double mHeading, const Sensor* sensor, omnetpp::SimTime time);
        // +送信元車両のidと危険度を加える　CPMの受信時に使用
        Tracking(std::string mTraciId, double mPosX, double mPosY, double mSpeed, double mHeading, const Sensor* sensor, omnetpp::SimTime time, std::string sourceId);

        bool expired() const;
        void update();

        // センサ、取得時間、2次元位置、速度、進行方向から、トラッキング情報を更新
        void tap(const Sensor*, omnetpp::SimTime, double, double, double, double);
        void tap(const Sensor*, omnetpp::SimTime, double, double, double, double, std::string);

        void tapSensedIds(std::string id) {mSensedIds[id] = omnetpp::simTime();}
        const  std::map<std::string, omnetpp::SimTime>& getSensedIds() const {return mSensedIds;}

        std::string traci() const { return mTraciId; } // 追加
        double getPosX() const {return mPosX;}
        double getPosY() const {return mPosY;}
        double getSpeed() const {return mSpeed;}
        double getHeading() const {return mHeading;}
        omnetpp::SimTime getLastTime() const {return mLastTime;}
        const TrackingMap& sensors() const { return mSensors; }
        int getSize() const { mCpmRecv.size(); }
        int getSize100() const {mCpmRecv100.size(); }
        
    private:
        std::string mTraciId; //追加
        double mPosX;
        double mPosY;
        double mSpeed;
        double mHeading;
        omnetpp::SimTime mLastTime;
        TrackingMap mSensors;
        
        // この物体のcpmを送信している車両のidと、受信した時刻を保持
        std::map<std::string, omnetpp::SimTime> mCpmRecv = {};
        std::map<std::string, omnetpp::SimTime> mCpmRecv100 = {};
        
        // この物体をセンシングしている車両のidと、その情報を受信した時刻
        std::map<std::string, omnetpp::SimTime> mSensedIds = {};
    };

    using TrackedObjects = std::map<Object, Tracking, std::owner_less<Object>>;
    using TrackedObject = typename TrackedObjects::value_type;


    LocalEnvironmentModel();
    virtual ~LocalEnvironmentModel() = default;

    int numInitStages() const override;
    void initialize(int stage) override;
    void finish() override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

    /**
     * Updates the local environment model
     *
     * This method is supposed to get called at each TraCI simulation step.
     * Expired and no longer existing objects are removed from the local tracking.
     */
    void update();

    /**
     * Complements the local database with the sensor data objects
     * @param objs Sensor detection result including objects and obstacles
     * @param sensor Detections are measured by this sensor
     */
    void complementObjects(const SensorDetection&, const Sensor&);
    void complementObjects(const SensorDetection&, const Sensor&, std::string);
    /**
     * Get all currently seen objects by any local sensor
     */
    const TrackedObjects& allObjects() const { return mObjects; }

    // 追加
    /**
     * Get list of all sensors attached to this local entity
     *
     * Sensor pointers are only valid as long as this LocalEnvironmentModel exists!
     */
    const std::vector<Sensor*>& getSensors() const { return mSensors; }

    // 各車両のセンシング情報を更新
    void complementSensingIds(std::string, const std::shared_ptr<EnvironmentModelObject>&, std::vector<std::string>, const std::vector<std::shared_ptr<EnvironmentModelObject>>&);
    // 各車両のセンシング情報を取得
    const std::map<std::string, std::set<std::string>>& getSensingIds() const {return mSensingIds;}
private:
    void initializeSensors();

    Middleware* mMiddleware;
    GlobalEnvironmentModel* mGlobalEnvironmentModel;
    IdentityRegistry* mIdentityRegistry;

    int mTrackingCounter = 0; //追加
    TrackedObjects mObjects;
    std::vector<Sensor*> mSensors;
    std::map<std::string, std::set<std::string>> mSensingIds; // 各車両がセンシングしている車両idのマップ 
    // std::map<std::string, std::map<std::string, omnetpp::SimTime>> mSensedIds; // 各車両をセンシングしている車両idのマップ

    // シミュレーション結果のログを取る範囲
    double mLowerPosX;
    double mUpperPosX;
    double mLowerPosY;
    double mUpperPosY;
    
    // // 各物体について認識の有無を送信するシグナル
    // omnetpp::simsignal_t objectPerceptionSignal;
    // // 全物体との距離を送るシグナル
    // omnetpp::simsignal_t objectDistanceSignal;

    // // 認識している物体についてAoIを送信するシグナル
    // omnetpp::simsignal_t aoiSignal;
    // // 認識している物体との距離を送信するシグナル
    // omnetpp::simsignal_t perceptedObjectDistanceSignal;
    // omnetpp::simsignal_t cpmRecvCntSignal;

    omnetpp::simsignal_t idSignal;
    omnetpp::simsignal_t positionXSignal;
    omnetpp::simsignal_t positionYSignal;
    omnetpp::simsignal_t speedSignal;
    omnetpp::simsignal_t headingSignal;
    omnetpp::simsignal_t longitudinalAccelerationSignal;
    omnetpp::simsignal_t yawrateSignal;
    omnetpp::simsignal_t sensedVehicleIdSignal;
    omnetpp::simsignal_t sensorNameSignal;
    omnetpp::simsignal_t sensorMeasureTimeSignal;

    // 自車両の情報を得るために追加
    const VehicleDataProvider* mVehicleDataProvider = nullptr;
    const Identity* mIdentity = nullptr;
    std::string mTraciId;
    
    //AoIや物体認識確率を計算するためのデータを集める間隔
    omnetpp::SimTime mMeasureTimeDelta;
    omnetpp::SimTime mLastMeasureTime;
};

using TrackedObjectsFilterPredicate = std::function<bool(const LocalEnvironmentModel::TrackedObject&)>;
using TrackedObjectsFilterIterator = boost::filter_iterator<TrackedObjectsFilterPredicate, LocalEnvironmentModel::TrackedObjects::const_iterator>;
using TrackedObjectsFilterRange = boost::iterator_range<TrackedObjectsFilterIterator>;

TrackedObjectsFilterRange filterBySensorCategory(const LocalEnvironmentModel::TrackedObjects&, const std::string&);
// 追加
// TrackedObjectsFilterRange filterBySensorName(const LocalEnvironmentModel::TrackedObjects&, const std::string&);

} // namespace artery

#endif /* LOCALENVIRONMENTMODEL_H_ */
