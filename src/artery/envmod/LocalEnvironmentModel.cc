/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/Middleware.h"
#include "artery/application/VehicleDataProvider.h" //追加
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/utility/FilterRules.h"
#include "artery/utility/IdentityRegistry.h"
#include <inet/common/ModuleAccess.h>
#include <omnetpp/cxmlelement.h>
#include <utility>

#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <algorithm> // 追加
#include <map>
#include <string>
#include <random>

using namespace omnetpp;

namespace artery
{

Define_Module(LocalEnvironmentModel);

static const simsignal_t EnvironmentModelRefreshSignal = cComponent::registerSignal("EnvironmentModel.refresh");

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

LocalEnvironmentModel::LocalEnvironmentModel() :
    mGlobalEnvironmentModel(nullptr)
{
}

int LocalEnvironmentModel::numInitStages() const
{
    return 2;
}

//初期化メソッド
void LocalEnvironmentModel::initialize(int stage)
{
    if (stage == 0) {
        // GlobalEnvironmentModelへのポインタを初期化
        mGlobalEnvironmentModel = inet::getModuleFromPar<GlobalEnvironmentModel>(par("globalEnvironmentModule"), this);
        mGlobalEnvironmentModel->subscribe(EnvironmentModelRefreshSignal, this);

        // ミドルウェアへのポインタを初期化
        auto vehicle = inet::findContainingNode(this);
        mMiddleware = inet::getModuleFromPar<Middleware>(par("middlewareModule"), vehicle);
        
        Facilities& fac = mMiddleware->getFacilities();
        fac.register_mutable(mGlobalEnvironmentModel);
        fac.register_mutable(this);
        
        // mMeasureTimeDelta = par("measureTimeDelta");
        // mLastMeasureTime = omnetpp::SimTime();

    } else if (stage == 1) {
        // センサを初期化
        initializeSensors();

        mLowerPosX = par("lowerPosX");
        mUpperPosX = par("upperPosX");
        mLowerPosY = par("lowerPosY");
        mUpperPosY = par("upperPosY");
        
        // 各シグナルを初期化
        // objectPerceptionSignal = registerSignal("objectPerception");
        // objectDistanceSignal = registerSignal("objectDistance");

        // aoiSignal = registerSignal("aoi");
        // perceptedObjectDistanceSignal = registerSignal("perceptedObjectDistance");
        // cpmRecvCntSignal = registerSignal("cpmRecvCnt");

        idSignal = registerSignal("id");
        positionXSignal = registerSignal("positionX");
        positionYSignal = registerSignal("positionY");
        speedSignal = registerSignal("speed");
        headingSignal = registerSignal("heading");
        longitudinalAccelerationSignal = registerSignal("longitudinalAcceleration");
        yawrateSignal = registerSignal("yawrate");
        sensedVehicleIdSignal = registerSignal("sensedVehicleId");
        sensorNameSignal = registerSignal("sensorName");
        sensorMeasureTimeSignal = registerSignal("sensorMeasureTime");

        //　追加 VehicleDataProviderを初期化
        mVehicleDataProvider = &mMiddleware->getFacilities().get_const<VehicleDataProvider>();
        mIdentity = &mMiddleware->getFacilities().get_const<Identity>();
        mTraciId = mIdentity->traci;
        mIdentityRegistry = inet::getModuleFromPar<IdentityRegistry>(par("identityRegistryModule"), this);
        // emit(sumoId, mTraciId);
    }
}

// 終了メソッド
void LocalEnvironmentModel::finish()
{
    mGlobalEnvironmentModel->unsubscribe(EnvironmentModelRefreshSignal, this);
    mObjects.clear();
}

// グローバル環境モデルのリフレッシュシグナルを受信したら、センシングを実行し、ローカル環境モデルを更新
// ローカル環境モデルは<obj1, <sensor1, <ftime, ltime>>, <sensor2, <ftime, ltime>>>
//                     <obj2, <sensor1, <ftime, ltime>>, <sensor2, <ftime, ltime>>>
// センサは各オブジェクトについて、各センサが最後にセンシングした時間ltimeを更新
// この更新結果に基づいて、モデル全体を更新
void LocalEnvironmentModel::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (signal == EnvironmentModelRefreshSignal) {
        for (auto* sensor : mSensors) {
            sensor->measurement(); // センサ側でcomplementObjectを実行
        }
        update();
    }
}

// センサが実行するメソッド
// センサの検出結果から、最後にトラッキングした時間を更新
void LocalEnvironmentModel::complementObjects(const SensorDetection& detection, const Sensor& sensor)
{
    for (int i = 0; i < detection.objects.size(); i++) {
        auto identity = mIdentityRegistry->lookup<IdentityRegistry::traci>(detection.objects[i]->getExternalId());
        if (identity) { 
            // if (mTraciId == "25.0" && identity->traci == "29.0")
            //     std::cout << sensor.getSensorCategory() << ": " << detection.measurements[i] << "s, " << detection.posXs[i] << "m, " << detection.posYs[i] << "m, " << detection.speeds[i] << "m/s, " << detection.headings[i] << "radian\n";
            auto foundObject = mObjects.find(detection.objects[i]);
            if (foundObject != mObjects.end()) {
            // 発見した物体がすでに検出済みの場合、その物体のトラッキング情報を更新
                Tracking& tracking = foundObject->second;
                tracking.tap(&sensor, detection.measurements[i], detection.posXs[i], detection.posYs[i], detection.speeds[i], detection.headings[i]);
            } else {
                // 初めて検出した場合、物体をmObjectに追加し、トラッキング情報を追加
                // mObjects.emplace(detectedObject, Tracking { &sensor });
                // std::cout << "check identity\n";
                mObjects.emplace(detection.objects[i], Tracking {identity->traci, detection.posXs[i], detection.posYs[i], detection.speeds[i], detection.headings[i], &sensor, detection.measurements[i]});
                    // std::cout << "add: " << identity->traci;
            }
        }
    } 
}

// cpmの送信元のidと受信時刻をトラッキング情報として追加
void LocalEnvironmentModel::complementObjects(const SensorDetection& detection, const Sensor& sensor, std::string sourceId)
{
    for (int i = 0; i < detection.objects.size(); i++) {
        auto identity = mIdentityRegistry->lookup<IdentityRegistry::traci>(detection.objects[i]->getExternalId());
        if (identity) { 
            //  if (mTraciId == "25.0" && identity->traci == "29.0")
            //     std::cout << sensor.getSensorCategory() << ": " << detection.measurements[i] << "s, " << detection.posXs[i] << "m, " << detection.posYs[i] << "m, " << detection.speeds[i] << "m/s, " << detection.headings[i] << "radian\n";
            auto foundObject = mObjects.find(detection.objects[i]);
            if (foundObject != mObjects.end()) {
            // 発見した物体がすでに検出済みの場合、その物体のトラッキング情報を更新
                Tracking& tracking = foundObject->second;
                tracking.tap(&sensor, detection.measurements[i], detection.posXs[i], detection.posYs[i], detection.speeds[i], detection.headings[i], sourceId);
            } else {
                // 初めて検出した場合、物体をmObjectに追加し、トラッキング情報を追加
                // mObjects.emplace(detectedObject, Tracking { &sensor });
                // std::cout << "check identity\n";
                mObjects.emplace(detection.objects[i], Tracking {identity->traci, detection.posXs[i], detection.posYs[i], detection.speeds[i], detection.headings[i], &sensor, detection.measurements[i], sourceId});
            }
        }
    } 
}

// 各車両のセンシング情報を更新
void LocalEnvironmentModel::complementSensingIds(std::string sourceId, const std::shared_ptr<EnvironmentModelObject>& sourceObject, std::vector<std::string> ids, const std::vector<std::shared_ptr<EnvironmentModelObject>>& objects) 
{
    // std::cout << "LocalEnvironmentModel::complementSensingIds\n";
    if (mSensingIds.find(sourceId) != mSensingIds.end()) mSensingIds.erase(sourceId);
    for (int i = 0; i < objects.size(); i++) {
        auto identity = mIdentityRegistry->lookup<IdentityRegistry::traci>(sourceId);
        if (identity) { 
            //  if (mTraciId == "25.0" && identity->traci == "29.0")
            mSensingIds[sourceId].insert(ids[i]);
            auto foundObject = mObjects.find(objects[i]);
            if (foundObject != mObjects.end()) {
                Tracking& tracking = foundObject->second;
                tracking.tapSensedIds(sourceId);
                //  if (mTraciId == "36.0" && ids[i] == "66.0") std::cout << sourceId << "in \n"; 
                // tracking.tapSensedObjects(sourceObject);
            }
        }
    }
    // if (mTraciId == "85.0" && sourceId == "0.0") {
    //     std::cout << "LEM: " << mTraciId << " perceive " << sourceId << " sensing ";
    //     for (std::string id: mSensingIds[sourceId]) {
    //         std::cout  << id << ", ";
    //     }
    //     std::cout << "\n";
    // }
}

// ローカル環境モデル[mObject]を更新
void LocalEnvironmentModel::update()
{
    //0.1sごとに自車両の情報を送信
    //  emit(idSignal, std::hash<std::string>()(mTraciId));
    // emit(idSignal, std::stol(mTraciId));
    // emit(positionXSignal, round(mVehicleDataProvider->position().x, vanetza::units::si::meter));
    // emit(positionYSignal, round(mVehicleDataProvider->position().y, vanetza::units::si::meter));
    // emit(speedSignal, round(mVehicleDataProvider->speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec);
    // emit(headingSignal,  round(mVehicleDataProvider->heading(), decidegree));
    // emit(longitudinalAccelerationSignal, mVehicleDataProvider->acceleration() / vanetza::units::si::meter_per_second_squared);
    // emit(yawrateSignal, round(mVehicleDataProvider->yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0);
    
    // 0.1sごとにLEMの情報を送信
    // std::map<std::string, int> nameToIndex =  {
    //     {"Radar", 0},
    //     {"CA", 1},
    //     {"CP", 2}
    // };


    
    // std::cout << mTraciId << "\n";
    // LEMないの情報を更新
    for (auto it = mObjects.begin(); it != mObjects.end();) {
        const Object& object = it->first;
        Tracking& tracking = it->second;
        // std::cout << "  object: " << tracking.traci() << "\n";
        // トラッキング情報を更新
        tracking.update();

        if (object.expired() || tracking.expired()) {
            it = mObjects.erase(it);
        } else it++;
    }

    // std::cout << mTraciId << "\n";
    // LEMの中身をシグナルで送信
    // for (auto object: mObjects) {
    //     auto tracking = object.second;
    //     for (auto sensor: tracking.sensors()) {
    //         // std::cout << "   " << tracking.traci() << ", " << sensor.first->getSensorCategory() << ", " << sensor.second.last() << "\n";
    //         // emit(sensedVehicleIdSignal,  std::hash<std::string>()(tracking.traci()));
    //         emit(sensedVehicleIdSignal,  std::stol(tracking.traci()));
    //         emit(sensorNameSignal, nameToIndex[sensor.first->getSensorCategory()]);
    //         emit(sensorMeasureTimeSignal, sensor.second.last());
    //     }
    // }

    // 物体認識確率を計算するために、各車両について認識の有無と距離を送信
    // グローバル環境モデルからすべてのオブジェクトを取得
    // auto objs = mGlobalEnvironmentModel->getAllObject();
    // for (auto obj: objs) {
    //     auto identity = mIdentityRegistry->lookup<IdentityRegistry::traci>(obj->getExternalId());
    //     if (identity) {
    //         // そのオブジェクトが自車両ならスキップ
    //         if (identity->traci == mIdentity->traci) continue;
    //         // std::cout << "   " << obj->getExternalId() << ": ";
    //         // 各オブジェクトについて、ローカル環境オブジェクトに含まれるかどうかをチェック
    //         auto local_obj = mObjects.find(obj);
    //         if (local_obj != mObjects.end()) {
    //             // ローカル環境モデルにオブジェクトがある場合、少なくとも１つのセンサが一秒以内に認識していればOK
    //             const Tracking& tracking = local_obj->second;
    //             bool ok = false;
    //             for (auto sensor: tracking.sensors()) {
    //                 if (sensor.second.last() + omnetpp::SimTime { 1000, SIMTIME_MS } >= simTime()) {
    //                     ok = true;
    //                     break;
    //                 }
    //             }
    //             if (ok) emit(objectPerceptionSignal, 1);
    //             else emit(objectPerceptionSignal, 0);
    //         } else emit(objectPerceptionSignal, 0);

    //         // 自車両とそのオブジェクトの距離をシグナルで送信
    //         auto dist = round(distance(mVehicleDataProvider->position(), obj->getVehicleData().position()), vanetza::units::si::meter);
    //         emit(objectDistanceSignal,  dist);
    //     }
    // }

    // AoIを計算するために、認識している物体とのAoIと距離をシグナルで送信
    // ローカル環境モデルからすべてのオブジェクトを取得
    // double posX = round(mVehicleDataProvider->position().x, vanetza::units::si::meter);
    // double posY = round(mVehicleDataProvider->position().y, vanetza::units::si::meter);
    // 自車両の位置がある範囲内ならログを取る
    // std::cout << mLowerPosX << ", " << mUpperPosX << ", " << mLowerPosY << ", " << mUpperPosY << "\n";
    // if (posX >= mLowerPosX && posX <= mUpperPosX && posY >= mLowerPosY && posY <= mUpperPosY) {
    //     // std::cout << mTraciId << ": " << posX << ", " << posY << "\n";
    //     // std::cout << mTraciId << "\n";
    //     for (auto object: mObjects) {
    //         std::shared_ptr<EnvironmentModelObject> obj = object.first.lock();
    //         if (obj) {
    //             auto identity = mIdentityRegistry->lookup<IdentityRegistry::traci>(obj->getExternalId());
    //             if (identity) {
    //                 auto tracking = object.second;
    //                 // オブジェクトとの距離をシグナルで送信
    //                 // 高速道路シナリオ使用時
    //                 // auto dist = std::sqrt(std::pow(posX - tracking.getPosX(), 2) + std::pow(posY - tracking.getPosY(), 2));
    //                 // auto dist = round(distance(mVehicleDataProvider->position(), obj->getVehicleData().position()), vanetza::units::si::meter);
    //                 // マンハッタンシナリオ使用時
    //                 // std::cout << "  " << tracking.traci() << ":  " << tracking.getPosX() << ", " << tracking.getPosY() << "\n";
    //                 // emit(perceptedObjectDistanceSignal, dist);

    //                 // 各センサについて、最も最近に物体を認識したセンサをもとに、AoIを計算
    //                 omnetpp::SimTime latest{0, SIMTIME_MS};
    //                 for (auto sensor: tracking.sensors()) {
    //                     latest = std::max(latest, sensor.second.last());
    //                 }
    //                 omnetpp::SimTime aoi = simTime() - latest;
    //                 emit(aoiSignal, aoi);
    //                 emit(cpmRecvCntSignal, tracking.size());
    //             }
    //         }
    //     }
    // }
}

// xmlファイルからセンサを初期化
void LocalEnvironmentModel::initializeSensors()
{
    cXMLElement* config = par("sensors").xmlValue();
    for (cXMLElement* sensor_cfg : config->getChildrenByTagName("sensor"))
    {
        cXMLElement* sensor_filters = sensor_cfg->getFirstChildWithTag("filters");
        bool sensor_applicable = true;
        if (sensor_filters) {
            auto identity = mMiddleware->getIdentity();
            FilterRules rules(getRNG(0), identity);
            sensor_applicable = rules.applyFilterConfig(*sensor_filters);
        }

        if (sensor_applicable) {
            cModuleType* module_type = cModuleType::get(sensor_cfg->getAttribute("type"));
            const char* sensor_name = sensor_cfg->getAttribute("name");
            if (!sensor_name || !*sensor_name) {
                sensor_name = module_type->getName();
            }

            // cModule* module = module_type->createScheduleInit(sensor_name, this);
            //追加
            cModule* module = module_type->create(sensor_name, this);
            module->finalizeParameters();
            module->buildInside();

            auto sensor = dynamic_cast<artery::Sensor*>(module);

            if (sensor != nullptr) {
                cXMLElement* vis_cfg = sensor_cfg->getFirstChildWithTag("visualization");
                // sensor->setVisualization(SensorVisualizationConfig(vis_cfg));
                // 追加
                // sensor->setSensorName(sensor_name);
            } else {
                throw cRuntimeError("%s is not of type Sensor", module_type->getFullName());
            }

            // 追加
            module->scheduleStart(simTime());
            module->callInitialize();

            mSensors.push_back(sensor);
        }
    }
}


// LocalEnvironmentModel::Tracking::Tracking(const Sensor* sensor)
// {
//     mSensors.emplace(sensor, TrackingTime {});
// }
// 追加
LocalEnvironmentModel::Tracking::Tracking(std::string traciId, double posX, double posY, double speed, double heading, const Sensor* sensor, omnetpp::SimTime time, std::string sourceId)
 : mTraciId(traciId), mPosX(posX), mPosY(posY), mSpeed(speed), mHeading(heading), mLastTime(time)
{
    if (traciId != sourceId) mCpmRecv[sourceId] = omnetpp::simTime();
    if (traciId != sourceId) mCpmRecv100[sourceId] = omnetpp::simTime();

    mSensors.emplace(sensor, TrackingTime {time});
}

LocalEnvironmentModel::Tracking::Tracking(std::string traciId, double posX, double posY, double speed, double heading, const Sensor* sensor, omnetpp::SimTime time)
 : mTraciId(traciId), mPosX(posX), mPosY(posY), mSpeed(speed), mHeading(heading), mLastTime(time)
{
    mSensors.emplace(sensor, TrackingTime {time});
}

bool LocalEnvironmentModel::Tracking::expired() const
{
    return mSensors.empty();
}

// 各オブジェクトについて、トラッキング情報を更新
void LocalEnvironmentModel::Tracking::update()
{
    // 1.001秒より長く受信していないcpmの送信元は削除
    // if (mTraciId == "0.0") std::cout << mTraciId << "mCpmRecv\n";
    // std::random_device rnd;
    // std::mt19937 mt(rnd());
    // std::normal_distribution<> norm(100.0, 20.0); // 平均100ms, 分散20msの乱数を生成
    for (auto it = mCpmRecv.begin(); it != mCpmRecv.end();) {
        // if (mTraciId == "0.0") std::cout << "  "  << it->first << ", rcv time: " << it->second << "[s]\n";
        // cpmの受信ウインドウ1秒を超えたら、廃棄
       if (it->second + omnetpp::SimTime{1000, SIMTIME_MS} < omnetpp::simTime()) it = mCpmRecv.erase(it);
        else it++;
    }

    for (auto it = mCpmRecv100.begin(); it != mCpmRecv100.end();) {
        // if (mTraciId == "0.0") std::cout << "  "  << it->first << ", rcv time: " << it->second << "[s]\n";
        // cpmの受信ウインドウ100msを超えたら、廃棄
       if (it->second + omnetpp::SimTime{100, SIMTIME_MS} < omnetpp::simTime()) it = mCpmRecv100.erase(it);
        else it++;
    }

    // 車両をセンシングしている車両idを更新
    for (auto it = mSensedIds.begin(); it != mSensedIds.end();) {
        if (it->second + omnetpp::SimTime{100, SIMTIME_MS} < omnetpp::simTime()) it = mSensedIds.erase(it);
        else it++;
    }
    // std::cout << "\n";
    // すべてのセンサをチェック
    // for (auto it = mSensors.begin(); it != mSensors.end();) {
      
    //   const Sensor* sensor = it->first;
    //   const TrackingTime& tracking = it->second;

    //   // トラッキングが一定以上されていなければそのセンサーを消去
    //   const bool expired = tracking.last() + omnetpp::SimTime{3000, SIMTIME_MS} < simTime();
    //   if (expired) {
    //       it = mSensors.erase(it);
    //   } else {
    //       ++it;
    //   }
    // }
}

// 新しく取得したセンサ情報からトラッキング情報を追加
void LocalEnvironmentModel::Tracking::tap(const Sensor* sensor, omnetpp::SimTime time, double posX, double posY, double speed, double heading)
{
    // if (mTraciId == "34.0") std::cout << omnetpp::simTime() << ": add lastTime: " << time << ", posX: " << posX << ", posY: " << posY << "\n";
    if (mLastTime < time) {
        mPosX = posX;
        mPosY = posY;
        mSpeed = speed;
        mHeading = heading;
        mLastTime = time;
    }
    auto found = mSensors.find(sensor);
    if (found != mSensors.end()) {
        // そのセンサーによって検出済みの場合、TrackingTimeのtap()を実行し、最後に認識した時間を現在時刻に更新
         TrackingTime& tracking = found->second;
         tracking.tap(time);
    } else {
        // そのセンサーによって初めて検出した場合、センサとTrackingTimeのペアをmSensorsに追加
        mSensors.emplace(sensor, TrackingTime {time});
    }
}

void LocalEnvironmentModel::Tracking::tap(const Sensor* sensor, omnetpp::SimTime time, double posX, double posY, double speed, double heading, std::string sourceId)
{
    
    if (mLastTime < time) {
        mPosX = posX;
        mPosY = posY;
        mSpeed = speed;
        mHeading = heading;
        mLastTime = time;
    }
    
    mCpmRecv[sourceId] = omnetpp::simTime();
    mCpmRecv100[sourceId] = omnetpp::simTime();

    if (sourceId != mTraciId) tapSensedIds(sourceId);

    auto found = mSensors.find(sensor);
    if (found != mSensors.end()) {
        // そのセンサーによって検出済みの場合、TrackingTimeのtap()を実行し、最後に認識した時間を現在時刻に更新
         TrackingTime& tracking = found->second;
         tracking.tap(time);
    } else {
        // そのセンサーによって初めて検出した場合、センサとTrackingTimeのペアをmSensorsに追加
        mSensors.emplace(sensor, TrackingTime {time});
    }
}

LocalEnvironmentModel::TrackingTime::TrackingTime() :
   mFirst(simTime()), mLast(simTime())
{
}

LocalEnvironmentModel::TrackingTime::TrackingTime(omnetpp::SimTime time) :
   mFirst(time), mLast(time)
{
}

// あるオブジェクトをあるセンサが最後に認識した時間を更新

void LocalEnvironmentModel::TrackingTime::tap(omnetpp::SimTime time)
{
    if (time > mLast) mLast = time;
}


TrackedObjectsFilterRange filterBySensorCategory(const LocalEnvironmentModel::TrackedObjects& all, const std::string& category)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedObjectsFilterPredicate seenByCategory = [category](const LocalEnvironmentModel::TrackedObject& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&category](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorCategory() == category;
                });
    };

    auto begin = boost::make_filter_iterator(seenByCategory, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByCategory, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

// 追加
// TrackedObjectsFilterRange filterBySensorName(const LocalEnvironmentModel::TrackedObjects& all, const std::string& name)
// {
//     // capture `category` by value because lambda expression will be evaluated after this function's return
//     TrackedObjectsFilterPredicate seenByName = [name](const LocalEnvironmentModel::TrackedObject& obj) {
//         const auto& detections = obj.second.sensors();
//         return std::any_of(detections.begin(), detections.end(),
//                 [&name](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
//                     const Sensor* sensor = tracking.first;
//                     return sensor->getSensorName() == name;
//                 });
//     };

//     auto begin = boost::make_filter_iterator(seenByName, all.begin(), all.end());
//     auto end = boost::make_filter_iterator(seenByName, all.end(), all.end());
//     return boost::make_iterator_range(begin, end);
// }

} // namespace artery
