/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CaObject.h"
#include "artery/application/CaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include <string>
#include "artery/application/Middleware.h"



namespace artery
{

using namespace omnetpp;

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(100); // 500msから変更。CAMのバイトサイズを350byte程度に維持するため

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}


Define_Module(CaService)

CaService::CaService() :
	mGenCamMin { 100, SIMTIME_MS },
	mExponentialMean { 50, SIMTIME_MS },
	mGenCamMax { 1000, SIMTIME_MS },
	mGenCam(mGenCamMax), /*the currently valid upper limit of the CAM generation interval*/
	mGenCamLowDynamicsCounter(0),
	/*if vehicle is low dynamic, 3 consective cam is sent after cam is sent due to high dynamics*/
	mGenCamLowDynamicsLimit(3)
{
}

void CaService::initialize()
{
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	
	// 追加 sumoのIDを取得
	mTraciId = getFacilities().get_const<Identity>().traci; 
	mCount = 0;
	
	/*map managing awareness object*/
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCamTimestamp = simTime();

    double delay = 0.001 * intuniform(0, 1000, 0);
    startUpTime = simTime() + delay;

	// first generated CAM shall include the low frequency container
	mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

	// generation rate boundaries
	mGenCamMin = par("minInterval");
	mGenCamMax = par("maxInterval");

	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");
    mExponentialNonPeriodic = par( "exponentialNonPeriodic");
    mExponentialMean = par("exponentialMean");
    mGenCamNonPeriodic = mGenCamMin + exponential(mExponentialMean);

	// シグナルを登録
	camGenerationTimeSignal = registerSignal("camGenerationTime");
	camSourceIdSignal = registerSignal("camSourceId");
	// camByteSignal = registerSignal("camByte");
	// camReceivedByteSignal = registerSignal("camReceivedByte");
	// look up primary channel for CA
	ChannelNumber mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);
}

void CaService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}

/*method of reciving CAM*/
void CaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");
	// std::cout << omnetpp::simTime() << ": CaService " << mTraciId << " received\n";

	Asn1PacketVisitor<vanetza::asn1::Cam> visitor;

	const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
	// emit(camReceivedByte, )
	// 受信したCAMの送信元とメッセージIDをプリント
	
	if (cam && cam->validate()) {
		CaObject obj = visitor.shared_wrapper;
		emit(scSignalCamReceived, &obj);
		/*update map of awareness object*/
		mLocalDynamicMap->updateAwareness(obj);
	}
}

void CaService::checkTriggeringConditions(const SimTime& T_now)
{
	// std::cout << T_now << "s: " << mTraciId << " checkTriggeringConditions\n";
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenCam = mGenCam;
	const SimTime& T_GenCamMin = mGenCamMin;
	const SimTime& T_GenCamMax = mGenCamMax;
	const SimTime& T_GenCamNonPeriodic = mGenCamNonPeriodic;
    const SimTime& T_GenCamDcc = genCamDcc();

	// 前の送信からT_GenCamFinalだけたったら送信する
	// デフォルトでは0.1s
    SimTime T_GenCamFinal = T_GenCamMin;
	if (mDccRestriction) {
        T_GenCamFinal = T_GenCamDcc; // dccを設定していれば、dccによって決定された送信間隔を使用
    } else if (mExponentialNonPeriodic) {
        T_GenCamFinal = T_GenCamNonPeriodic; // 指数分布による送信を設定していれば、その送信間隔を使用
        // Need to update the time for next sending
        mGenCamNonPeriodic = mGenCamMin + exponential(mExponentialMean, 0);
	}

	/* elapsed time from when last cam was sent*/
	const SimTime T_elapsed = T_now - mLastCamTimestamp;

	if (T_elapsed >= T_GenCamFinal & T_now > startUpTime) {
		if (mFixedRate || mExponentialNonPeriodic) {
			sendCam(T_now);
		} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
			// std::cout << T_now << ": " << mTraciId << " Dynamics\n";
			// std::cout << mTraciId << "send cam, interval: " << T_elapsed << "\n";
			sendCam(T_now);
			T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
			mGenCamLowDynamicsCounter = 0;
		} else if (T_elapsed >= T_GenCam) {
			// std::cout << "Time elapsed\n";
			// std::cout << mTraciId << "send cam, interval: " << T_elapsed << "\n";
			sendCam(T_now);
			if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
				T_GenCam = T_GenCamMax;
			}
		}
	}
}

bool CaService::checkHeadingDelta() const
{
	// std::cout << "  " << round(mLastCamHeading, decidegree) << "deci° -> " << round(mVehicleDataProvider->heading(), decidegree) <<"deci°\n";
	return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool CaService::checkPositionDelta() const
{
	Position::value_type lx = mLastCamPosition.x;
	Position::value_type cx = mVehicleDataProvider->position().x;
	// std::cout << "  " << round(lx, vanetza::units::si::meter) << "m -> " << round(cx, vanetza::units::si::meter) << "m\n";
	// std::cout << omnetpp::simTime() << ": ";
	// std::cout << mLastCamPosition.x << ", " << mVehicleDataProvider->position().x << '\n';
	// std::cout << distance(mLastCamPosition, mVehicleDataProvider->position()) << '\n';
	return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool CaService::checkSpeedDelta() const
{
	// std::cout << "  " << round(mLastCamSpeed, centimeter_per_second) * SpeedValue_oneCentimeterPerSec << "cm/s -> " << round( mVehicleDataProvider->speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec << "cm/s\n";
	return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void CaService::sendCam(const SimTime& T_now)
{
	// camを生成
	// if (mTraciId == "2.0") return;

	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod, mCount);

	// camのフローIDと参照時刻をプリント
	// std::cout << "cam " << mTraciId << ": " << mVehicleDataProvider->updated() << "\n";
	// std::cout << "re: " << mTimer->getTimeFor(mTimer->reconstructMilliseconds(genDeltaTimeMod)) << "\n";

	// std::cout << mTraciId << ": " << omnetpp::simTime() << " send cam\n";

	mLastCamPosition = mVehicleDataProvider->position();
	mLastCamSpeed = mVehicleDataProvider->speed();
	mLastCamHeading = mVehicleDataProvider->heading();
	mLastCamTimestamp = T_now;
	
	/*condition of sending low frequency container*/
	if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		addLowFrequencyContainer(cam, par("pathHistoryLength"));
		mLastLowCamTimestamp = T_now;
		// std::cout << "low\n";
	}

	// 常にパスヒストリーを追加することでcamのバイトサイズを調節
	// addLowFrequencyContainer(cam, par("pathHistoryLength"));

	// BTP層へのリクエストを作成 リクエストにはポートや優先度などをセット
	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CAM;
	request.gn.its_aid = aid::CA;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2)); // 優先度DP2
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;


	// camからCaobjectを生成し、送信シグナルを出す	
	CaObject obj(std::move(cam));
	
	emit(scSignalCamSent, &obj);

	emit(camGenerationTimeSignal, mVehicleDataProvider->updated());
	// emit(camSourceIdSignal, std::hash<std::string>()(mTraciId));
	emit(camSourceIdSignal, std::stol(mTraciId));
	// // 送信するcamの送信時間、位置、速度、向きをプリント
	// std::cout << omnetpp::simTime() << "s, ";
	// std::cout << mTraciId << ", " << round(mVehicleDataProvider->position().x, vanetza::units::si::meter) << "m\n";
	// // generationDeltaTimeには、車両の位置を更新した時間=camに載せる位置を参照した時間[ms]が含まれる 16bit長(最大65534)
	// std::cout << "  time: " << obj.asn1()->cam.generationDeltaTime << "ms, ";
	// // camに載せた経度[東を正]と緯度[北を正] [micro degree] を表示 舞台はモンゴル
	// std::cout << "pos: " << obj.asn1()->cam.camParameters.basicContainer.referencePosition.longitude << "μ° " << obj.asn1()->cam.camParameters.basicContainer.referencePosition.latitude << "μ° , ";
	// // camに載せた車両の進行方向(0~3600 deci°)を表示
	// std::cout << "heading: " << obj.asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue << "deci°, "; 
	// // camに載せた速度(cm/sec)を表示
	// std::cout << "speed: " << obj.asn1()->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue << "cm/sec \n";  
    
	// 送信するCAMの送信元とメッセージIDをプリント
    // std::cout << omnetpp::simTime() << ": " << mTraciId << " send\n";
	
	// // BTP層へペイロードを送信
	using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);

	// // CAMの送信元のsumoのIDと、バイトサイズをプリント
	// payload->layer(OsiLayer::Application).size()
	// std::cout << omnetpp::simTime() << ": cam [src: " << mTraciId << ", byte: " << payload->size() << "]\n\n";
	
	// payload->setByteLength(350);
	// emit(camByteSignal, payload->size());
	
	this->request(request, std::move(payload), mNetworkInterfaceTable->select(0).get());
	mCount++;

	// パケットをプリント
	// PacketPrinter printer; // turns packets into human readable strings
	// printer.printPacket(std::cout, packet); // print to standard output

	// std::cout << "Time: " << omnetpp::simTime() << "s, ";
	// printf("CAM [src %d, Byte: %d]\n", mVehicleDataProvider->station_id(), sizeof(obj));
	// if (mTraciId == "26.0") std::cout << omnetpp::simTime() << ":   cam\n";
}

SimTime CaService::genCamDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	// vanetza::Clock::duration delay = trc->delay(ca_tx);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	// SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(delay).count(), SIMTIME_MS };
	 SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	// std::cout << "dcc interval: " << dcc << "\n";
	return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
}

// CAMを生成して値を格納し、返すメソッド
vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime, long mCount)
{
	vanetza::asn1::Cam message;

	// ヘッダ
	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 1;
	header.messageID = ItsPduHeader__messageID_cam;
	header.stationID = vdp.station_id();
	header.count = mCount;

	CoopAwareness_t& cam = (*message).cam;
	cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;

	// Basicコンテナ
	BasicContainer_t& basic = cam.camParameters.basicContainer;
	// HF コンテナ
	HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	// 緯度と経度をセット
	basic.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	// 追加
	basic.posX = vdp.position().x.value();
	basic.posY = vdp.position().y.value();

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;

	// HFコンテナ内のbasicVehicleコンテナ
	// 進行方向をセット
	bvc.heading.headingValue = round(vdp.heading(), decidegree); // デシ度（0~3600）
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	// 速度をセット　cm/s
	bvc.speed.speedValue = round(vdp.speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	// 	車両が前進かバックかをセット
	bvc.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	// 縦方向の加速度をセット
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	// 車両の曲率をセット
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;

	// 車両の曲率計算モード
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	// 車両のヨーレートをセット
	bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (abs(bvc.yawRate.yawRateValue) >= YawRateValue_unavailable) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	// 車両の長さ
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	// 車両の幅
	bvc.vehicleWidth = VehicleWidth_unavailable;

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
	}

	return message;
}

// CAMとパスヒストリーの長さを引数とする
void addLowFrequencyContainer(vanetza::asn1::Cam& message, unsigned pathHistoryLength)
{
	// パスヒストリーが40を超える場合、エラー
	if (pathHistoryLength > 40) {
        EV_WARN << "path history can contain 40 elements at maximum";
        pathHistoryLength = 40;
    }

	LowFrequencyContainer_t*& lfc = message->cam.camParameters.lowFrequencyContainer;
	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
	// 車両の役割
	bvc.vehicleRole = VehicleRole_default;
	// 車両の外灯
	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	assert(nullptr != bvc.exteriorLights.buf);
	bvc.exteriorLights.size = 1;
	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);
	// TODO: add pathHistory
	// 追加；パスヒストリーを指定したサイズで追加、パスデルタタイムのみ値を入れ、他の中身はunavalidable
	for (unsigned i = 0; i < pathHistoryLength; ++i) {
			PathPoint* pathPoint = vanetza::asn1::allocate<PathPoint>();
			pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
			*(pathPoint->pathDeltaTime) = (i + 1) * PathDeltaTime_tenMilliSecondsInPast * 10;
			pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
			pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
			pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
			ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
		}

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
	}
}

} // namespace artery
