//
// Artery V2X Simulation Framework
// Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.envmod;

simple LocalEnvironmentModel
{
    parameters:
        // //  物体認識確率を計算するためのシグナル
        // @signal[objectPerception](type="long");
        // @statistic[objectPerception](title="objectPerception"; source="objectPerception"; record=vector);

        // @signal[objectDistance](type="long");
        // @statistic[objectDistance](title="objectDistance"; source="objectDistance"; record=vector);

        // AoIを計算するためのシグナル
        // @signal[aoi](type="omnetpp::SimTime");
        // @statistic[aoi](title="aoi"; source="aoi"; record=vector);

        // @signal[perceptedObjectDistance](type="long");
        // @statistic[perceptedObjectDistance](title="perceptedObjectDistance"; source="perceptedObjectDistance"; record=vector);

        // @signal[cpmRecvCnt](type="long");
        // @statistic[cpmRecvCnt](title="cpmRecvCnt"; source="cpmRecvCnt"; record=vector);

        // 車両のSUMO IDを送信するシグナル
        @signal[id](type="long");
        @statistic[id](title="id"; source="id"; record=vector);
        // 車両のx座標
        @signal[positionX](type="double");
        @statistic[positionX](title="positionX"; source="positionX"; record=vector);
        // 車両のy座標
        @signal[positionY](type="double");
        @statistic[positionY](title="positionY"; source="positionY"; record=vector);
        // 車両の速度を送信するシグナル
        @signal[speed](type="long");
        @statistic[speed](title="speed"; source="speed"; record=vector);
        // 車両の進行方向を送信するシグナル
        @signal[heading](type="long");
        @statistic[heading](title="heading"; source="heading"; record=vector);      
        // 車両の縦加速度を送信するシグナル
        @signal[longitudinalAcceleration](type="double");
        @statistic[longitudinalAcceleration](title="longitudinalAcceleration"; source="longitudinalAcceleration"; record=vector);
        // 車両のヨーレートを送信するシグナル
        @signal[yawrate](type="double");
        @statistic[yawrate](title="yawrate"; source="yawrate"; record=vector);
        // センシングしている車両のid
        @signal[sensedVehicleId](type="long");
        @statistic[sensedVehicleId](title="sensedVehicleId"; source="sensedVehicleId"; record=vector);
        // センサ名
        @signal[sensorName](type="long");
        @statistic[sensorName](title="sensorName"; source="sensorName"; record=vector);
        // センシング情報の測定時間
        @signal[sensorMeasureTime](type="double");
        @statistic[sensorMeasureTime](title="sensorMeasureTime"; source="sensorMeasureTime"; record=vector);

        @display("i=abstract/db");
        xml sensors = default(xml("<sensors />"));
        string globalEnvironmentModule;
        string middlewareModule;
        string identityRegistryModule;

        // シミュレーション結果のログを取る範囲
        double lowerPosX = default(-10000);
        double upperPosX = default(10000);
        double lowerPosY = default(-10000);
        double upperPosY = default(10000); 
        // double measureTimeDelta @unit(s) = default(1s); // データを収集する間隔
}

