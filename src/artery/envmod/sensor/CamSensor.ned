//
// Artery V2X Simulation Framework
// Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.envmod.sensor;

simple CamSensor
{
    parameters:
        
        // camの受信時にgenerationDeltaTimeを送信するシグナル
        @signal[camReceivedGenerationTime](type="omnetpp::SimTime");
        @statistic[camReceivedGenerationTime](title="camReceivedGenerationTime";source="camReceivedGenerationTime";record=vector);
        // camの受信時に送信元IDを送信するシグナル
        @signal[camReceivedSourceId](type="long");
        @statistic[camReceivedSourceId](title="camReceivedSourceId";source="camReceivedSourceId";record=vector);
        // camの受信時にバイト数を送信するシグナル
        // @signal[camReceivedByte](type="long");
        // @statistic[camReceivedByte](title="camReceivedByte";source="camReceivedByte";record=vector);

        string identityRegistryModule;
}
