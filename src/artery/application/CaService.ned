//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package artery.application;

simple CaService like ItsG5Service
{
    parameters:
        @signal[CamReceived](type=CaObject);
        @signal[CamSent](type=CaObject);

        @statistic[reception](source=CamReceived;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);
        @statistic[transmission](source=CamSent;record=vector(camStationId)?,vector(camGenerationDeltaTime)?);

        // camの送信時にgenerationDeltaTimeを送信するシグナル
        @signal[camGenerationTime](type="omnetpp::SimTime");
        @statistic[camGenerationTime](title="camGenerationTime";source="camGenerationTime";record=vector);
        // camの送信時に送信元IDを送信するシグナル
        @signal[camSourceId](type="long");
        @statistic[camSourceId](title="camSourceId";source="camSourceId";record=vector);
        // camのバイト数を送信するシグナル
        // @signal[camByte](type="long");
        // @statistic[camByte](title="camByte";source="camByte";record=vector);

        // // camの受信時のバイト数を送信するシグナル
        // @signal[camReceivedByte](type="long");
        // @statistic[camReceivedByte](title="camReceivedByte";source="camReceivedByte";record=vector);
        
        // evaluate DCC transmission interval restrictions
        bool withDccRestriction = default(true);

        // generation interval boundariesS
        double minInterval @unit(s) = default(0.1s);
        double maxInterval @unit(s) = default(1.0s);

        bool exponentialNonPeriodic = default(false);
        double exponentialMean @unit(s) = default(0.05s);

        // generate at fixed rate (using minInterval, optionally restricted by DCC)
        bool fixedRate = default(false);

        // change in orientation triggering CAM generation (in degree)
        double headingDelta = default(4.0);

        // change in position triggering CAM generation
        double positionDelta @unit(m) = default(4.0m);

        // change in speed triggering CAM generation (in meter/second)
        double speedDelta @unit(mps) = default(0.5mps);

        // length of path history
        volatile int pathHistoryLength = default(36);
}
