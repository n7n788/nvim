package artery.lte;

import artery.application.VehicleMiddleware;
import artery.networking.Vanetza;
import artery.nic.IRadioDriver;

import lte.stack.phy.ILteNic;

module Mode4Car extends artery.inet.PlainVehicle
{
    parameters:
        // LTE NIC
        string nicType = default("LteNicUeMode4D2D");
        int macNodeId = default(0);
        int masterId;

    gates:
        input lteRadioIn @directIn;

    submodules:
        lteNic: <nicType> like ILteNic {
            @display("p=150,400");
            nodeType = "UE";
        }

        radioDriver: RadioDriver {
            parameters:
                @display("p=250,300");
        }

        vanetza: Vanetza {
            parameters:
                @display("p=250,200");
                *.middlewareModule = absPath("^.middleware");
                *.mobilityModule = absPath("^.mobility");
                *.radioDriverModule = absPath("^.radioDriver");
                runtime.datetime = middleware.datetime;
        }

        middleware: VehicleMiddleware {
            parameters:
                @display("p=250,100");
                routerModule = ".vanetza.router";
                mobilityModule = ".mobility";
        }

    connections allowunconnected:
        lteNic.radioIn <-- lteRadioIn;

        //networkLayer.ifOut++ --> lteNic.upperLayerIn;
        //networkLayer.ifIn++ <-- lteNic.upperLayerOut;

        radioDriver.upperLayer <--> vanetza.radioDriverData;
        radioDriver.properties --> vanetza.radioDriverProperties;

        radioDriver.lowerLayerOut --> lteNic.upperLayerIn;
        radioDriver.lowerLayerIn <-- lteNic.upperLayerOut;
}
