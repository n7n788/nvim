//
// Artery V2X Simulation Framework
// Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.envmod;

import artery.envmod.GlobalEnvironmentModel;
import artery.envmod.LocalEnvironmentModel;
import artery.inet.World;
// 追加
import artery.lte.Mode4World;

import artery.utility.IdentityRegistry;

// network World extends artery.inet.World
// 追加
network World extends artery.lte.Mode4World
{
    parameters:
        **.globalEnvironmentModule = default("environmentModel");

    submodules:
        environmentModel: GlobalEnvironmentModel {
            parameters:
                @display("p=140,20");
                identityRegistryModule = default("idRegistry");
                traciModule = default("traci");
                nodeMobilityModule = default(".mobility");
        }

        idRegistry: IdentityRegistry {
            parameters:
                @display("p=180,20");
        }
}
