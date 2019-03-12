#ifndef ARTERY_MODE4MOBILITY_H_JFWG67L1
#define ARTERY_MODE4MOBILITY_H_JFWG67L1

#include "artery/traci/MobilityBase.h"
#include <veins/base/modules/BaseMobility.h>
#include <veins/base/utils/Coord.h>

namespace lte
{

class Mode4Mobility : public BaseMobility /* Mode4 */, public MobilityBase /* Artery */
{
public:
    void initialize(int stage) override;

private:
    void initialize(const Position&, Angle, double speed) override;
    void update(const Position&, Angle, double speed) override;

    Coord mPosition;
    Coord mDirection;
    double mSpeed;
};

} // namespace artery

#endif /* ARTERY_MODE4MOBILITY_H_JFWG67L1 */

