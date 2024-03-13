#include "PlayerData.h"

namespace RLGSC {
	void PlayerData::UpdateFromCar(Car* car, uint64_t tickCount, int tickSkip) {
		carId = car->id;
		team = car->team;
		carState = car->GetState();
		phys = PhysObj(carState);
		physInv = PhysObj(phys.Invert());

		ballTouchedStep = carState.ballHitInfo.tickCountWhenHit >= (tickCount - tickSkip);
		ballTouchedTick = carState.ballHitInfo.tickCountWhenHit == (tickCount - 1);

		hasFlip =
			!carState.isOnGround &&
			!carState.hasDoubleJumped && !carState.hasFlipped
			&& carState.airTimeSinceJump < RLConst::DOUBLEJUMP_MAX_DELAY;

		boostFraction = carState.boost / 100;
	}
}