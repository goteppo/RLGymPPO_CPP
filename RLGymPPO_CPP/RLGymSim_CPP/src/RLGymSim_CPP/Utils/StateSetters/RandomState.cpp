#include "RandomState.h"
#include "../../Math.h"

Vec RandNormVec() {
	return RLGSC::Math::RandVec(Vec(-1, -1, -1), Vec(1, 1, 1)).Normalized();
}

// Exponential distribution, peak density at 0, clamped to maxVal.
// lambda controls how quickly probability falls off — larger lambda = more concentrated near 0.
// With infinite maxVal, the mean of the distribution would be 1/lambda.
float ExpRandFloat(float maxVal, float lambda) {
    float u = ::Math::RandFloat(0, 1);
	if (u == 1) {
		return maxVal;
	}
    float x = -std::log(1.0f - u) / lambda;
    return std::min(x, maxVal);
}

RLGSC::GameState RLGSC::RandomState::ResetState(Arena* arena) {
	
	// Reset boost pads and everything
	arena->ResetToRandomKickoff();

	constexpr float
		// Open field.
		X_MAX = 4096.0f,
		Y_MAX = 5120.0f,
		Z_MAX = 2044.0f,
		CORNER_CUT = 1152.0f,

		// Inside a goal.
		GOAL_X_MAX = 892.755f, // Goal width.
		GOAL_Y_MIN = 5120.0f,    // Goal line.
		GOAL_Y_MAX = 6000.0f,    // Back of the net.
		GOAL_Z_MAX = 642.775f, // Goal height.

		// Minimum distance from the walls.
		BALL_DIST_MIN         = 170.0f, // 95+75 (apprx. ball radius + bottom/top corner curvature)
		CAR_DIST_AERIAL_MIN   = 155.0f, // 80+75 (apprx. max car length from RJ + partial bottom/top corner curvature)
		CAR_DIST_GROUNDED_MIN = 336.0f, // 80+256 (apprx. max car length from RJ + full bottom/top corner curvature)

		DEEP_NET_SPAWN = 0.15f, // Probability for one of the cars to spawn inside a goal.
		GROUNDED_SPAWN = 0.50f, // Probability for a car to spawn grounded.

		PITCH_MAX = M_PI / 2,
		YAW_MAX = M_PI,
		ROLL_MAX = M_PI,
		BALL_VEL_MAX = 6000.0f,
		BALL_ANGVEL_MAX = 6.0f,
		CAR_ANGVEL_MAX = 5.5f;

	{
		// Randomize ball.
		BallState bs = {};
		do {
			// Randomize ball position.
			bs.pos = Math::RandVec(
				Vec(-(X_MAX - BALL_DIST_MIN), -(Y_MAX - BALL_DIST_MIN), BALL_DIST_MIN),
				Vec(X_MAX - BALL_DIST_MIN, Y_MAX - BALL_DIST_MIN, Z_MAX - BALL_DIST_MIN)
			);
		} while (std::abs(bs.pos.x) + std::abs(bs.pos.y) > (X_MAX + Y_MAX - CORNER_CUT - BALL_DIST_MIN));

		// Randomize linear and angular velocities of the ball.
		bs.vel = RandNormVec() * ExpRandFloat(BALL_VEL_MAX, 1.0f/RLConst::CAR_MAX_SPEED); // Mean set to supersonic speed (in uu/s).
		bs.angVel = RandNormVec() * ::Math::RandFloat(0, BALL_ANGVEL_MAX);
		
		arena->ball->SetState(bs);
	}

	bool doNetSpawn = ::Math::RandFloat(0, 1) < DEEP_NET_SPAWN;
	int netSpawnCarIndex = doNetSpawn ? (int)(::Math::RandFloat(0, 1) * arena->_cars.size()) : -1;
	bool netPositiveSide = ::Math::RandFloat(0, 1) < 0.5f; // Which goal, decided once for the episode.

	int spawnIndex = 0;
	for (Car* car : arena->_cars) { // Randomize cars
		bool onGround = ::Math::RandFloat(0, 1) < GROUNDED_SPAWN;

		CarState cs = {};
		if (spawnIndex == netSpawnCarIndex) {
			if (onGround) {
				float yMin = netPositiveSide ? GOAL_Y_MIN : -(GOAL_Y_MAX-CAR_DIST_GROUNDED_MIN);
				float yMax = netPositiveSide ? (GOAL_Y_MAX-CAR_DIST_GROUNDED_MIN) : -GOAL_Y_MIN;
				cs.pos = Vec(
					::Math::RandFloat(-(GOAL_X_MAX - CAR_DIST_GROUNDED_MIN), GOAL_X_MAX - CAR_DIST_GROUNDED_MIN),
					::Math::RandFloat(yMin, yMax),
					17
				);
			} else {
				float yMin = netPositiveSide ? GOAL_Y_MIN : -(GOAL_Y_MAX-CAR_DIST_AERIAL_MIN);
				float yMax = netPositiveSide ? (GOAL_Y_MAX-CAR_DIST_AERIAL_MIN) : -GOAL_Y_MIN;
				cs.pos = Vec(
					::Math::RandFloat(-(GOAL_X_MAX - CAR_DIST_AERIAL_MIN), GOAL_X_MAX - CAR_DIST_AERIAL_MIN),
					::Math::RandFloat(yMin, yMax),
					CAR_DIST_AERIAL_MIN + ExpRandFloat(GOAL_Z_MAX - 2*CAR_DIST_AERIAL_MIN, 1.0f/(230-CAR_DIST_AERIAL_MIN)) // Mean set to apprx. single-jump height.
				);
			}
		} else {
			if (onGround) {
				do {
					cs.pos = Vec(
						::Math::RandFloat(-(X_MAX - CAR_DIST_GROUNDED_MIN), X_MAX - CAR_DIST_GROUNDED_MIN),
						::Math::RandFloat(-(Y_MAX - CAR_DIST_GROUNDED_MIN), Y_MAX - CAR_DIST_GROUNDED_MIN),
						17
					);
				} while (std::abs(cs.pos.x) + std::abs(cs.pos.y) > (X_MAX + Y_MAX - CORNER_CUT - CAR_DIST_GROUNDED_MIN));
			} else {
				do {
					cs.pos = Vec(
						::Math::RandFloat(-(X_MAX - CAR_DIST_AERIAL_MIN), X_MAX - CAR_DIST_AERIAL_MIN),
						::Math::RandFloat(-(Y_MAX - CAR_DIST_AERIAL_MIN), Y_MAX - CAR_DIST_AERIAL_MIN),
						CAR_DIST_AERIAL_MIN + ExpRandFloat(Z_MAX - 2*CAR_DIST_AERIAL_MIN, 1.0f/(500-CAR_DIST_AERIAL_MIN)) // Mean set to apprx. double-jump height.
					);
				} while (std::abs(cs.pos.x) + std::abs(cs.pos.y) > (X_MAX + Y_MAX - CORNER_CUT - CAR_DIST_AERIAL_MIN));
			}
		}
		spawnIndex++;

		Angle angle = Angle(::Math::RandFloat(-YAW_MAX, YAW_MAX), ::Math::RandFloat(-PITCH_MAX, PITCH_MAX), ::Math::RandFloat(-ROLL_MAX, ROLL_MAX));

		if (onGround) {
			// Randomize velocity along the 2D floor.
			angle.pitch = angle.roll = 0;
			cs.vel = angle.GetForwardVec() * ExpRandFloat(RLConst::CAR_MAX_SPEED, 1.0f/1410.0f); // Mean set to top (non-boosting) driving speed.
			cs.angVel = {};
		} else {
			// Randomize linear and angular velocities of an aerial car.
			
			// Randomize the velocity along the XY-plane.
			float horizSpeed = ExpRandFloat(RLConst::CAR_MAX_SPEED, 1.0f/1410.0f); // Mean set to top (non-boosting) driving speed.
			Vec horizDir2D =  RLGSC::Math::RandVec(Vec(-1, -1, 0), Vec(1, 1, 0)).Normalized();
			
			// Randomize velocity along the Z-axis.
			float zSpeed;
			if (::Math::RandFloat(0,1) < 0.5f) { // Randomly choose upward vs downward, then sample magnitude appropriately.
				zSpeed = ExpRandFloat(737.875f, 1.0f/445.875f); // Upward, capped at max double-jump velocity.
			} else {
				zSpeed = -ExpRandFloat(806.2f, 1.0f/546.8f); // Downward, capped at terminal velocity when falling from double-jump heigth of ~500uu.
			}

			// Clamp the final velocity at maximum car speed.
			cs.vel = Vec(horizDir2D.x * horizSpeed, horizDir2D.y * horizSpeed, zSpeed);
			float totalSpeed = cs.vel.Length();
			if (totalSpeed > RLConst::CAR_MAX_SPEED) {
				cs.vel *= RLConst::CAR_MAX_SPEED / totalSpeed;
			}

			// Randomize the angular velocity.
			cs.angVel = RandNormVec() * ::Math::RandFloat(0, CAR_ANGVEL_MAX);
		}

		cs.rotMat = angle.ToRotMat();

		cs.boost = ::Math::RandFloat(0, 100);

		car->SetState(cs);
	}

	return GameState(arena);
}
