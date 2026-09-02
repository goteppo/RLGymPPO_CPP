#pragma once
#include "StateSetter.h"

namespace RLGSC {
	class RandomState : public StateSetter {
	public:
		RandomState() {}

		virtual GameState ResetState(Arena* arena);
	};
}