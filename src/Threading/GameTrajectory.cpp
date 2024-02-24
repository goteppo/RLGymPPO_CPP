#include "GameTrajectory.h"

namespace RLGPC {

	void GameTrajectory::Append(GameTrajectory& other) {
		if (size == 0) {
			// No concat needed
			*this = other;
			return;
		}

		size_t oldSize = size;

		for (auto itr1 = data.begin(), itr2 = other.data.begin(); itr1 != data.end(); itr1++, itr2++) {
			auto ourT = itr1->slice(0, 0, size); // Remove our capacity
			auto otherT = *itr2; // Include capacity
			*itr1 = TorchFuncs::ConcatSafe(ourT, otherT);
		}

		size = oldSize + other.size;
		capacity = oldSize + other.capacity;

		assert(data.begin()->size(0) == capacity);
	}

	void GameTrajectory::RemoveCapacity() {
		if (capacity > size)
			for (torch::Tensor& t : data)
				t = t.slice(0, 0, size);
	}

	void GameTrajectory::AppendSingleStep(TrajectoryTensors step) {

		if (size > 0) {
			if (size == capacity)
				DoubleReserve();

			torch::Tensor indexTensor = torch::tensor((int64_t)size);
			for (auto itr1 = data.begin(), itr2 = step.begin(); itr1 != data.end(); itr1++, itr2++) {
				torch::Tensor& t = *itr1;
				t.index_copy_(0, indexTensor, itr2->unsqueeze(0));
			}

			size++;
		} else {
			for (auto itr1 = data.begin(), itr2 = step.begin(); itr1 != data.end(); itr1++, itr2++) {
				*itr1 = itr2->unsqueeze(0);
			}

			size = capacity = 1;
		}
	}

	void GameTrajectory::DoubleReserve() {
		if (capacity > 0) {
			for (torch::Tensor& t : data) {
				// TODO: This is annoying, is there a better way to do this?
				//	t.repeat() requires you specify the amount for all dimensions
				auto repeatSize = std::vector<int64_t>(t.dim(), 1);
				repeatSize[0] = 2;
				t = t.repeat(repeatSize);
			}

			capacity *= 2;
		}
	}
}