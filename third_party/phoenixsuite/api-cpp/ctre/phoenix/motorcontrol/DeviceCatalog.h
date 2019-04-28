#pragma once

#include "IMotorController.h"
#include <vector>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Class to keep track of multiple devices
 */
class DeviceCatalog {
public:
	/**
	 * Add motor controller to catalog
	 * @param motorController motorController to add
	 */
	void Register(IMotorController *motorController) {
		_mcs.push_back(motorController);
	}

	/**
	 * @return count of motor controllers in catalog
	 */
	size_t MotorControllerCount() {
		return _mcs.size();
	}

	/**
	 * Get motor controller at index
	 * @param idx index of motor controller in catalog
	 * @return motor controller at specified index
	 */
	IMotorController* Get(int idx) {
		return _mcs[idx];
	}

	/**
	 * @return static instance of deviceCatalog
	 */
	DeviceCatalog & GetInstance() {
		if (!_instance)
			_instance = new DeviceCatalog();
		return *_instance;
	}
private:
	std::vector<IMotorController*> _mcs;

	static DeviceCatalog * _instance;
};

}
} // namespace phoenix
}

