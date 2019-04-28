#pragma once

#include "IMotorController.h"
#include <vector>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Group of motor controllers
 */
class GroupMotorControllers {
public:
	/**
	 * Add motor controller to the group
	 * @param motorController motor controller to add
	 */
	static void Register(IMotorController *motorController);
	/**
	 * @return number of motorcontrollers in group
	 */
	static size_t MotorControllerCount();
	/**
	 * @param idx Index of motor controller to get
	 * @return Motor controller at specified index
	 */
	static IMotorController* Get(int idx);

private:
	static std::vector<IMotorController*> _mcs;
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre

