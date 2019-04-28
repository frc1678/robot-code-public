#pragma once

namespace ctre {
namespace phoenix {
namespace signals {

/**
 * Interface for output signals
 */
class IOutputSignal {
public:
	virtual ~IOutputSignal(){}
	/**
	 * Set signal
	 * @param value value to set
	 */
	virtual void Set(double value) = 0;
};

} // namespace  Signals
} // namespace phoenix
} // namespace ctre
