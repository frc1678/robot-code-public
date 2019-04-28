#pragma once

namespace ctre {
namespace phoenix {
/** signals namespace */
namespace signals {

/**
 * Interface for invertable objects
 */
class IInvertable {
public:
	virtual ~IInvertable(){}
	/**
	 * Inverts the output of the object
	 * 
	 * @param invert
	 *            Invert state to set.
	 */
	virtual void SetInverted(bool invert) = 0;
	/**
	 * @return invert setting of output.
	 */
	virtual bool GetInverted() const = 0;
};

} // namespace  Signals
} // namespace phoenix
} // namespace ctre
