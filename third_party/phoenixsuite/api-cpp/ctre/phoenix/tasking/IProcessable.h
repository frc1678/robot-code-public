#pragma once
namespace ctre { namespace phoenix { namespace tasking{
	
/**
 * Interface for processable objects
 */
class IProcessable {
public:
	virtual ~IProcessable(){}
	/**
	 * This function is called when a process occurs
	 */
	virtual void Process() = 0;
};
}}}
