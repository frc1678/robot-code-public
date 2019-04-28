#pragma once

namespace ctre { namespace phoenix { namespace tasking {
	
/**
 * Interface for loopable objects
 */
class ILoopable{
public:
	virtual ~ILoopable(){}
	/**
	 * Function called when object starts
	 */
	virtual void OnStart() = 0;
	/**
	 * Function called every loop
	 */
	virtual void OnLoop() = 0;
	/**
	 * @return true if object is done 
	 */
	virtual bool IsDone() = 0;
	/**
	 * Function called when object stops
	 */
	virtual void OnStop() = 0;
};
}}}
