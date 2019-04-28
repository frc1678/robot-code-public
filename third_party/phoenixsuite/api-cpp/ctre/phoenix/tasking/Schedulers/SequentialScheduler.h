#pragma once

#include <vector>
#include "ctre/phoenix/tasking/ILoopable.h"
#include "ctre/phoenix/tasking/IProcessable.h"

namespace ctre { namespace phoenix { namespace tasking { namespace schedulers {

/**
 * Scheduler that will run its ILoopables in sequence
 */
class SequentialScheduler: public ILoopable, public IProcessable{
public:
	/** should be private */
	bool _running = false;
	/** should be private */
	std::vector<ILoopable*> _loops;
	/** should be private */
	unsigned int _idx = 0;
	/** should be private */
	bool _iterated = false;

	SequentialScheduler();
	virtual ~SequentialScheduler();

	/**
	 * Add ILoopable to scheduler
	 * @param aLoop ILoopable to add
	 */
	void Add(ILoopable *aLoop);
	/**
	 * Get the currently running ILoopable
	 * @return null, not implemented
	 */
	ILoopable * GetCurrent();
	/**
	 * Remove all ILoopables
	 */
	void RemoveAll();
	/**
	 * Start next ILoopable
	 */
	void Start();
	/**
	 * Stop every ILoopable
	 */
	void Stop();

	//IProcessable
	/**
	 * Process the currently active ILoopable
	 * 
	 * Call this every loop
	 */
	void Process();

	//ILoopable
	/**
	 * Start next ILoopable
	 */
	void OnStart();
	/**
	 * Process currently active ILoopable
	 */
	void OnLoop();
	/**
	 * Stop all ILoopables
	 */
	void OnStop();
	/**
	 * @return true when no longer running
	 */
	bool IsDone();
};
}}}}
