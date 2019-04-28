#pragma once

#include <vector>
#include "ctre/phoenix/tasking/ILoopable.h"
#include "ctre/phoenix/tasking/IProcessable.h"

namespace ctre {
namespace phoenix {
/** tasking namespace */
namespace tasking {
/** schedulers namespace */
namespace schedulers {

/**
 * Scheduler that wil run its ILoopables in concurrency
 */
class ConcurrentScheduler: public ILoopable, public IProcessable {
public:
	/** Should be private */
	std::vector<ILoopable*> _loops;
	/** Should be private */
	std::vector<bool> _enabs;

	ConcurrentScheduler();
	virtual ~ConcurrentScheduler();
	/**
	 * Add ILoopable to schedule
	 * @param aLoop ILoopable to add to schedule
	 * @param enable Whether to enable ILoopable
	 */
	void Add(ILoopable *aLoop, bool enable = true);
	/**
	 * Remove all ILoopables from scheduler
	 */
	void RemoveAll();
	/**
	 * Start an ILoopable
	 * @param toStart ILoopable to start
	 */
	void Start(ILoopable *toStart);
	/**
	 * Stop an ILoopable
	 * @param toStop ILoopable to stop
	 */
	void Stop(ILoopable *toStop);
	/**
	 * Start all ILoopables
	 */
	void StartAll();
	/**
	 * Stop all ILoopables
	 */
	void StopAll();

	//IProcessable
	/**
	 * Process every ILoopable
	 * 
	 * Call this every loop
	 */
	void Process();

	//ILoopable
	/**
	 * @return false, this never iterates
	 */
	bool Iterated();
	/**
	 * Start all ILoopables
	 */
	void OnStart();
	/**
	 * Process all ILoopables
	 */
	void OnLoop();
	/**
	 * Stop all ILoopables
	 */
	void OnStop();
	/**
	 * @return false, this is never done
	 */
	bool IsDone();
};
}
}
}
}
