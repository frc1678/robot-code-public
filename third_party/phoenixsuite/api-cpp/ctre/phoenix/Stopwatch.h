#pragma once

#include <chrono>
#include <cstdint>

namespace ctre {
namespace phoenix {

/**
 * Stopwatch to track time in milliseconds
 */
class Stopwatch {
public:
	/**
	 * Start the stopwatch
	 */
	void Start();
	/**
	 * @return Current time elapsed since start in ms
	 */
	long long int DurationMs();
	/**
	 * @return Current time elapsed since start in s
	 */
	double Duration();
	
private:
	std::chrono::steady_clock::time_point _t0 = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point _t1 = std::chrono::steady_clock::now();
};

}}
