#pragma once

#include <chrono>
#include <cstdint>

namespace ctre {
namespace phoenix {

class Stopwatch {
public:
	void Start();
	long long int DurationMs();
	double Duration();
	
private:
	std::chrono::steady_clock::time_point _t0 = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point _t1 = std::chrono::steady_clock::now();
};

}}
