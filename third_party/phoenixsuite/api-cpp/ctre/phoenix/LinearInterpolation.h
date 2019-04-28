#pragma once

namespace ctre {
namespace phoenix {
	
/**
 * Class to calculate linear interpolation
 */
class LinearInterpolation {
public:
	/**
	 * Calculates the linear interpolation of x
	 * @param x value to interpolate between
	 * @param x1 min x value
	 * @param y1 y value that corresponds to x1
	 * @param x2 max x value
	 * @param y2 y value that corresponds to x2
	 * @return interpolated point
	 */
	static float Calculate(float x, float x1, float y1, float x2, float y2);
};

}}

