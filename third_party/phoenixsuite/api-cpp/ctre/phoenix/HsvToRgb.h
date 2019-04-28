#pragma once

namespace ctre {
namespace phoenix {

/**
 * Class to convert Hue, Saturation, Value to Red, Green, Blue
 */
class HsvToRgb {
public:
/**
 * Converts HSV to RGB
 * @param hDegrees hue value
 * @param S saturation value
 * @param V value Lightness value
 * @param r pointer to red value
 * @param g pointer to green value
 * @param b pointer to blue value
 */
	static void Convert(double hDegrees, double S, double V, float* r, float* g,
			float* b);
};

}
}
