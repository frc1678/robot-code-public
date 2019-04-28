#pragma once

namespace ctre {
namespace phoenix {
	
/**
 * Class with basic utility methods
 */
class Utilities {
public:
	/**
	 * Calculates absolute value of f
	 * @param f value to calculate absolute value of
	 * @return absolute value of f
	 */
	static float abs(float f);
	/**
	 * Caps the value
	 * @param value Value to cap
	 * @param capValue Maximum/-Minimum value can be
	 * @return Capped value
	 */
	static float bound(float value, float capValue = 1);
	/**
	 * Caps the value
	 * @param value Value to cap
	 * @param peak Maximum/-Minimum value can be
	 * @return Capped value
	 */
	static float cap(float value, float peak);
	/**
	 * Deadbands the value
	 * @param value reference of value to deadband
	 * @param deadband If abs of value is under this, it will be brought to 0
	 */
	static void Deadband(float &value, float deadband = -.10f);
	/**
	 * Calculates if value is within a delta
	 * @param value initial value to compare
	 * @param compareTo value to compare against first value
	 * @param allowDelta the range value can be away from compareTo
	 * @return true if value is within allowDelta range of compareTo
	 */
	static bool IsWithin(float value, float compareTo, float allowDelta);
	/**
	 * Gets the minimum of the two values
	 * @param value_1 first value
	 * @param value_2 second value
	 * @return minimum value
	 */
	static int SmallerOf(int value_1, int value_2);
	/**
	 * Converts forward and turn to left and right speeds
	 * @param forward forward value of robot
	 * @param turn turn value of robot
	 * @param left Pointer to the left speed of robot, this will be filled
	 * @param right Pointer to the right speed of robot, this will be filled
	 */
	static void Split_1(float forward, float turn, float *left, float *right);
	/**
	 * Converts left and right to forward and turn speeds
	 * @param left left speed of robot
	 * @param right right speed of robot
	 * @param forward Pointer to the forward speed of the robot, this will be filled
	 * @param turn Pointer to the turn speed of the robot, this will be filled
	 */
	static void Split_2(float left, float right, float *forward, float *turn);
private:
	static bool Contains(char array[], char item);
};

}}
