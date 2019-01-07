#pragma once

#include <cstdint>
#include "ctre/phoenix/CANBusAddressable.h"
#include "ctre/phoenix/CustomParamConfiguration.h"
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/CANifierControlFrame.h"
#include "ctre/phoenix/CANifierStatusFrame.h"
#include "ctre/phoenix/CANifierStickyFaults.h"
#include "ctre/phoenix/CANifierFaults.h"
#include "ctre/phoenix/CANifierVelocityMeasPeriod.h"

namespace ctre {namespace phoenix {
	
	/**
	 * CTRE CANifier
	 *
	 * Device for interfacing common devices to the CAN bus.
	 */

struct CANifierConfiguration : CustomParamConfiguration{
    CANifierVelocityMeasPeriod velocityMeasurementPeriod;
	int velocityMeasurementWindow;
    bool clearPositionOnLimitF;
    bool clearPositionOnLimitR;
    bool clearPositionOnQuadIdx;
	CANifierConfiguration() : 
		velocityMeasurementPeriod(Period_100Ms), 
		velocityMeasurementWindow(64), 	
	    clearPositionOnLimitF(false),
        clearPositionOnLimitR(false),
        clearPositionOnQuadIdx(false)
    {
	}

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {

        std::string retstr = prependString + ".velocityMeasurementPeriod = " + CANifierVelocityMeasPeriodRoutines::toString(velocityMeasurementPeriod) + ";\n";
        retstr += prependString + ".velocityMeasurementWindow = " + std::to_string(velocityMeasurementWindow) + ";\n";
        retstr += prependString + ".clearPositionOnLimitF = " + std::to_string(clearPositionOnLimitF) + ";\n";
        retstr += prependString + ".clearPositionOnLimitR = " + std::to_string(clearPositionOnLimitR) + ";\n";
        retstr += prependString + ".clearPositionOnQuadIdx = " + std::to_string(clearPositionOnQuadIdx) + ";\n";
        
        retstr += CustomParamConfiguration::toString(prependString);

        return retstr;
    }

};// struct CANifierConfiguration

struct CANifierConfigUtils {
private:
	static CANifierConfiguration _default;
public:
	static bool VelocityMeasurementPeriodDifferent (const CANifierConfiguration & settings) { return (!(settings.velocityMeasurementPeriod == _default.velocityMeasurementPeriod)) || !settings.enableOptimizations; }
	static bool VelocityMeasurementWindowDifferent (const CANifierConfiguration & settings) { return (!(settings.velocityMeasurementWindow == _default.velocityMeasurementWindow)) || !settings.enableOptimizations; }
	static bool ClearPositionOnLimitFDifferent (const CANifierConfiguration & settings) { return (!(settings.clearPositionOnLimitF == _default.clearPositionOnLimitF)) || !settings.enableOptimizations; }
	static bool ClearPositionOnLimitRDifferent (const CANifierConfiguration & settings) { return (!(settings.clearPositionOnLimitR == _default.clearPositionOnLimitR)) || !settings.enableOptimizations; }
	static bool ClearPositionOnQuadIdxDifferent (const CANifierConfiguration & settings) { return (!(settings.clearPositionOnQuadIdx == _default.clearPositionOnQuadIdx)) || !settings.enableOptimizations; }
	static bool CustomParam0Different (const CANifierConfiguration & settings) { return (!(settings.customParam0 == _default.customParam0)) || !settings.enableOptimizations; }
	static bool CustomParam1Different (const CANifierConfiguration & settings) { return (!(settings.customParam1 == _default.customParam1)) || !settings.enableOptimizations; }
};


class CANifier: public CANBusAddressable {
public:
	/**
	 * Enum for the LED Output Channels
	 */
	enum LEDChannel {
		LEDChannelA = 0, LEDChannelB = 1, LEDChannelC = 2,
	};

	/**
	 * Enum for the PWM Input Channels
	 */
	enum PWMChannel {
		PWMChannel0 = 0, PWMChannel1 = 1, PWMChannel2 = 2, PWMChannel3 = 3,
	};
	const int PWMChannelCount = 4;

	/**
	 * General IO Pins on the CANifier
	 */
	enum GeneralPin {
		QUAD_IDX = 0,	//----- Must match CANifier_CCI enums -----//
		QUAD_B = 1,
		QUAD_A = 2,
		LIMR = 3,
		LIMF = 4,
		SDA = 5,
		SCL = 6,
		SPI_CS = 7,
		SPI_MISO_PWM2P = 8,
		SPI_MOSI_PWM1P = 9,
		SPI_CLK_PWM0P = 10,
	};

	/**
	 * Structure to hold the pin values.
	 */
	struct PinValues {
		bool QUAD_IDX;
		bool QUAD_B;
		bool QUAD_A;
		bool LIMR;
		bool LIMF;
		bool SDA;
		bool SCL;
		bool SPI_CS_PWM3;
		bool SPI_MISO_PWM2;
		bool SPI_MOSI_PWM1;
		bool SPI_CLK_PWM0;
	};

	/**
	 * Constructor.
	 * @param deviceNumber	The CAN Device ID of the CANifier.
	 */
	CANifier(int deviceNumber);
    
    ~CANifier();

    static void DestroyAllCANifiers();
	
	/**
	 * Sets the LED Output
	 * @param percentOutput Output duty cycle expressed as percentage.
	 * @param ledChannel Channel to set the output of.
	 */
    ErrorCode SetLEDOutput(double percentOutput, LEDChannel ledChannel);
	/**
	 * Sets the output of a General Pin
	 * @param outputPin 		The pin to use as output.
	 * @param outputValue 	The desired output state.
	 * @param outputEnable	Whether this pin is an output. "True" enables output.
	 */
	ErrorCode SetGeneralOutput(GeneralPin outputPin, bool outputValue, bool outputEnable);
	/**
	 * Sets the output of all General Pins
	 * @param outputBits 	A bit mask of all the output states.  LSB->MSB is in the order of the #GeneralPin enum.
	 * @param isOutputBits A boolean bit mask that sets the pins to be outputs or inputs.  A bit of 1 enables output.
	 */
	ErrorCode SetGeneralOutputs(int outputBits, int isOutputBits);
	/**
	 * Gets the state of all General Pins
	 * @param allPins A structure to fill with the current state of all pins.
	 */
	ErrorCode GetGeneralInputs(PinValues &allPins);
	/**
	 * Gets the state of the specified pin
	 * @param inputPin  The index of the pin.
	 * @return The state of the pin.
	 */
	bool GetGeneralInput(GeneralPin inputPin);
	/**
	 * Gets the quadrature encoder's position
	 * @return Position of encoder 
	 */
	int GetQuadraturePosition();
	/**
	 * Gets the quadrature encoder's velocity
	 * @return Velocity of encoder
	 */
	int GetQuadratureVelocity();
	/**
	 * Sets the quadrature encoder's position
	 * @param newPosition  Position to set
	 * @param timeoutMs  
					Timeout value in ms. If nonzero, function will wait for
					config success and report an error if it times out.
					If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode SetQuadraturePosition(int newPosition, int timeoutMs = 0);
	/**
	 * Configures the period of each velocity sample.
	 * Every 1ms a position value is sampled, and the delta between that sample
	 * and the position sampled kPeriod ms ago is inserted into a filter.
	 * kPeriod is configured with this function.
	 *
	 * @param period Desired period for the velocity measurement.
	 * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode ConfigVelocityMeasurementPeriod(
			CANifierVelocityMeasPeriod period, int timeoutMs = 0);
	/**
	 * Sets the number of velocity samples used in the rolling average velocity
	 * measurement.
	 *
	 * @param windowSize Number of samples in the rolling average of velocity
	 *            measurement. Valid values are 1,2,4,8,16,32. If another
	 *            value is specified, it will truncate to nearest support value.
	 * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode ConfigVelocityMeasurementWindow(int windowSize, int timeoutMs = 0);
    /**
     * Enables clearing the position of the feedback sensor when the forward 
     * limit switch is triggered
     *
     * @param clearPositionOnLimitF     Whether clearing is enabled, defaults false
     * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
	ErrorCode ConfigClearPositionOnLimitF (bool clearPositionOnLimitF, int timeoutMs = 0);
    /**
     * Enables clearing the position of the feedback sensor when the reverse 
     * limit switch is triggered
     *
     * @param clearPositionOnLimitR     Whether clearing is enabled, defaults false
     * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
	ErrorCode ConfigClearPositionOnLimitR (bool clearPositionOnLimitR, int timeoutMs = 0);
    /**
     * Enables clearing the position of the feedback sensor when the quadrature index signal
     * is detected
     *
     * @param clearPositionOnQuadIdx    Whether clearing is enabled, defaults false
     * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
	ErrorCode ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx, int timeoutMs = 0);
	/**
	 * Gets the bus voltage seen by the device.
	 *
	 * @return The bus voltage value (in volts).
	 */
	double GetBusVoltage();
	/**
	 * Call GetLastError() generated by this object.
	 * Not all functions return an error code but can
	 * potentially report errors.
	 *
	 * This function can be used to retrieve those error codes.
	 *
	 * @return The last ErrorCode generated.
	 */
	ErrorCode GetLastError();
	/**
	 * Sets the PWM Output
	 * Currently supports PWM 0, PWM 1, and PWM 2
	 * @param pwmChannel  Index of the PWM channel to output.
	 * @param dutyCycle   Duty Cycle (0 to 1) to output.  Default period of the signal is 4.2 ms.
	 */
	ErrorCode SetPWMOutput(int pwmChannel, double dutyCycle);
	/**
	 * Enables PWM Outputs
	 * Currently supports PWM 0, PWM 1, and PWM 2
	 * @param pwmChannel  Index of the PWM channel to enable.
	 * @param bEnable			"True" enables output on the pwm channel.
	 */
	ErrorCode EnablePWMOutput(int pwmChannel, bool bEnable);
	/**
	 * Gets the PWM Input
	 * @param pwmChannel  PWM channel to get.
	 * @param pulseWidthAndPeriod	Double array to hold Duty Cycle [0] and Period [1].
	 */
	ErrorCode GetPWMInput(PWMChannel pwmChannel, double pulseWidthAndPeriod[]);

	//------ Custom Persistent Params ----------//
	/**
	 * Sets the value of a custom parameter. This is for arbitrary use.
   *
   * Sometimes it is necessary to save calibration/duty cycle/output
   * information in the device. Particularly if the
   * device is part of a subsystem that can be replaced.
	 *
	 * @param newValue  Value for custom parameter.
	 * @param paramIndex  Index of custom parameter. [0-1]
	 * @param timeoutMs  Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode ConfigSetCustomParam(int newValue, int paramIndex,
			int timeoutMs = 0);
	/**
	 * Gets the value of a custom parameter. This is for arbitrary use.
   *
   * Sometimes it is necessary to save calibration/duty cycle/output
   * information in the device. Particularly if the
   * device is part of a subsystem that can be replaced.
	 *
	 * @param paramIndex Index of custom parameter. [0-1]
	 * @param timeoutMs  Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Value of the custom param.
	 */
	int ConfigGetCustomParam(int paramIndex,
			int timeoutMs = 0);
	//------ Generic Param API, typically not used ----------//
	/**
	 * Sets a parameter. Generally this is not used.
   * This can be utilized in
   * - Using new features without updating API installation.
   * - Errata workarounds to circumvent API implementation.
   * - Allows for rapid testing / unit testing of firmware.
	 *
	 * @param param Parameter enumeration.
	 * @param value Value of parameter.
	 * @param subValue  Subvalue for parameter. Maximum value of 255.
	 * @param ordinal  Ordinal of parameter.
	 * @param timeoutMs  Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode ConfigSetParameter(ParamEnum param, double value,
			uint8_t subValue, int ordinal, int timeoutMs = 0);
	/**
	 * Gets a parameter. Generally this is not used.
   * This can be utilized in
   * - Using new features without updating API installation.
   * - Errata workarounds to circumvent API implementation.
   * - Allows for rapid testing / unit testing of firmware.
	 *
	 * @param param Parameter enumeration.
	 * @param ordinal  Ordinal of parameter.
	 * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Value of parameter.
	 */
	double ConfigGetParameter(ParamEnum param, int ordinal, int timeoutMs = 0);
    
    ErrorCode ConfigGetParameter(ParamEnum param, int32_t valueToSend,
            int32_t & valueReceived, uint8_t & subValue, int32_t ordinal,
            int32_t timeoutMs);


	/**
	 * Sets the period of the given status frame.
	 *
	 * @param statusFrame  Frame whose period is to be changed.
	 * @param periodMs Period in ms for the given frame.
	 * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode SetStatusFramePeriod(CANifierStatusFrame statusFrame,
			uint8_t periodMs, int timeoutMs = 0);
	/**
	 * Gets the period of the given status frame.
	 *
	 * @param frame  Frame to get the period of.
	 * @param timeoutMs  Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
	 * @return Period of the given status frame.
	 */
	int GetStatusFramePeriod(CANifierStatusFrame frame, int timeoutMs = 0);
	/**
	 * Sets the period of the given control frame.
	 *
	 * @param frame Frame whose period is to be changed.
	 * @param periodMs Period in ms for the given frame.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode SetControlFramePeriod(CANifierControlFrame frame, int periodMs);
	/**
	 * Gets the firmware version of the device.
	 *
	 * @return Firmware version of device.
	 */
	int GetFirmwareVersion();
	/**
	 * Returns true if the device has reset since last call.
	 *
	 * @return Has a Device Reset Occurred?
	 */
	bool HasResetOccurred();
	/**
	 * Gets the CANifier fault status
	 *
	 * @param toFill
	 *            Container for fault statuses.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode GetFaults(CANifierFaults & toFill);
	/**
	 * Gets the CANifier sticky fault status
	 *
	 * @param toFill
	 *            Container for sticky fault statuses.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode GetStickyFaults(CANifierStickyFaults & toFill);
	/**
	 * Clears the Sticky Faults
	 * 
	 * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
	 *
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode ClearStickyFaults(int timeoutMs = 0);
	
	//------ All Configs ----------//
    /**
     * Configures all peristant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     *
     * @return Error Code generated by function. 0 indicates no error. 
     */
    ctre::phoenix::ErrorCode ConfigAllSettings(const CANifierConfiguration &allConfigs, int timeoutMs = 50);
    /**
     * Gets all persistant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
    void GetAllConfigs(CANifierConfiguration &allConfigs, int timeoutMs = 50);
    /**
     * Configures all peristant settings to defaults (overloaded so timeoutMs is 50 ms).
	 * 
	 * @param timeoutMs Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     *
     * @return Error Code generated by function. 0 indicates no error. 
     */
    ErrorCode ConfigFactoryDefault(int timeoutMs = 50);


private:
	void* m_handle;
	bool _tempPins[11];
};// class CANifier 

} // namespace phoenix
} // namespace ctre
