/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 *
 * Cross The Road Electronics (CTRE) licenses to you the right to
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL,
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

#pragma once

#include <string>
#include "ctre/phoenix/CANBusAddressable.h"
#include "ctre/phoenix/CustomParamConfiguration.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/sensors/PigeonIMU_ControlFrame.h"
#include "ctre/phoenix/sensors/PigeonIMU_Faults.h"
#include "ctre/phoenix/sensors/PigeonIMU_StatusFrame.h"
#include "ctre/phoenix/sensors/PigeonIMU_StickyFaults.h"

/* forward prototype */
namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {
class TalonSRX;
}
}
}
}

namespace ctre {
namespace phoenix {
namespace sensors {

struct PigeonIMUConfiguration : CustomParamConfiguration{
	PigeonIMUConfiguration() {}

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {
        std::string retstr = CustomParamConfiguration::toString(prependString);

        return retstr;
    }
};// struct PigeonIMU

struct PigeonIMUConfigUtils {
private:
	static PigeonIMUConfiguration _default;
public:
	static bool CustomParam0Different (const PigeonIMUConfiguration & settings) { return (!(settings.customParam0 == _default.customParam0)) || !settings.enableOptimizations; }
	static bool CustomParam1Different (const PigeonIMUConfiguration & settings) { return (!(settings.customParam1 == _default.customParam1)) || !settings.enableOptimizations; }
};

/**
 * Pigeon IMU Class.
 * Class supports communicating over CANbus and over ribbon-cable (CAN Talon SRX).
 */
class PigeonIMU: public CANBusAddressable {
public:
	/** Data object for holding fusion information. */
	struct FusionStatus {
		double heading;
		bool bIsValid;
		bool bIsFusing;
		std::string description;
		/**
		 * Same as GetLastError()
		 */
		int lastError;
	};
	/** Various calibration modes supported by Pigeon. */
	enum CalibrationMode {
		BootTareGyroAccel = 0,
		Temperature = 1,
		Magnetometer12Pt = 2,
		Magnetometer360 = 3,
		Accelerometer = 5,
	};
	/** Overall state of the Pigeon. */
	enum PigeonState {
		NoComm, Initializing, Ready, UserCalibration,
	};
	/**
	 * Data object for status on current calibration and general status.
	 *
	 * Pigeon has many calibration modes supported for a variety of uses.
	 * The modes generally collects and saves persistently information that makes
	 * the Pigeon signals more accurate.  This includes collecting temperature, gyro, accelerometer,
	 * and compass information.
	 *
	 * For FRC use-cases, typically compass and temperature calibration is not required.
	 *
	 * Additionally when motion driver software in the Pigeon boots, it will perform a fast boot calibration
	 * to initially bias gyro and setup accelerometer.
	 *
	 * These modes can be enabled with the EnterCalibration mode.
	 *
	 * When a calibration mode is entered, caller can expect...
	 *
	 *  - PigeonState to reset to Initializing and bCalIsBooting is set to true.  Pigeon LEDs will blink the boot pattern.
	 *  	This is similar to the normal boot cal, however it can an additional ~30 seconds since calibration generally
	 *  	requires more information.
	 *  	currentMode will reflect the user's selected calibration mode.
	 *
	 *  - PigeonState will eventually settle to UserCalibration and Pigeon LEDs will show cal specific blink patterns.
	 *  	bCalIsBooting is now false.
	 *
	 *  - Follow the instructions in the Pigeon User Manual to meet the calibration specific requirements.
	 * 		When finished calibrationError will update with the result.
	 * 		Pigeon will solid-fill LEDs with red (for failure) or green (for success) for ~5 seconds.
	 * 		Pigeon then perform boot-cal to cleanly apply the newly saved calibration data.
	 */
	struct GeneralStatus {
		/**
		 * The current state of the motion driver.  This reflects if the sensor signals are accurate.
		 * Most calibration modes will force Pigeon to reinit the motion driver.
		 */
		PigeonIMU::PigeonState state;
		/**
		 * The currently applied calibration mode if state is in UserCalibration or if bCalIsBooting is true.
		 * Otherwise it holds the last selected calibration mode (when calibrationError was updated).
		 */
		PigeonIMU::CalibrationMode currentMode;
		/**
		 * The error code for the last calibration mode.
		 * Zero represents a successful cal (with solid green LEDs at end of cal)
		 * and nonzero is a failed calibration (with solid red LEDs at end of cal).
		 * Different calibration
		 */
		int calibrationError;
		/**
		 * After caller requests a calibration mode, pigeon will perform a boot-cal before
		 * entering the requested mode.  During this period, this flag is set to true.
		 */
		bool bCalIsBooting;
		/**
		 * general string description of current status
		 */
		std::string description;
		/**
		 * Temperature in Celsius
		 */
		double tempC;
		/**
		 * Number of seconds Pigeon has been up (since boot).
		 * This register is reset on power boot or processor reset.
		 * Register is capped at 255 seconds with no wrap around.
		 */
		int upTimeSec;
		/**
		 * Number of times the Pigeon has automatically rebiased the gyro.
		 * This counter overflows from 15 -> 0 with no cap.
		 */
		int noMotionBiasCount;
		/**
		 * Number of times the Pigeon has temperature compensated the various signals.
		 * This counter overflows from 15 -> 0 with no cap.
		 */
		int tempCompensationCount;
		/**
		 * Same as GetLastError()
		 */
		int lastError;
	};

	/**
	 * Create a Pigeon object that communicates with Pigeon on CAN Bus.
	 *
	 * @param deviceNumber
	 *            CAN Device Id of Pigeon [0,62]
	 */
	PigeonIMU(int deviceNumber);
	/**
	 * Create a Pigeon object that communciates with Pigeon through the
	 * Gadgeteer ribbon cable connected to a Talon on CAN Bus.
	 *
	 * @param talonSrx
	 *            Object for the TalonSRX connected via ribbon cable.
	 */
	PigeonIMU(ctre::phoenix::motorcontrol::can::TalonSRX * talonSrx);

    ~PigeonIMU();

    static void DestroyAllPigeonIMUs();

	/**
	 * Sets the Yaw register to the specified value.
	 *
	 * @param angleDeg  Degree of Yaw [+/- 23040 degrees]
	 * @param timeoutMs
   *            Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int SetYaw(double angleDeg, int timeoutMs = 0);
	/**
	 * Atomically add to the Yaw register.
	 *
	 * @param angleDeg  Degrees to add to the Yaw register.
	 * @param timeoutMs
   *            Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int AddYaw(double angleDeg, int timeoutMs = 0);
	/**
	 * Sets the Yaw register to match the current compass value.
	 *
	 * @param timeoutMs
   *            Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int SetYawToCompass(int timeoutMs = 0);

	/**
	 * Sets the Fused Heading to the specified value.
	 *
	 * @param angleDeg  Degree of heading [+/- 23040 degrees]
	 * @param timeoutMs
   *            Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int SetFusedHeading(double angleDeg, int timeoutMs = 0);
	/**
	 * Atomically add to the Fused Heading register.
	 *
	 * @param angleDeg  Degrees to add to the Fused Heading register.
	 * @param timeoutMs
   *            Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int AddFusedHeading(double angleDeg, int timeoutMs = 0);
	/**
	 * Sets the Fused Heading register to match the current compass value.
	 *
	 * @param timeoutMs
   *            Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int SetFusedHeadingToCompass(int timeoutMs = 0);
	/**
	 * Sets the AccumZAngle.
	 *
	 * @param angleDeg  Degrees to set AccumZAngle to.
	 * @param timeoutMs
   *            Timeout value in ms. If nonzero, function will wait for
   *            config success and report an error if it times out.
   *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int SetAccumZAngle(double angleDeg, int timeoutMs = 0);

	/**
     * @deprecated use setTemperatureCompensationDisable instead 
     * This was done to better match with the lower level API.
     * NOTE: this isn't a persistant config, every boot temperature 
     * compensation will be enabled
     * This was also done so the default value for the paramter is false instead of true.
	 * Enable/Disable Temp compensation. Pigeon defaults with this on at boot.
	 *
 	 * @param bTempCompEnable Set to "True" to enable temperature compensation.
	 * @param timeoutMs
     *          Timeout value in ms. If nonzero, function will wait for
     *          config success and report an error if it times out.
     *          If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
    int ConfigTemperatureCompensationEnable(bool bTempCompEnable,
			int timeoutMs = 0);

    /**
     * Disable/Enable Temp compensation. Pigeon has this on/False at boot.
     *
     * @param bTempCompDisable Set to "False" to enable temperature compensation.
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
	int SetTemperatureCompensationDisable(bool bTempCompDisable,
			int timeoutMs = 0);
	/**
	 * Set the declination for compass. Declination is the difference between
	 * Earth Magnetic north, and the geographic "True North".
	 *
	 * @param angleDegOffset  Degrees to set Compass Declination to.
	 * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int SetCompassDeclination(double angleDegOffset, int timeoutMs = 0);
	/**
	 * Sets the compass angle. Although compass is absolute [0,360) degrees, the
	 * continuous compass register holds the wrap-arounds.
	 *
	 * @param angleDeg  Degrees to set continuous compass angle to.
	 * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int SetCompassAngle(double angleDeg, int timeoutMs = 0);

	/**
	 * Enters the Calbration mode.  See the Pigeon IMU documentation for More
	 * information on Calibration.
	 *
	 * @param calMode  Calibration to execute
	 * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int EnterCalibrationMode(CalibrationMode calMode, int timeoutMs = 0);
	/**
	 * Get the status of the current (or previousley complete) calibration.
	 *
	 * @param [out] statusToFill Container for the status information.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	int GetGeneralStatus(PigeonIMU::GeneralStatus & statusToFill);
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
	 * Get 6d Quaternion data.
	 *
	 * @param wxyz Array to fill with quaternion data w[0], x[1], y[2], z[3]
	 * @return The last ErrorCode generated.
	 */
	int Get6dQuaternion(double wxyz[4]);
	/**
	 * Get Yaw, Pitch, and Roll data.
	 *
	 * @param ypr Array to fill with yaw[0], pitch[1], and roll[2] data
	 * @return The last ErrorCode generated.
	 */
	int GetYawPitchRoll(double ypr[3]);
	/**
	 * Get AccumGyro data.
	 * AccumGyro is the integrated gyro value on each axis.
	 *
	 * @param xyz_deg Array to fill with x[0], y[1], and z[2] AccumGyro data
	 * @return The last ErrorCode generated.
	 */
	int GetAccumGyro(double xyz_deg[3]);
	/**
	 * Get the absolute compass heading.
	 * @return compass heading [0,360) degrees.
	 */
	double GetAbsoluteCompassHeading();
	/**
	 * Get the continuous compass heading.
	 * @return continuous compass heading [-23040, 23040) degrees. Use
	 *         SetCompassHeading to modify the wrap-around portion.
	 */
	double GetCompassHeading();
	/**
	 * Gets the compass' measured magnetic field strength.
	 * @return field strength in Microteslas (uT).
	 */
	double GetCompassFieldStrength();
	/**
	 * Gets the temperature of the pigeon.
	 *
	 * @return Temperature in ('C)
	 */
	double GetTemp();
	/**
	 * Gets the current Pigeon state
	 *
	 * @return PigeonState enum
	 */
	PigeonState GetState();
	/**
	 * Gets the current Pigeon uptime.
	 *
	 * @return How long has Pigeon been running in whole seconds. Value caps at
	 *         255.
	 */
	uint32_t GetUpTime();
	/**
	 * Get Raw Magnetometer data.
	 *
	 * @param rm_xyz Array to fill with x[0], y[1], and z[2] data
	 * 				Number is equal to 0.6 microTeslas per unit.
	 * @return The last ErrorCode generated.
	 */
	int GetRawMagnetometer(int16_t rm_xyz[3]);

	/**
	 * Get Biased Magnetometer data.
	 *
	 * @param bm_xyz Array to fill with x[0], y[1], and z[2] data
	 * 				Number is equal to 0.6 microTeslas per unit.
	 * @return The last ErrorCode generated.
	 */
	int GetBiasedMagnetometer(int16_t bm_xyz[3]);
	/**
	 * Get Biased Accelerometer data.
	 *
	 * @param ba_xyz Array to fill with x[0], y[1], and z[2] data.
	 * 			These are in fixed point notation Q2.14.  eg. 16384 = 1G
	 * @return The last ErrorCode generated.
	 */
	int GetBiasedAccelerometer(int16_t ba_xyz[3]);
	/**
	 * Get Raw Gyro data.
	 *
	 * @param xyz_dps Array to fill with x[0], y[1], and z[2] data in degrees per second.
	 * @return The last ErrorCode generated.
	 */
	int GetRawGyro(double xyz_dps[3]);
	/**
	 * Get Accelerometer tilt angles.
	 *
	 * @param tiltAngles Array to fill with x[0], y[1], and z[2] angles in degrees.
	 * @return The last ErrorCode generated.
	 */
	int GetAccelerometerAngles(double tiltAngles[3]);
	/**
	 * Get the current Fusion Status (including fused heading)
	 *
	 * @param status 	object reference to fill with fusion status flags.
	 *					Caller may pass null if flags are not needed.
	 * @return The fused heading in degrees.
	 */
	double GetFusedHeading(FusionStatus & status);
	/**
	 * Gets the Fused Heading
	 *
	 * @return The fused heading in degrees.
	 */
	double GetFusedHeading();
	uint32_t GetResetCount();
	uint32_t GetResetFlags();
	uint32_t GetFirmVers();

	/**
	 * @return true iff a reset has occurred since last call.
	 */
	bool HasResetOccurred();

	static std::string ToString(PigeonIMU::PigeonState state);
	static std::string ToString(CalibrationMode cm);

	/**
	 * Sets the value of a custom parameter. This is for arbitrary use.
	 *
	 * Sometimes it is necessary to save calibration/declination/offset
	 * information in the device. Particularly if the
	 * device is part of a subsystem that can be replaced.
	 *
	 * @param newValue
	 *            Value for custom parameter.
	 * @param paramIndex
	 *            Index of custom parameter. [0-1]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs = 0);
	/**
	 * Gets the value of a custom parameter. This is for arbitrary use.
   *
   * Sometimes it is necessary to save calibration/declination/offset
   * information in the device. Particularly if the
   * device is part of a subsystem that can be replaced.
	 *
	 * @param paramIndex
	 *            Index of custom parameter. [0-1]
	 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
	 * @return Value of the custom param.
	 */
	int ConfigGetCustomParam(int paramIndex, int timeoutMs = 0);
	/**
	 * Sets a parameter. Generally this is not used.
   * This can be utilized in
   * - Using new features without updating API installation.
   * - Errata workarounds to circumvent API implementation.
   * - Allows for rapid testing / unit testing of firmware.
	 *
	 * @param param
	 *            Parameter enumeration.
	 * @param value
	 *            Value of parameter.
	 * @param subValue
	 *            Subvalue for parameter. Maximum value of 255.
	 * @param ordinal
	 *            Ordinal of parameter.
	 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
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
	 * @param param
	 *            Parameter enumeration.
	 * @param ordinal
	 *            Ordinal of parameter.
	 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
	 * @return Value of parameter.
	 */
	double ConfigGetParameter(ctre::phoenix::ParamEnum param, int ordinal, int timeoutMs = 0);
    ErrorCode ConfigGetParameter(ParamEnum param, int32_t valueToSend,
            int32_t & valueReceived, uint8_t & subValue, int32_t ordinal,
            int32_t timeoutMs);

	/**
	 * Sets the period of the given status frame.
	 *
	 * @param statusFrame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode SetStatusFramePeriod(PigeonIMU_StatusFrame statusFrame, uint8_t periodMs,
			int timeoutMs = 0);

	/**
	 * Gets the period of the given status frame.
	 *
	 * @param frame
	 *            Frame to get the period of.
	 * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
	 * @return Period of the given status frame.
	 */
	int GetStatusFramePeriod(PigeonIMU_StatusFrame frame,
			int timeoutMs = 0) ;
	/**
	 * Sets the period of the given control frame.
	 *
	 * @param frame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode SetControlFramePeriod(PigeonIMU_ControlFrame frame,
			int periodMs);
	/**
	 * Gets the firmware version of the device.
	 *
	 * @return param holds the firmware version of the device. Device must be powered
	 * cycled at least once.
	 */
	int GetFirmwareVersion() ;
	/**
	 * Gets the fault status
	 *
	 * @param toFill
	 *            Container for fault statuses.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode GetFaults(PigeonIMU_Faults & toFill) ;
	/**
	 * Gets the sticky fault status
	 *
	 * @param toFill
	 *            Container for sticky fault statuses.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode GetStickyFaults(PigeonIMU_StickyFaults & toFill);
	/**
	 * Clears the Sticky Faults
	 *
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	ErrorCode ClearStickyFaults(int timeoutMs = 0);
	
	void* GetLowLevelHandle() {
		return _handle;
	}
	
	//------ All Configs ----------//
    /**
     * Configures all peristant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     *
     * @return Error Code generated by function. 0 indicates no error. 
     */
    virtual ctre::phoenix::ErrorCode ConfigAllSettings(const PigeonIMUConfiguration &allConfigs, int timeoutMs = 50);
    /**
     * Gets all persistant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
    virtual void GetAllConfigs(PigeonIMUConfiguration &allConfigs, int timeoutMs = 50);
    /**
     * Configures all peristant settings to defaults.
     *
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     *
     * @return Error Code generated by function. 0 indicates no error. 
     */
    virtual ErrorCode ConfigFactoryDefault(int timeoutMs = 50);
private:
	/** firmware state reported over CAN */
	enum MotionDriverState {
		Init0 = 0,
		WaitForPowerOff = 1,
		ConfigAg = 2,
		SelfTestAg = 3,
		StartDMP = 4,
		ConfigCompass_0 = 5,
		ConfigCompass_1 = 6,
		ConfigCompass_2 = 7,
		ConfigCompass_3 = 8,
		ConfigCompass_4 = 9,
		ConfigCompass_5 = 10,
		SelfTestCompass = 11,
		WaitForGyroStable = 12,
		AdditionalAccelAdjust = 13,
		Idle = 14,
		Calibration = 15,
		LedInstrum = 16,
		Error = 31,
	};
	/** sub command for the various Set param enums */
	enum TareType {
		SetValue = 0x00, AddOffset = 0x01, MatchCompass = 0x02, SetOffset = 0xFF,
	};
	/** data storage for reset signals */
	struct ResetStats {
		int32_t resetCount;
		int32_t resetFlags;
		int32_t firmVers;
		bool hasReset;
	};
	ResetStats _resetStats = { 0, 0, 0, false };

	/** Portion of the arbID for all status and control frames. */
	void* _handle;
	uint32_t _deviceNumber;
	uint32_t _usageHist = 0;
	uint64_t _cache;
	uint32_t _len;

	/** overall threshold for when frame data is too old */
	const uint32_t EXPECTED_RESPONSE_TIMEOUT_MS = (200);

	int PrivateSetParameter(ParamEnum paramEnum, TareType tareType,
			double angleDeg, int timeoutMs = 0);

	PigeonIMU::PigeonState GetState(int errCode, const uint64_t & statusFrame);
	double GetTemp(const uint64_t & statusFrame);
	



};// class PigeonIMU
} // namespace signals
} // namespace phoenix
} // namespace ctre
