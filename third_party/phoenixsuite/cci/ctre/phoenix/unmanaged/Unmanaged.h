#pragma once

#include <cstdint>

namespace ctre {
	namespace phoenix {
		namespace unmanaged {
			/**
			 * Feed the robot enable.
			 * This function does nothing on a roborio during FRC use.
			 * @param timeoutMs Timeout before disabling
			 */
			void FeedEnable(int timeoutMs);
			/**
			 * @return true if enabled
			 */
			bool GetEnableState();
			/**
			 * Sets whether to enable transmitting
			 * This function does nothing on a roborio during FRC use.
			 * @param en True enables transmitting
			 */
			void SetTransmitEnable(bool en);
			/**
			 * @return true if transmitting is enabled
			 */
			bool GetTransmitEnable();
			/**
			 * @return Phoenix version
			 */
			int GetPhoenixVersion();
			void LoadPhoenix();

			int IoControl(uint32_t ioControlCode, uint64_t ioControlParam, char* inValue = 0, int inValueSize = 0, char* outValue = 0, int outValueSize = 0);
		}
	}
}
