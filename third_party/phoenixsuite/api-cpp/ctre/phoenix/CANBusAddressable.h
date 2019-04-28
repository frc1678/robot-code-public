#pragma once
#include <stdint.h>
namespace ctre {
	namespace phoenix {
		/**
		 * Simple address holder.
		 */
		class CANBusAddressable {

		public:
			/**
			 * Constructor for a CANBusAddressable device
			 * @param deviceNumber CAN Device ID of device
			 */
			CANBusAddressable(int deviceNumber) {
				_deviceNum = deviceNumber;
			}

			/**
			 * @return CAN device number of device
			 */
			int GetDeviceNumber() {
				return _deviceNum;
			}
		protected:
		private:
			int _deviceNum;
		};
	}
}
