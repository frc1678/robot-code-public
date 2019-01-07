#pragma once
#include <stdint.h>
namespace ctre {
	namespace phoenix {
		/**
		 * Simple address holder.
		 */
		class CANBusAddressable {

		public:
			CANBusAddressable(int deviceNumber) {
				_deviceNum = deviceNumber;
			}

			int GetDeviceNumber() {
				return _deviceNum;
			}
		protected:
		private:
			int _deviceNum;
		};
	}
}
