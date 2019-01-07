#pragma once

namespace ctre {
	namespace phoenix {
		namespace unmanaged {
			//These functions do nothing on a roborio for FRC use
			void FeedEnable(int timeoutMs);
			bool GetEnableState();
			void SetTransmitEnable(bool en);
			bool GetTransmitEnable();
			int GetPhoenixVersion();
		}
	}
}
