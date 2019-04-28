#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/cci/Platform_CCI.h"

namespace ctre {
namespace phoenix {
namespace platform {

/**
 * Class for simulation as a platform
 */
class PlatformSim {

public:
    /**
     * Create Device
     * @param type Device Type
     * @param id Device ID
     * @return ErrorCode
     */
    static int32_t SimCreate(DeviceType type, int id);
    /**
     * Destroy Device
     * @param type Device Type
     * @param id Device ID
     * @return ErrorCode
     */
    static int32_t SimDestroy(DeviceType type, int id);
    /**
     * Destroy all devices in sim
     * @return ErrorCode
     */
    static int32_t SimDestroyAll();
    /**
     * Get configs of simulated device
     * @param type Device Type
     * @param param Param to get 
     * @param valueToSend subValue
     * @param outValueReceived requested value
     * @param outSubvalue requested subValue
     * @param ordinal Ordinal
     * @param id Device ID
     * @return ErrorCode
     */
    static int32_t SimConfigGet(DeviceType type, uint32_t param, uint32_t valueToSend, uint32_t & outValueReceived, uint32_t & outSubvalue, uint32_t ordinal, int32_t id);
    /**
     * Sets configs of simulated device
     * @param type Device Type
     * @param param Param to send
     * @param value Value to send
     * @param subValue Subvalue to send
     * @param ordinal Ordinal
     * @param id Device ID
     * @return ErrorCode
     */
    static int32_t SimConfigSet(DeviceType type, uint32_t param, uint32_t value, uint32_t subValue, uint32_t ordinal, int32_t id);

};

}
}
}
