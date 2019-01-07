#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/cci/Platform_CCI.h"

namespace ctre {
namespace phoenix {
namespace platform {

class PlatformSim {

public:
    static int32_t SimCreate(DeviceType type, int id);
    static int32_t SimDestroy(DeviceType type, int id);
    static int32_t SimDestroyAll();
    static int32_t SimConfigGet(DeviceType type, uint32_t param, uint32_t valueToSend, uint32_t & outValueReceived, uint32_t & outSubvalue, uint32_t ordinal, int32_t id);
    static int32_t SimConfigSet(DeviceType type, uint32_t param, uint32_t value, uint32_t subValue, uint32_t ordinal, int32_t id);

};

}
}
}
