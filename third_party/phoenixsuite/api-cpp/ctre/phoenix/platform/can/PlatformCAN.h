#include "ctre/phoenix/cci/PlatformCAN_CCI.h"

namespace ctre {
namespace phoenix {
namespace platform {
namespace can {

class PlatformCAN {
public:
    static int32_t SetCANInterface(const char * canInterface);

    static int32_t StartAll();

    static int32_t DestroyAll();

    static void SetAutocacheLevel(ctre::phoenix::AutocacheState state);
};

}
}
}
}
