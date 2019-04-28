#include "ctre/phoenix/cci/Unmanaged_CCI.h"

namespace ctre {
namespace phoenix {
/** unmanaged namespace */
namespace unmanaged {

/**
 * Handles enabling when used in a non-FRC manner
 */
class Unmanaged {
public:
    /**
     * Feed the enable frame
     * @param timeoutMs Timeout before disabling
     */
    static void FeedEnable(int timeoutMs);
    /**
     * @return true if enabled
     */
    static bool GetEnableState();
    /**
     * @return Phoenix version
     */
    static int GetPhoenixVersion();
    /**
     * Calling this function will load and start
     * the Phoenix background tasks.
     * 
     * This can be useful if you need the
     * Enable/Disable functionality for CAN devices
     * but aren't using any of the CAN device classes.
     * 
     * This function does NOT need to be called if
     * you are using any of the Phoenix CAN device classes.
     */
    static void LoadPhoenix();
};

}
}
}
