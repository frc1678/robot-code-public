#include "ctre/phoenix/cci/Unmanaged_CCI.h"

namespace ctre {
namespace phoenix {
namespace unmanaged {

class Unmanaged {
public:
    static void FeedEnable(int timeoutMs);
    static bool GetEnableState();
    static int GetPhoenixVersion();
};

}
}
}
