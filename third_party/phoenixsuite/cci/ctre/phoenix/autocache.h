#pragma once

namespace ctre{
namespace phoenix{
    /**
     * Keeps track of cache state
     */
    enum AutocacheState{
        /**
         * Disabled cache
         */
        DISABLED = 0,
        /**
         * Enabled cache
         */
        ENABLED = 1,
    };
}
}