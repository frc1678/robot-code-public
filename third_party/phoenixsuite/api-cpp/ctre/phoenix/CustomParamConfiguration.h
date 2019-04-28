#pragma once

#include <string>

namespace ctre {
namespace phoenix {
/**
 * Configurables for any custom param configs
 */
struct CustomParamConfiguration {
	/**
	 * Custom Param 0
	 */
	int customParam0;	
	/**
	 * Custom Param 1
	 */
	int customParam1;	
	/**
	 * Enable optimizations for ConfigAll (defaults true)
	 */
	bool enableOptimizations;
	CustomParamConfiguration() :
		customParam0(0),
		customParam1(0),
		enableOptimizations(true)
	{
	}

    /**
     * @return string representation of currently selected configs
     */
	std::string toString() {
		return toString("");
	}

    /**
     * @param prependString String to prepend to all the configs
     * @return string representation fo currently selected configs
     */
    std::string toString(std::string prependString) {
        std::string retstr = prependString + ".customParam0 = " + std::to_string(customParam0) + ";\n";
        retstr += prependString + ".customParam1 = " + std::to_string(customParam1) + ";\n";
        
        return retstr;
    }
};

/**
 * Util class to help custom configs
 */
struct CustomParamConfigUtil {
	private:
		static CustomParamConfiguration _default;
	public:
		/**
		 * @param settings Settings to compare against
		 * @return Whether CustomParam0 is different
		 */
		static bool CustomParam0Different (const CustomParamConfiguration & settings) { return (!(settings.customParam0 == _default.customParam0)) || !settings.enableOptimizations; }
		/**
		 * @param settings Settings to compare against
		 * @return Whether CustomParam1 is different
		 */
		static bool CustomParam1Different (const CustomParamConfiguration & settings) { return (!(settings.customParam1 == _default.customParam1)) || !settings.enableOptimizations; }
};
}
}