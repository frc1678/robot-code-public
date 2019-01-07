#pragma once

#include <string>

namespace ctre {
namespace phoenix {
struct CustomParamConfiguration {
	int customParam0;	
	int customParam1;	
	bool enableOptimizations;
	CustomParamConfiguration() :
		customParam0(0),
		customParam1(0),
		enableOptimizations(true)
	{
	}

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {
        std::string retstr = prependString + ".customParam0 = " + std::to_string(customParam0) + ";\n";
        retstr += prependString + ".customParam1 = " + std::to_string(customParam1) + ";\n";
        
        return retstr;
    }
};

struct CustomParamConfigUtil {
	private:
		static CustomParamConfiguration _default;
	public:
		static bool CustomParam0Different (const CustomParamConfiguration & settings) { return (!(settings.customParam0 == _default.customParam0)) || !settings.enableOptimizations; }
		static bool CustomParam1Different (const CustomParamConfiguration & settings) { return (!(settings.customParam1 == _default.customParam1)) || !settings.enableOptimizations; }
};
}
}