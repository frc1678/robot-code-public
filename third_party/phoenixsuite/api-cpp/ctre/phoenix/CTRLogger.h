#include "ctre/phoenix/ErrorCode.h" // ErrorCode
#include <string>

namespace ctre {
namespace phoenix {

/**
 * Object to handle error logging
 */
class CTRLogger {
public:
	/**
	 * Close the logger
	 */
	static void Close();
	/**
	 * Logs an entry into the Phoenix DS Error/Logger stream
	 * @param code Error code to log.  If OKAY is passed, no action is taken.
	 * @param origin Origin string to send to DS/Log
	 * @return OKAY error code.
	 */
	static ErrorCode Log(ErrorCode code, const char * dev, const char * func);
	/**
	 * Open the logger
	 * @param language the language you're using
	 */
	static void Open(int language);
	//static void Description(ErrorCode code, const char *&shrt, const char *&lng);
};

}
}
