#ifndef GadgeteerUartClient_h_
#define GadgeteerUartClient_h_
#include <stdint.h>
#include <string>

/**
 * Interface for uart gadgeteer devices
 */
class IGadgeteerUartClient
{
public:
	IGadgeteerUartClient() {}
    virtual ~IGadgeteerUartClient() {}

	/**
	 * Device connected to gadgeteer
	 */
	enum GadgeteerProxyType
	{
		/**
		 * General Gadgeteer Proxy
		 */
		General = 0,
		/**
		 * Pigeon connected to gadgeteer
		 */
		Pigeon = 1,
		/**
		 * HERO connected to gadgeteer
		 */
		PC_HERO = 2,
	};
	/**
	 * Method of connection to gadgeteer
	 */
	enum GadgeteerConnection
	{
		/**
		 * Device not connected
		 */
		NotConnected = 0,
		/**
		 * Device in process of connecting
		 */
		Connecting = 1,
		/**
		 * Device is connected
		 */
		Connected = 2,
	};
	/**
	 * The status of the gadgeteer device
	 */
	struct GadgeteerUartStatus {
		/** Type of gadgeteer */
		GadgeteerProxyType type;
		/** Connection status */
		GadgeteerConnection conn;
		/** Bitrate of connection */
		uint32_t bitrate;
		/** Number of resets that have happened */
		uint32_t resetCount;
	};

	/**
	 * Gets gadgeteer status
	 * @param status status object to fill
	 * @return ErrorCode
	 */
    virtual int GetGadgeteerStatus(GadgeteerUartStatus & status) = 0;

	/**
	 * Gets the string representation of GadgeteerProxyType
	 * @param gpt GadgeteerProxyType to get the string of
	 * @return strnig representation of GadgeteerProxyType
	 */
    static std::string ToString(IGadgeteerUartClient::GadgeteerProxyType gpt) {
    	std::string retval;
		switch(gpt) {
			case General: retval = "General"; break;
			case Pigeon : retval = "Pigeon"; break;
			case PC_HERO: retval = "PC_HERO"; break;
		}
		return retval;
    }
	/**
	 * Gets the string representation of GadgeteerConnection
	 * @param gc GadgeteerConnection to get the string of
	 * @return strnig representation of GadgeteerConnection
	 */
    static std::string ToString(IGadgeteerUartClient::GadgeteerConnection gc) {
    	std::string retval;
		switch(gc) {
			case NotConnected: retval = "NotConnected"; break;
			case Connecting : retval = "Connecting"; break;
			case Connected: retval = "Connected"; break;
		}
		return retval;
    }

protected:
};
#endif
