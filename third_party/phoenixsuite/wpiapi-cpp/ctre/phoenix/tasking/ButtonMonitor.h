#pragma once

#include "ctre/phoenix/tasking/ILoopable.h"
#include "ctre/phoenix/tasking/IProcessable.h"
#include <functional>

/* forward proto's */
namespace frc {
	class GenericHID;
}

namespace ctre {
namespace phoenix {
namespace tasking {

/**
 * Class to handle button events
 */
class ButtonMonitor: public IProcessable, public ILoopable {
public:

	/**
	 * Interface for classes that handle button events
	 */
	class IButtonPressEventHandler {
		public:
			virtual ~IButtonPressEventHandler(){}
			/**
			 * Method to execute when a button is pressed
			 * @param idx Index of button pressed
			 * @param isDown Whether the button is down or not
			 */
			virtual void OnButtonPress(int idx, bool isDown) = 0;
	};

	/**
	 * Constructor for ButtonMonitor
	 * @param controller Controller to monitor
	 * @param buttonIndex Button to monitor
	 * @param ButtonPressEventHandler Class that will handle buttonPresses
	 */
	ButtonMonitor(frc::GenericHID * controller, int buttonIndex, IButtonPressEventHandler * ButtonPressEventHandler);
	/**
	 * This is able to copy another ButtonMonitor class
	 */
	ButtonMonitor(const ButtonMonitor & rhs);
	virtual ~ButtonMonitor() {	}

	/* IProcessable */
	/**
	 * Call this every loop, it monitors for button presses
	 */
	virtual void Process();

	/* ILoopable */
	/**
	 * Do nothing on start
	 */
	virtual void OnStart();
	/**
	 * Process every loop
	 */
	virtual void OnLoop();
	/**
	 * @return false, this is never done
	 */
	virtual bool IsDone();
	/**
	 * Do nothing on stop
	 */
	virtual void OnStop();

private:
	frc::GenericHID * _gameCntrlr;
	int _btnIdx;
	IButtonPressEventHandler * _handler;
	bool _isDown = false;
};
}
}
}