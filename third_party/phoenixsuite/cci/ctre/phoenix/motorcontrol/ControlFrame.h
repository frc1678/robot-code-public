#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Control Frames for motor controllers
 */
enum ControlFrame {
	/**
	 * Control
	 */
	Control_3_General = 0x040080,
	/**
	 * Advanced Control
	 */
	Control_4_Advanced = 0x0400C0,
	/**
	 * Trajectory points 
	 */
	Control_6_MotProfAddTrajPoint = 0x040140,
};

/**
 * Control Frames for enhanced motor controllers
 */
enum ControlFrameEnhanced {
	/**
	 * Control
	 */
	Control_3_General_ = 0x040080,
	/**
	 * Advanced Control
	 */
	Control_4_Advanced_ = 0x0400c0,
	/**
	 * Override feedback output
	 */
	Control_5_FeedbackOutputOverride_ = 0x040100,
	/**
	 * Trajectory points
	 */
	Control_6_MotProfAddTrajPoint_ = 0x040140,
};

/**
 * Class to handle promotion of controlFrame to controlFrameEnhanced
 */
class ControlFrameRoutines {
	/**
	 * Promotes a ControlFrame to a ControlFrameEnhanced
	 * @param controlFrame frame to promote
	 * @return promoted ControlFrame
	 */
	static ControlFrameEnhanced Promote(ControlFrame controlFrame) {
		return (ControlFrameEnhanced) controlFrame;
	}
};

}
}
}

