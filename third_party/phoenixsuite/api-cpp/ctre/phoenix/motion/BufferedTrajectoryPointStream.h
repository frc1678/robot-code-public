#pragma once
#include <stdint.h>
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/cci/BuffTrajPointStream_CCI.h"
#include "ctre/phoenix/motion/TrajectoryPoint.h"
namespace ctre {
	namespace phoenix {
		namespace motion {
			/**
			 * Stream of trajectory points for Talon/Victor motion profiling.
			 */
			class BufferedTrajectoryPointStream {
			public:
				BufferedTrajectoryPointStream();
				~BufferedTrajectoryPointStream();
				BufferedTrajectoryPointStream(const BufferedTrajectoryPointStream &) = delete;
				BufferedTrajectoryPointStream& operator=(const BufferedTrajectoryPointStream &) = delete;
				/**
				 * @brief Clear all trajectory points.
			 	 * @return nonzero error code if operation fails.
			 	 */
				ctre::phoenix::ErrorCode Clear();
				/**
				* @brief Write a single trajectory point into the buffer.
				* @return nonzero error code if operation fails.
				*/
				ctre::phoenix::ErrorCode Write(const TrajectoryPoint & trajPt);
				/**
				* @brief Writes an array of trajectory point into the buffer.
				* @return nonzero error code if operation fails.
				*/
				ctre::phoenix::ErrorCode Write(const TrajectoryPoint * trajPts, uint32_t trajPtCount);
				/**
				 * @return raw handle for resource management.
				 */
				void * GetHandle() { return _handle; }
			private:
				void * _handle;
			};
		} // namespace motion
	} // namespace phoenix
} // namespace ctre
