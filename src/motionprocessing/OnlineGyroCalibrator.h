#ifndef MOTIONPROCESSING_OnlineGyroCalibrator_H
#define MOTIONPROCESSING_OnlineGyroCalibrator_H

#include "RestDetection.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "types.h"

namespace SlimeVR::MotionProcessing {

class OnlineGyroCalibrator {
public:
	OnlineGyroCalibrator(sensor_real_t accelTs, sensor_real_t gyroTs);

	void updateAcc(const sensor_real_t rawSample[3]);

	void updateGryo(
		const int16_t rawData[3],
		const sensor_real_t rawSample[3],
		SlimeVR::Configuration::SoftFusionSensorConfig& sensorConfig
	);

	void reset();

private:
	void tryUpdateCalibration(
		SlimeVR::Configuration::SoftFusionSensorConfig& sensorConfig
	);

	sensor_real_t m_AccelTs;
	sensor_real_t m_GyroTs;
	RestDetection m_RestDetection;

	int m_NumAccelSamples = 0;

	int m_NumGyroSamples = 0;
	int m_SumRawGryoData[3] = {};
	sensor_real_t m_SumRawGryoSample[3] = {};

	Logging::Logger m_Logger{"OnlineGyroCalibrator"};
};

}  // namespace SlimeVR::MotionProcessing

#endif
