#ifndef MOTIONPROCESSING_ONLINERESTCALIBRATOR_H
#define MOTIONPROCESSING_ONLINERESTCALIBRATOR_H

#include "types.h"
#include "RestDetection.h"
#include "logging/Logger.h"

namespace SlimeVR::MotionProcessing
{

class OnlineRestCalibrator
{
public:
	OnlineRestCalibrator(sensor_real_t accelTs, sensor_real_t gyroTs);

    void updateAcc(const sensor_real_t rawSample[3], const sensor_real_t calibratedSample[3]);
    void updateGryo(const sensor_real_t rawSample[3], const sensor_real_t calibratedSample[3]);

	void reset();

private:
	bool tryReportStats();

	sensor_real_t m_AccelTs;
	sensor_real_t m_GyroTs;
    RestDetection m_RestDetection;

    int m_NumAccelSamples = 0;
    sensor_real_t m_SumRawAccelSample[3] = {};
    sensor_real_t m_SumCalibratedAccelSample[3] = {};

    int m_NumGyroSamples = 0;
    sensor_real_t m_SumRawGryoSample[3] = {};
    sensor_real_t m_SumCalibratedGryoSample[3] = {};

	bool m_Paused = false;

    Logging::Logger m_Logger{"OnlineRestCalibrator"};
};

} // namespace

#endif
