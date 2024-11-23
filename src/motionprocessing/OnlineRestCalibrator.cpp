#include "OnlineRestCalibrator.h"

namespace SlimeVR::MotionProcessing
{

constexpr sensor_real_t SAMPLE_DURATION_IN_SEC = 10.0f;

OnlineRestCalibrator::OnlineRestCalibrator(sensor_real_t accelTs, sensor_real_t gyroTs)
	: m_AccelTs(accelTs),
	  m_GyroTs(gyroTs),
	  m_RestDetection(RestDetectionParams{}, gyroTs, accelTs)
{
}

void OnlineRestCalibrator::updateAcc(const sensor_real_t rawSample[3], const sensor_real_t calibratedSample[3])
{
	// REVIEW: Why does this need accelTs when it's already provided in the constructor?
	m_RestDetection.updateAcc(m_AccelTs, rawSample);

	if (m_RestDetection.getRestDetected())
	{
		if (!m_Paused)
		{
			if (m_NumAccelSamples == 0)
			{
				// m_Logger.debug("Starting online rest calibration...");
			}

			++m_NumAccelSamples;
			m_SumRawAccelSample[0] += rawSample[0];
			m_SumRawAccelSample[1] += rawSample[1];
			m_SumRawAccelSample[2] += rawSample[2];
			m_SumCalibratedAccelSample[0] += calibratedSample[0];
			m_SumCalibratedAccelSample[1] += calibratedSample[1];
			m_SumCalibratedAccelSample[2] += calibratedSample[2];

			if (tryReportStats())
			{
				reset();
				// m_Paused = true;
			}
		}
	}
	else
	{
		reset();
	}
}

void OnlineRestCalibrator::updateGryo(const sensor_real_t rawSample[3], const sensor_real_t calibratedSample[3])
{
	m_RestDetection.updateGyr(rawSample);

	if (m_RestDetection.getRestDetected())
	{
		if (!m_Paused)
		{
			// if (m_NumGyroSamples % 20 == 0) {
			// 	m_Logger.info("Gryo %f %f %f", calibratedSample[0], calibratedSample[1], calibratedSample[2]);
			// }

			++m_NumGyroSamples;
			m_SumRawGryoSample[0] += rawSample[0];
			m_SumRawGryoSample[1] += rawSample[1];
			m_SumRawGryoSample[2] += rawSample[2];
			m_SumCalibratedGryoSample[0] += calibratedSample[0];
			m_SumCalibratedGryoSample[1] += calibratedSample[1];
			m_SumCalibratedGryoSample[2] += calibratedSample[2];

			if (tryReportStats())
			{
				reset();
				// m_Paused = true;
			}
		}
	}
	else
	{
		reset();
	}
}

void OnlineRestCalibrator::reset()
{
	m_NumAccelSamples = 0;
	m_SumRawAccelSample[0] = 0.0f;
	m_SumRawAccelSample[1] = 0.0f;
	m_SumRawAccelSample[2] = 0.0f;
	m_SumCalibratedAccelSample[0] = 0.0f;
	m_SumCalibratedAccelSample[1] = 0.0f;
	m_SumCalibratedAccelSample[2] = 0.0f;

	m_NumGyroSamples = 0;
	m_SumRawGryoSample[0] = 0.0f;
	m_SumRawGryoSample[1] = 0.0f;
	m_SumRawGryoSample[2] = 0.0f;
	m_SumCalibratedGryoSample[0] = 0.0f;
	m_SumCalibratedGryoSample[1] = 0.0f;
	m_SumCalibratedGryoSample[2] = 0.0f;

	m_Paused = false;
}

bool OnlineRestCalibrator::tryReportStats()
{
	if ((m_NumAccelSamples < SAMPLE_DURATION_IN_SEC / m_AccelTs) ||
		(m_NumGyroSamples < SAMPLE_DURATION_IN_SEC / m_GyroTs))
	{
		return false;
	}

	float arx = m_SumRawAccelSample[0] / m_NumAccelSamples;
	float ary = m_SumRawAccelSample[1] / m_NumAccelSamples;
	float arz = m_SumRawAccelSample[2] / m_NumAccelSamples;
	float arn = std::sqrt(arx * arx + ary * ary + arz * arz);

	float acx = m_SumCalibratedAccelSample[0] / m_NumAccelSamples;
	float acy = m_SumCalibratedAccelSample[1] / m_NumAccelSamples;
	float acz = m_SumCalibratedAccelSample[2] / m_NumAccelSamples;
	float acn = std::sqrt(acx * acx + acy * acy + acz * acz);

	float grx = m_SumRawGryoSample[0] / m_NumGyroSamples;
	float gry = m_SumRawGryoSample[1] / m_NumGyroSamples;
	float grz = m_SumRawGryoSample[2] / m_NumGyroSamples;

	float gcx = m_SumCalibratedGryoSample[0] / m_NumGyroSamples;
	float gcy = m_SumCalibratedGryoSample[1] / m_NumGyroSamples;
	float gcz = m_SumCalibratedGryoSample[2] / m_NumGyroSamples;

	m_Logger.info(
		"Rest calibrated accel [%f, %f, %f, %f] gryo [%f, %f, %f], "
		"raw accel [%f, %f, %f, %f] gryo [%f, %f, %f]",
		acx, acy, acz, acn,
		gcx, gcy, gcz,
		arx, ary, arz, arn,
		grx, gry, grz
	);

	return true;
}

} // namespace
