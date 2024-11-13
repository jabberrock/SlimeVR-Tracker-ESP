#include "OnlineGyroCalibrator.h"

namespace SlimeVR::MotionProcessing {

constexpr sensor_real_t SAMPLE_DURATION_IN_SEC = 10.0f;

OnlineGyroCalibrator::OnlineGyroCalibrator(sensor_real_t accelTs, sensor_real_t gyroTs)
	: m_AccelTs(accelTs)
	, m_GyroTs(gyroTs)
	, m_RestDetection(RestDetectionParams{}, gyroTs, accelTs) {}

void OnlineGyroCalibrator::updateAcc(const sensor_real_t rawSample[3]) {
	m_RestDetection.updateAcc(m_AccelTs, rawSample);
	if (!m_RestDetection.getRestDetected()) {
		reset();
		return;
	}

	++m_NumAccelSamples;
}

void OnlineGyroCalibrator::updateGryo(
	const int16_t rawData[3],
	const sensor_real_t rawSample[3],
	SlimeVR::Configuration::SoftFusionSensorConfig& sensorConfig
) {
	m_RestDetection.updateGyr(rawSample);
	if (!m_RestDetection.getRestDetected()) {
		reset();
		return;
	}

	++m_NumGyroSamples;
	m_SumRawGryoData[0] += rawData[0];
	m_SumRawGryoData[1] += rawData[1];
	m_SumRawGryoData[2] += rawData[2];
	m_SumRawGryoSample[0] += rawSample[0];
	m_SumRawGryoSample[1] += rawSample[1];
	m_SumRawGryoSample[2] += rawSample[2];

	tryUpdateCalibration(sensorConfig);
}

void OnlineGyroCalibrator::reset() {
	m_NumAccelSamples = 0;

	if (m_NumGyroSamples != 0) {
		m_NumGyroSamples = 0;
		m_SumRawGryoData[0] = 0;
		m_SumRawGryoData[1] = 0;
		m_SumRawGryoData[2] = 0;
		m_SumRawGryoSample[0] = 0.0f;
		m_SumRawGryoSample[1] = 0.0f;
		m_SumRawGryoSample[2] = 0.0f;
	}
}

void OnlineGyroCalibrator::tryUpdateCalibration(
	SlimeVR::Configuration::SoftFusionSensorConfig& sensorConfig
) {
	if ((m_NumAccelSamples < SAMPLE_DURATION_IN_SEC / m_AccelTs)
		|| (m_NumGyroSamples < SAMPLE_DURATION_IN_SEC / m_GyroTs)) {
		return;
	}

	sensorConfig.G_off[0] = static_cast<float>(m_SumRawGryoData[0]) / m_NumGyroSamples;
	sensorConfig.G_off[1] = static_cast<float>(m_SumRawGryoData[1]) / m_NumGyroSamples;
	sensorConfig.G_off[2] = static_cast<float>(m_SumRawGryoData[2]) / m_NumGyroSamples;

	m_Logger.info(
		"Updated gryo calibration to [%f, %f, %f] LSB/(deg/s) (%f, %f, %f) deg/s",
		sensorConfig.G_off[0],
		sensorConfig.G_off[1],
		sensorConfig.G_off[2],
		m_SumRawGryoSample[0] / m_NumGyroSamples * (180.0f / M_PI),
		m_SumRawGryoSample[1] / m_NumGyroSamples * (180.0f / M_PI),
		m_SumRawGryoSample[2] / m_NumGyroSamples * (180.0f / M_PI)
	);

	reset();
}

}  // namespace SlimeVR::MotionProcessing
