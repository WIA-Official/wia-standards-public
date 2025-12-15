//! Nanosensor implementation

use crate::error::{NanoError, NanoResult};
use crate::types::*;
use crate::traits::*;
use async_trait::async_trait;

/// Standard nanosensor implementation
pub struct StandardNanosensor {
    id: String,
    sensor_type: SensorType,
    status: SystemStatus,
    environment: Option<Environment>,
    scale: Option<Scale>,
    last_reading: Option<SensorReading>,
    raw_signal: Option<Signal>,
    calibration: Option<Calibration>,
    performance: SensorPerformance,
    threshold: Option<DetectionThreshold>,
    monitoring: bool,
    reading_count: u64,
}

impl StandardNanosensor {
    /// Create a new nanosensor
    pub fn new(id: impl Into<String>, sensor_type: SensorType) -> Self {
        Self {
            id: id.into(),
            sensor_type,
            status: SystemStatus::Offline,
            environment: None,
            scale: Some(Scale::default()),
            last_reading: None,
            raw_signal: None,
            calibration: None,
            performance: SensorPerformance {
                detection_limit: 1e-9,
                dynamic_range_min: 1e-9,
                dynamic_range_max: 1e-3,
                sensitivity: Some(0.9),
                specificity: Some(0.95),
                response_time_ms: Some(100.0),
            },
            threshold: None,
            monitoring: false,
            reading_count: 0,
        }
    }

    /// Create with builder
    pub fn builder(id: impl Into<String>, sensor_type: SensorType) -> NanosensorBuilder {
        NanosensorBuilder::new(id, sensor_type)
    }

    fn generate_reading(&mut self) -> SensorReading {
        use rand::Rng;
        let mut rng = rand::thread_rng();

        // Simulate realistic sensor reading
        let base_value = match self.sensor_type {
            SensorType::Ph => 7.0 + rng.gen_range(-0.5..0.5),
            SensorType::Temperature => 310.0 + rng.gen_range(-2.0..2.0), // ~37°C
            SensorType::Pressure => 101325.0 + rng.gen_range(-100.0..100.0),
            SensorType::Chemical => rng.gen_range(1e-9..1e-6),
            _ => rng.gen_range(0.0..100.0),
        };

        let unit = match self.sensor_type {
            SensorType::Ph => "pH",
            SensorType::Temperature => "K",
            SensorType::Pressure => "Pa",
            SensorType::Chemical => "M",
            SensorType::Optical => "RFU",
            SensorType::Magnetic => "T",
            SensorType::Ultrasonic => "Pa",
            SensorType::Electrical => "V",
        };

        self.reading_count += 1;

        SensorReading::new(&self.id, self.sensor_type, base_value, unit)
            .with_accuracy(self.performance.sensitivity.unwrap_or(0.9))
    }
}

/// Builder for StandardNanosensor
pub struct NanosensorBuilder {
    id: String,
    sensor_type: SensorType,
    environment: Option<Environment>,
    performance: Option<SensorPerformance>,
}

impl NanosensorBuilder {
    pub fn new(id: impl Into<String>, sensor_type: SensorType) -> Self {
        Self {
            id: id.into(),
            sensor_type,
            environment: None,
            performance: None,
        }
    }

    pub fn with_environment(mut self, env: Environment) -> Self {
        self.environment = Some(env);
        self
    }

    pub fn with_performance(mut self, perf: SensorPerformance) -> Self {
        self.performance = Some(perf);
        self
    }

    pub fn build(self) -> StandardNanosensor {
        let mut sensor = StandardNanosensor::new(self.id, self.sensor_type);
        sensor.environment = self.environment;
        if let Some(perf) = self.performance {
            sensor.performance = perf;
        }
        sensor
    }
}

#[async_trait]
impl NanoSystem for StandardNanosensor {
    fn system_type(&self) -> NanoSystemType {
        NanoSystemType::Nanosensor
    }

    fn id(&self) -> &str {
        &self.id
    }

    fn status(&self) -> SystemStatus {
        self.status
    }

    async fn initialize(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::Initializing;
        self.status = SystemStatus::Idle;
        Ok(())
    }

    async fn shutdown(&mut self) -> NanoResult<()> {
        self.status = SystemStatus::ShuttingDown;
        self.monitoring = false;
        self.status = SystemStatus::Offline;
        Ok(())
    }

    async fn reset(&mut self) -> NanoResult<()> {
        self.last_reading = None;
        self.raw_signal = None;
        self.monitoring = false;
        self.reading_count = 0;
        self.status = SystemStatus::Idle;
        Ok(())
    }

    fn environment(&self) -> Option<&Environment> {
        self.environment.as_ref()
    }

    fn set_environment(&mut self, env: Environment) {
        self.environment = Some(env);
    }

    fn scale(&self) -> Option<&Scale> {
        self.scale.as_ref()
    }

    fn to_message(&self) -> NanoResult<NanoMessage> {
        let payload = serde_json::json!({
            "system_type": "nanosensor",
            "sensor_type": self.sensor_type.as_str(),
            "monitoring": self.monitoring,
            "reading_count": self.reading_count,
            "last_reading": self.last_reading.as_ref().map(|r| r.value)
        });

        Ok(NanoMessage::builder()
            .device_id(&self.id)
            .system_type(NanoSystemType::Nanosensor)
            .payload(payload)?)
    }
}

#[async_trait]
impl Nanosensor for StandardNanosensor {
    fn sensor_id(&self) -> &str {
        &self.id
    }

    fn sensor_type(&self) -> SensorType {
        self.sensor_type
    }

    async fn read(&mut self) -> NanoResult<SensorReading> {
        if !self.is_operational() {
            return Err(NanoError::SensorError {
                sensor_id: self.id.clone(),
                message: "Sensor not operational".into(),
            });
        }

        self.status = SystemStatus::Active;
        let reading = self.generate_reading();
        self.last_reading = Some(reading.clone());
        self.status = SystemStatus::Idle;

        Ok(reading)
    }

    async fn read_burst(&mut self, count: usize) -> NanoResult<Vec<SensorReading>> {
        if !self.is_operational() {
            return Err(NanoError::SensorError {
                sensor_id: self.id.clone(),
                message: "Sensor not operational".into(),
            });
        }

        self.status = SystemStatus::Active;
        let readings: Vec<SensorReading> = (0..count)
            .map(|_| self.generate_reading())
            .collect();

        if let Some(last) = readings.last() {
            self.last_reading = Some(last.clone());
        }

        self.status = SystemStatus::Idle;
        Ok(readings)
    }

    async fn start_monitoring(&mut self, _interval_ms: u64) -> NanoResult<()> {
        if !self.is_operational() {
            return Err(NanoError::SensorError {
                sensor_id: self.id.clone(),
                message: "Sensor not operational".into(),
            });
        }

        self.monitoring = true;
        self.status = SystemStatus::Active;
        Ok(())
    }

    async fn stop_monitoring(&mut self) -> NanoResult<()> {
        self.monitoring = false;
        self.status = SystemStatus::Idle;
        Ok(())
    }

    fn is_monitoring(&self) -> bool {
        self.monitoring
    }

    fn last_reading(&self) -> Option<&SensorReading> {
        self.last_reading.as_ref()
    }

    fn performance(&self) -> &SensorPerformance {
        &self.performance
    }

    fn calibration(&self) -> Option<&Calibration> {
        self.calibration.as_ref()
    }

    async fn calibrate(&mut self, reference: CalibrationReference) -> NanoResult<Calibration> {
        let calibration = Calibration {
            last_calibrated: chrono::Utc::now().to_rfc3339(),
            curve_id: Some(format!("cal-{}", uuid::Uuid::new_v4())),
            r_squared: Some(0.999),
            valid_until: Some(
                (chrono::Utc::now() + chrono::Duration::days(30)).to_rfc3339()
            ),
        };

        self.calibration = Some(calibration.clone());
        Ok(calibration)
    }

    fn raw_signal(&self) -> Option<&Signal> {
        self.raw_signal.as_ref()
    }

    fn set_threshold(&mut self, threshold: DetectionThreshold) {
        self.threshold = Some(threshold);
    }

    fn threshold(&self) -> Option<&DetectionThreshold> {
        self.threshold.as_ref()
    }

    fn is_target_detected(&self) -> bool {
        if let (Some(reading), Some(threshold)) = (&self.last_reading, &self.threshold) {
            match threshold.trigger_mode {
                TriggerMode::Rising => {
                    threshold.upper_bound.map_or(false, |t| reading.value >= t)
                }
                TriggerMode::Falling => {
                    threshold.lower_bound.map_or(false, |t| reading.value <= t)
                }
                TriggerMode::Both => {
                    threshold.upper_bound.map_or(false, |t| reading.value >= t) ||
                    threshold.lower_bound.map_or(false, |t| reading.value <= t)
                }
                TriggerMode::InRange => {
                    let above_lower = threshold.lower_bound.map_or(true, |t| reading.value >= t);
                    let below_upper = threshold.upper_bound.map_or(true, |t| reading.value <= t);
                    above_lower && below_upper
                }
                TriggerMode::OutOfRange => {
                    let below_lower = threshold.lower_bound.map_or(false, |t| reading.value < t);
                    let above_upper = threshold.upper_bound.map_or(false, |t| reading.value > t);
                    below_lower || above_upper
                }
            }
        } else {
            false
        }
    }
}
