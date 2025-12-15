//! Nanosensor trait definitions

use crate::error::NanoResult;
use crate::types::{SensorReading, SensorType, Signal, Calibration, SensorPerformance};
use async_trait::async_trait;

/// Trait for nanosensor operations
#[async_trait]
pub trait Nanosensor: Send + Sync {
    /// Get sensor ID
    fn sensor_id(&self) -> &str;

    /// Get sensor type
    fn sensor_type(&self) -> SensorType;

    /// Take a single reading
    async fn read(&mut self) -> NanoResult<SensorReading>;

    /// Take multiple readings (burst mode)
    async fn read_burst(&mut self, count: usize) -> NanoResult<Vec<SensorReading>>;

    /// Start continuous monitoring
    async fn start_monitoring(&mut self, interval_ms: u64) -> NanoResult<()>;

    /// Stop continuous monitoring
    async fn stop_monitoring(&mut self) -> NanoResult<()>;

    /// Get monitoring status
    fn is_monitoring(&self) -> bool;

    /// Get last reading
    fn last_reading(&self) -> Option<&SensorReading>;

    /// Get sensor performance specs
    fn performance(&self) -> &SensorPerformance;

    /// Get calibration info
    fn calibration(&self) -> Option<&Calibration>;

    /// Calibrate the sensor
    async fn calibrate(&mut self, reference: CalibrationReference) -> NanoResult<Calibration>;

    /// Get raw signal
    fn raw_signal(&self) -> Option<&Signal>;

    /// Set detection threshold
    fn set_threshold(&mut self, threshold: DetectionThreshold);

    /// Get current threshold
    fn threshold(&self) -> Option<&DetectionThreshold>;

    /// Check if target is detected
    fn is_target_detected(&self) -> bool;
}

/// Calibration reference for sensors
#[derive(Debug, Clone)]
pub struct CalibrationReference {
    pub reference_type: ReferenceType,
    pub known_value: f64,
    pub unit: String,
    pub tolerance: Option<f64>,
}

/// Type of calibration reference
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceType {
    /// Known concentration standard
    Standard,
    /// Zero/blank reference
    Blank,
    /// Multi-point calibration
    MultiPoint,
    /// Internal reference
    Internal,
}

/// Detection threshold configuration
#[derive(Debug, Clone)]
pub struct DetectionThreshold {
    pub lower_bound: Option<f64>,
    pub upper_bound: Option<f64>,
    pub unit: String,
    pub hysteresis: Option<f64>,
    pub trigger_mode: TriggerMode,
}

/// Threshold trigger mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerMode {
    /// Trigger when crossing above threshold
    Rising,
    /// Trigger when crossing below threshold
    Falling,
    /// Trigger on any crossing
    Both,
    /// Trigger when within range
    InRange,
    /// Trigger when outside range
    OutOfRange,
}

/// Chemical sensor specific trait
#[async_trait]
pub trait ChemicalSensor: Nanosensor {
    /// Get target analyte
    fn target_analyte(&self) -> &str;

    /// Get detection limit
    fn detection_limit(&self) -> f64;

    /// Get selectivity against interferents
    fn selectivity(&self) -> Vec<SelectivityInfo>;

    /// Measure concentration
    async fn measure_concentration(&mut self) -> NanoResult<Concentration>;
}

/// Selectivity information
#[derive(Debug, Clone)]
pub struct SelectivityInfo {
    pub interferent: String,
    pub cross_reactivity: f64, // 0.0 = no cross-reactivity, 1.0 = full
}

/// Concentration measurement
#[derive(Debug, Clone)]
pub struct Concentration {
    pub value: f64,
    pub unit: ConcentrationUnit,
    pub confidence: f64,
}

/// Concentration units
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConcentrationUnit {
    Molar,
    MilliMolar,
    MicroMolar,
    NanoMolar,
    PicoMolar,
    FemtoMolar,
    PartsPerMillion,
    PartsPerBillion,
}

/// Optical sensor specific trait
#[async_trait]
pub trait OpticalSensor: Nanosensor {
    /// Get wavelength range
    fn wavelength_range(&self) -> (f64, f64); // nm

    /// Measure fluorescence
    async fn measure_fluorescence(&mut self, excitation_nm: f64) -> NanoResult<FluorescenceReading>;

    /// Measure absorbance
    async fn measure_absorbance(&mut self, wavelength_nm: f64) -> NanoResult<f64>;

    /// Get quantum efficiency
    fn quantum_efficiency(&self) -> f64;
}

/// Fluorescence measurement
#[derive(Debug, Clone)]
pub struct FluorescenceReading {
    pub excitation_nm: f64,
    pub emission_nm: f64,
    pub intensity: f64,
    pub lifetime_ns: Option<f64>,
}

/// Sensor array trait for multi-sensor configurations
#[async_trait]
pub trait SensorArray: Send + Sync {
    /// Get number of sensors
    fn sensor_count(&self) -> usize;

    /// Get all sensor IDs
    fn sensor_ids(&self) -> Vec<String>;

    /// Read all sensors simultaneously
    async fn read_all(&mut self) -> NanoResult<Vec<SensorReading>>;

    /// Read specific sensor by ID
    async fn read_sensor(&mut self, sensor_id: &str) -> NanoResult<SensorReading>;

    /// Get array pattern/layout
    fn array_layout(&self) -> &ArrayLayout;

    /// Aggregate readings into composite
    fn aggregate(&self, readings: &[SensorReading]) -> AggregateReading;
}

/// Array sensor layout
#[derive(Debug, Clone)]
pub struct ArrayLayout {
    pub layout_type: LayoutType,
    pub dimensions: Vec<usize>,
    pub spacing_nm: f64,
}

/// Type of sensor array layout
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayoutType {
    Linear,
    Grid2D,
    Grid3D,
    Circular,
    Hexagonal,
    Custom,
}

/// Aggregated sensor reading
#[derive(Debug, Clone)]
pub struct AggregateReading {
    pub mean: f64,
    pub std_dev: f64,
    pub min: f64,
    pub max: f64,
    pub count: usize,
    pub unit: String,
}
