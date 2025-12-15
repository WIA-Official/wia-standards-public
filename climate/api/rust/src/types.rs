//! Type definitions for the WIA Climate Standard
//!
//! This module contains all the data structures that represent
//! the WIA Climate Standard data format.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

// ============================================================================
// Base Types
// ============================================================================

/// Data type identifier for climate messages
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataType {
    /// Carbon capture, utilization and storage
    CarbonCapture,
    /// Weather modification and cloud seeding
    WeatherControl,
    /// Climate intervention technologies
    Geoengineering,
    /// Indoor agriculture and controlled environment
    VerticalFarming,
    /// Marine pollution removal
    OceanCleanup,
    /// Climate simulation data
    ClimateModel,
    /// Custom/extension type
    Custom,
}

impl Default for DataType {
    fn default() -> Self {
        DataType::Custom
    }
}

impl std::fmt::Display for DataType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DataType::CarbonCapture => write!(f, "carbon_capture"),
            DataType::WeatherControl => write!(f, "weather_control"),
            DataType::Geoengineering => write!(f, "geoengineering"),
            DataType::VerticalFarming => write!(f, "vertical_farming"),
            DataType::OceanCleanup => write!(f, "ocean_cleanup"),
            DataType::ClimateModel => write!(f, "climate_model"),
            DataType::Custom => write!(f, "custom"),
        }
    }
}

/// Timestamp with both UNIX milliseconds and ISO 8601 format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timestamp {
    /// UNIX timestamp in milliseconds
    pub unix_ms: i64,
    /// ISO 8601 formatted timestamp (optional)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iso8601: Option<String>,
}

impl Timestamp {
    /// Create a new timestamp from current time
    pub fn now() -> Self {
        let now = Utc::now();
        Self {
            unix_ms: now.timestamp_millis(),
            iso8601: Some(now.to_rfc3339()),
        }
    }

    /// Create a timestamp from UNIX milliseconds
    pub fn from_unix_ms(unix_ms: i64) -> Self {
        let dt = DateTime::from_timestamp_millis(unix_ms)
            .unwrap_or_else(Utc::now);
        Self {
            unix_ms,
            iso8601: Some(dt.to_rfc3339()),
        }
    }

    /// Create a timestamp from a DateTime
    pub fn from_datetime(dt: DateTime<Utc>) -> Self {
        Self {
            unix_ms: dt.timestamp_millis(),
            iso8601: Some(dt.to_rfc3339()),
        }
    }
}

impl Default for Timestamp {
    fn default() -> Self {
        Self::now()
    }
}

/// Geographic location
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Location {
    /// Latitude in degrees (-90 to 90)
    pub latitude: f64,
    /// Longitude in degrees (-180 to 180)
    pub longitude: f64,
    /// Altitude in meters above sea level
    #[serde(skip_serializing_if = "Option::is_none")]
    pub altitude_m: Option<f64>,
    /// Coordinate Reference System
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crs: Option<String>,
}

impl Location {
    /// Create a new location with latitude and longitude
    pub fn new(latitude: f64, longitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude_m: None,
            crs: Some("EPSG:4326".to_string()),
        }
    }

    /// Create a location with altitude
    pub fn with_altitude(mut self, altitude_m: f64) -> Self {
        self.altitude_m = Some(altitude_m);
        self
    }

    /// Validate the location coordinates
    pub fn is_valid(&self) -> bool {
        self.latitude >= -90.0 && self.latitude <= 90.0
            && self.longitude >= -180.0 && self.longitude <= 180.0
    }
}

impl Default for Location {
    fn default() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            altitude_m: None,
            crs: Some("EPSG:4326".to_string()),
        }
    }
}

/// Device/sensor information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Device {
    /// Manufacturer or data provider name
    pub manufacturer: String,
    /// Model or system name
    pub model: String,
    /// Serial number or identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub serial: Option<String>,
    /// Firmware or software version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub firmware: Option<String>,
}

impl Device {
    /// Create a new device with manufacturer and model
    pub fn new(manufacturer: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            manufacturer: manufacturer.into(),
            model: model.into(),
            serial: None,
            firmware: None,
        }
    }

    /// Set serial number
    pub fn with_serial(mut self, serial: impl Into<String>) -> Self {
        self.serial = Some(serial.into());
        self
    }

    /// Set firmware version
    pub fn with_firmware(mut self, firmware: impl Into<String>) -> Self {
        self.firmware = Some(firmware.into());
        self
    }
}

impl Default for Device {
    fn default() -> Self {
        Self {
            manufacturer: "Unknown".to_string(),
            model: "Unknown".to_string(),
            serial: None,
            firmware: None,
        }
    }
}

/// Data source type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataSource {
    /// Direct sensor measurement
    Sensor,
    /// Model/simulation output
    Model,
    /// Manual entry
    Manual,
    /// Satellite observation
    Satellite,
    /// Derived from other data
    Derived,
}

/// Data processing level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProcessingLevel {
    /// Raw, unprocessed data
    Raw,
    /// Calibrated data
    Calibrated,
    /// Quality-validated data
    Validated,
    /// Derived/computed data
    Derived,
}

/// Metadata for the message
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Metadata {
    /// Data quality score (0.0 to 1.0)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quality_score: Option<f64>,
    /// Measurement uncertainty
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uncertainty: Option<f64>,
    /// Data source type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<DataSource>,
    /// Data processing level
    #[serde(skip_serializing_if = "Option::is_none")]
    pub processing_level: Option<ProcessingLevel>,
}

impl Metadata {
    /// Create new metadata with quality score
    pub fn with_quality(quality_score: f64) -> Self {
        Self {
            quality_score: Some(quality_score),
            ..Default::default()
        }
    }

    /// Set the data source
    pub fn source(mut self, source: DataSource) -> Self {
        self.source = Some(source);
        self
    }

    /// Set the processing level
    pub fn processing_level(mut self, level: ProcessingLevel) -> Self {
        self.processing_level = Some(level);
        self
    }
}

// ============================================================================
// Carbon Capture Types
// ============================================================================

/// Carbon capture technology type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CarbonCaptureTechnology {
    /// Direct Air Capture
    Dac,
    /// Post-combustion capture
    PostCombustion,
    /// Pre-combustion capture
    PreCombustion,
    /// Oxy-fuel combustion
    OxyFuel,
    /// Bioenergy with CCS
    BioenergyCcs,
}

impl Default for CarbonCaptureTechnology {
    fn default() -> Self {
        CarbonCaptureTechnology::Dac
    }
}

/// Sorbent/capture material type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SorbentType {
    /// Solid amine sorbent
    SolidAmine,
    /// Liquid amine sorbent
    LiquidAmine,
    /// Calcium looping
    CalciumLooping,
    /// Membrane separation
    Membrane,
    /// Cryogenic separation
    Cryogenic,
}

/// CO2 storage method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StorageMethod {
    /// Geological storage
    Geological,
    /// Ocean storage
    Ocean,
    /// Mineral carbonation
    Mineral,
    /// CO2 utilization
    Utilization,
    /// Enhanced oil recovery
    EnhancedOilRecovery,
}

/// Geological formation type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FormationType {
    /// Saline aquifer
    SalineAquifer,
    /// Depleted oil/gas reservoir
    DepletedReservoir,
    /// Basalt formation
    Basalt,
    /// Coal seam
    CoalSeam,
}

/// Sorbent status data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SorbentStatus {
    /// Sorbent type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub sorbent_type: Option<SorbentType>,
    /// Efficiency percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub efficiency_percentage: Option<f64>,
    /// Number of regeneration cycles completed
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cycles_completed: Option<u32>,
    /// Operating temperature in Celsius
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature_celsius: Option<f64>,
}

/// CO2 storage data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StorageData {
    /// Storage method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<StorageMethod>,
    /// Storage pressure in MPa
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pressure_mpa: Option<f64>,
    /// Storage depth in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub depth_m: Option<f64>,
    /// Formation type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub formation_type: Option<FormationType>,
    /// Injection rate in kg/hour
    #[serde(skip_serializing_if = "Option::is_none")]
    pub injection_rate_kg_per_hour: Option<f64>,
}

/// Carbon capture data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CarbonCaptureData {
    /// Capture technology type
    pub technology: CarbonCaptureTechnology,
    /// CO2 capture rate in kg/hour
    pub capture_rate_kg_per_hour: f64,
    /// Ambient CO2 concentration in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub co2_concentration_ppm: Option<f64>,
    /// Captured CO2 purity percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub co2_purity_percentage: Option<f64>,
    /// Energy consumption in kWh
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_consumption_kwh: Option<f64>,
    /// Sorbent status
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sorbent_status: Option<SorbentStatus>,
    /// Storage information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub storage: Option<StorageData>,
    /// Cumulative CO2 captured in tonnes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cumulative_captured_tonnes: Option<f64>,
}

// ============================================================================
// Weather Control Types
// ============================================================================

/// Weather modification operation type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WeatherOperationType {
    /// Cloud seeding for precipitation
    CloudSeeding,
    /// Fog dispersal
    FogDispersal,
    /// Hail suppression
    HailSuppression,
    /// Rain enhancement
    RainEnhancement,
}

impl Default for WeatherOperationType {
    fn default() -> Self {
        WeatherOperationType::CloudSeeding
    }
}

/// Seeding agent type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SeedingAgentType {
    /// Silver iodide (AgI)
    SilverIodide,
    /// Potassium iodide (KI)
    PotassiumIodide,
    /// Dry ice (solid CO2)
    DryIce,
    /// Liquid propane
    LiquidPropane,
    /// Salt particles
    Salt,
    /// Calcium chloride
    CalciumChloride,
}

/// Agent delivery method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeliveryMethod {
    /// Aircraft dispersal
    Aircraft,
    /// Ground-based generator
    GroundGenerator,
    /// Rocket delivery
    Rocket,
    /// Drone dispersal
    Drone,
    /// Artillery shell
    Artillery,
}

/// Cloud type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CloudType {
    /// Cumulus clouds
    Cumulus,
    /// Cumulonimbus (storm) clouds
    Cumulonimbus,
    /// Stratus clouds
    Stratus,
    /// Stratocumulus clouds
    Stratocumulus,
    /// Cirrus clouds
    Cirrus,
    /// Orographic (mountain) clouds
    Orographic,
}

/// Seeding agent data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SeedingAgent {
    /// Agent type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub agent_type: Option<SeedingAgentType>,
    /// Mass deployed in grams
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mass_grams: Option<f64>,
    /// Concentration in g/m³
    #[serde(skip_serializing_if = "Option::is_none")]
    pub concentration_g_per_m3: Option<f64>,
}

/// Target cloud data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TargetCloud {
    /// Cloud type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub cloud_type: Option<CloudType>,
    /// Cloud base altitude in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base_altitude_m: Option<f64>,
    /// Cloud top altitude in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top_altitude_m: Option<f64>,
    /// Cloud coverage area in km²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coverage_km2: Option<f64>,
}

/// Atmospheric conditions
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AtmosphericConditions {
    /// Air temperature in Celsius
    pub temperature_celsius: f64,
    /// Relative humidity percentage
    pub humidity_percentage: f64,
    /// Wind speed in m/s
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wind_speed_m_per_s: Option<f64>,
    /// Wind direction in degrees
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wind_direction_deg: Option<f64>,
    /// Atmospheric pressure in hPa
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pressure_hpa: Option<f64>,
}

/// Weather operation result
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WeatherResult {
    /// Resulting precipitation in mm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub precipitation_mm: Option<f64>,
    /// Precipitation duration in hours
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration_hours: Option<f64>,
    /// Estimated effectiveness percentage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub effectiveness_percentage: Option<f64>,
    /// Baseline expected precipitation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub baseline_precipitation_mm: Option<f64>,
}

/// Weather control data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WeatherControlData {
    /// Operation type
    pub operation_type: WeatherOperationType,
    /// Seeding agent information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub seeding_agent: Option<SeedingAgent>,
    /// Delivery method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivery_method: Option<DeliveryMethod>,
    /// Target cloud information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_cloud: Option<TargetCloud>,
    /// Atmospheric conditions
    pub atmospheric_conditions: AtmosphericConditions,
    /// Operation result
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<WeatherResult>,
}

// ============================================================================
// Geoengineering Types
// ============================================================================

/// Geoengineering intervention type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InterventionType {
    /// Stratospheric Aerosol Injection
    StratosphericAerosolInjection,
    /// Marine Cloud Brightening
    MarineCloudBrightening,
    /// Ocean Fertilization
    OceanFertilization,
    /// Enhanced Weathering
    EnhancedWeathering,
    /// Space Reflector
    SpaceReflector,
    /// Direct Air Capture
    DirectAirCapture,
    /// Biochar production
    Biochar,
    /// Afforestation
    Afforestation,
}

impl Default for InterventionType {
    fn default() -> Self {
        InterventionType::StratosphericAerosolInjection
    }
}

/// Geoengineering category
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GeoengineeringCategory {
    /// Solar Radiation Management
    SolarRadiationManagement,
    /// Carbon Dioxide Removal
    CarbonDioxideRemoval,
}

impl Default for GeoengineeringCategory {
    fn default() -> Self {
        GeoengineeringCategory::SolarRadiationManagement
    }
}

/// Aerosol type for SRM
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AerosolType {
    /// Sulfur dioxide
    SulfurDioxide,
    /// Sulfate particles
    Sulfate,
    /// Calcium carbonate
    CalciumCarbite,
    /// Sea salt
    SeaSalt,
    /// Titanium dioxide
    TitaniumDioxide,
}

/// Deployment platform
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeploymentPlatform {
    /// Aircraft deployment
    Aircraft,
    /// High-altitude balloon
    Balloon,
    /// Ship-based deployment
    Ship,
    /// Ground-based systems
    GroundBased,
}

/// Geoengineering deployment data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GeoengineeringDeployment {
    /// Aerosol type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aerosol_type: Option<AerosolType>,
    /// Injection altitude in km
    #[serde(skip_serializing_if = "Option::is_none")]
    pub injection_altitude_km: Option<f64>,
    /// Injection rate in kg/day
    #[serde(skip_serializing_if = "Option::is_none")]
    pub injection_rate_kg_per_day: Option<f64>,
    /// Particle size in micrometers
    #[serde(skip_serializing_if = "Option::is_none")]
    pub particle_size_um: Option<f64>,
    /// Deployment platform
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivery_platform: Option<DeploymentPlatform>,
}

/// Coverage area data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CoverageArea {
    /// Coverage area in km²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub area_km2: Option<f64>,
    /// Latitude range [min, max]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub latitude_range: Option<[f64; 2]>,
    /// Longitude range [min, max]
    #[serde(skip_serializing_if = "Option::is_none")]
    pub longitude_range: Option<[f64; 2]>,
}

/// Climate effects
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ClimateEffects {
    /// Radiative forcing change in W/m²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radiative_forcing_w_per_m2: Option<f64>,
    /// Temperature change in °C
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature_change_celsius: Option<f64>,
    /// Precipitation change in %
    #[serde(skip_serializing_if = "Option::is_none")]
    pub precipitation_change_percentage: Option<f64>,
    /// CO2 removal rate for CDR methods
    #[serde(skip_serializing_if = "Option::is_none")]
    pub co2_removal_tonnes_per_year: Option<f64>,
}

/// Environmental monitoring data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EnvironmentalMonitoring {
    /// Impact on ozone layer in %
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ozone_impact_percentage: Option<f64>,
    /// Acid deposition increase in %
    #[serde(skip_serializing_if = "Option::is_none")]
    pub acid_deposition_increase_percentage: Option<f64>,
    /// Ecosystem impact score (0-1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ecosystem_impact_score: Option<f64>,
}

/// Geoengineering data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GeoengineeringData {
    /// Intervention type
    pub intervention_type: InterventionType,
    /// Category (SRM or CDR)
    pub category: GeoengineeringCategory,
    /// Deployment information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deployment: Option<GeoengineeringDeployment>,
    /// Coverage area
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coverage: Option<CoverageArea>,
    /// Climate effects
    #[serde(skip_serializing_if = "Option::is_none")]
    pub effects: Option<ClimateEffects>,
    /// Environmental monitoring
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monitoring: Option<EnvironmentalMonitoring>,
}

// ============================================================================
// Vertical Farming Types
// ============================================================================

/// Cultivation system type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FarmingSystemType {
    /// Hydroponic cultivation
    Hydroponics,
    /// Aeroponic cultivation
    Aeroponics,
    /// Aquaponic system
    Aquaponics,
    /// Substrate-based cultivation
    Substrate,
    /// Hybrid system
    Hybrid,
}

impl Default for FarmingSystemType {
    fn default() -> Self {
        FarmingSystemType::Hydroponics
    }
}

/// Light source type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LightSourceType {
    /// LED lighting
    Led,
    /// High-pressure sodium
    Hps,
    /// Fluorescent
    Fluorescent,
    /// Metal halide
    MetalHalide,
    /// Hybrid lighting
    Hybrid,
}

/// Light spectrum type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LightSpectrum {
    /// Full spectrum
    FullSpectrum,
    /// Red and blue optimized
    RedBlue,
    /// Warm white
    WarmWhite,
    /// Cool white
    CoolWhite,
    /// Custom spectrum
    Custom,
}

/// Plant growth stage
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GrowthStage {
    /// Germination phase
    Germination,
    /// Seedling phase
    Seedling,
    /// Vegetative growth
    Vegetative,
    /// Flowering phase
    Flowering,
    /// Fruiting phase
    Fruiting,
    /// Harvest ready
    Harvest,
}

/// Farming environment data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FarmingEnvironment {
    /// Air temperature in Celsius
    pub temperature_celsius: f64,
    /// Relative humidity percentage
    pub humidity_percentage: f64,
    /// CO2 concentration in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub co2_ppm: Option<f64>,
    /// Vapor Pressure Deficit in kPa
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vpd_kpa: Option<f64>,
    /// Air flow rate in m³/hour
    #[serde(skip_serializing_if = "Option::is_none")]
    pub air_flow_m3_per_hour: Option<f64>,
}

/// Lighting data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LightingData {
    /// Light source type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub light_type: Option<LightSourceType>,
    /// PPFD in µmol/m²/s
    pub ppfd_umol_per_m2_s: f64,
    /// Daily Light Integral in mol/m²/day
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dli_mol_per_m2_day: Option<f64>,
    /// Light spectrum
    #[serde(skip_serializing_if = "Option::is_none")]
    pub spectrum: Option<LightSpectrum>,
    /// Photoperiod in hours
    pub photoperiod_hours: f64,
    /// Energy consumption in kWh
    #[serde(skip_serializing_if = "Option::is_none")]
    pub energy_consumption_kwh: Option<f64>,
}

/// Nutrient element concentrations
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NutrientElements {
    /// Nitrogen in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nitrogen_ppm: Option<f64>,
    /// Phosphorus in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub phosphorus_ppm: Option<f64>,
    /// Potassium in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub potassium_ppm: Option<f64>,
    /// Calcium in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calcium_ppm: Option<f64>,
    /// Magnesium in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnesium_ppm: Option<f64>,
    /// Sulfur in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sulfur_ppm: Option<f64>,
    /// Iron in ppm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iron_ppm: Option<f64>,
}

/// Nutrient solution data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NutrientSolution {
    /// Solution pH
    pub ph: f64,
    /// Electrical conductivity in mS/cm
    pub ec_ms_per_cm: f64,
    /// Solution temperature in Celsius
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature_celsius: Option<f64>,
    /// Dissolved oxygen in mg/L
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dissolved_oxygen_mg_per_l: Option<f64>,
    /// Nutrient element concentrations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elements: Option<NutrientElements>,
}

/// Crop data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CropData {
    /// Plant species (scientific name)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub species: Option<String>,
    /// Plant variety/cultivar
    #[serde(skip_serializing_if = "Option::is_none")]
    pub variety: Option<String>,
    /// Current growth stage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub growth_stage: Option<GrowthStage>,
    /// Days since planting
    #[serde(skip_serializing_if = "Option::is_none")]
    pub days_after_planting: Option<u32>,
    /// Number of plants
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plant_count: Option<u32>,
    /// Planting density per m²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub density_plants_per_m2: Option<f64>,
}

/// Yield data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct YieldData {
    /// Fresh weight harvest in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fresh_weight_kg: Option<f64>,
    /// Dry weight harvest in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dry_weight_kg: Option<f64>,
    /// Growing area in m²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub area_m2: Option<f64>,
    /// Yield per area in kg/m²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub kg_per_m2: Option<f64>,
    /// Days per harvest cycle
    #[serde(skip_serializing_if = "Option::is_none")]
    pub harvest_cycle_days: Option<u32>,
}

/// Vertical farming data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VerticalFarmingData {
    /// Cultivation system type
    pub system_type: FarmingSystemType,
    /// Environment data
    pub environment: FarmingEnvironment,
    /// Lighting data
    pub lighting: LightingData,
    /// Nutrient solution data
    pub nutrient_solution: NutrientSolution,
    /// Crop information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crop: Option<CropData>,
    /// Yield data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub yield_data: Option<YieldData>,
}

// ============================================================================
// Ocean Cleanup Types
// ============================================================================

/// Cleanup operation type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CleanupOperationType {
    /// Floating barrier system
    FloatingBarrier,
    /// River interceptor
    RiverInterceptor,
    /// Beach cleanup
    BeachCleanup,
    /// Autonomous cleanup vessel
    AutonomousVessel,
    /// Drone-based collection
    DroneCollection,
    /// Diver collection
    DiverCollection,
}

impl Default for CleanupOperationType {
    fn default() -> Self {
        CleanupOperationType::FloatingBarrier
    }
}

/// Vessel type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VesselType {
    /// Support vessel
    SupportVessel,
    /// Collection barge
    CollectionBarge,
    /// Autonomous drone
    AutonomousDrone,
    /// River interceptor
    Interceptor,
}

/// Ocean zone identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OceanZone {
    /// Great Pacific Garbage Patch
    GreatPacificGarbagePatch,
    /// North Atlantic Gyre
    NorthAtlanticGyre,
    /// South Atlantic Gyre
    SouthAtlanticGyre,
    /// Indian Ocean Gyre
    IndianOceanGyre,
    /// South Pacific Gyre
    SouthPacificGyre,
    /// Coastal waters
    Coastal,
    /// River mouth
    RiverMouth,
    /// Harbor area
    Harbor,
}

/// Debris categories
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DebrisCategories {
    /// Number of bottles collected
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bottles_count: Option<u32>,
    /// Number of bags collected
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bags_count: Option<u32>,
    /// Foam material in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub foam_kg: Option<f64>,
    /// Rope length in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rope_m: Option<f64>,
}

/// Collection data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CollectionData {
    /// Total debris mass in kg
    pub total_mass_kg: f64,
    /// Plastic mass in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub plastic_mass_kg: Option<f64>,
    /// Fishing gear/nets mass in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fishing_gear_kg: Option<f64>,
    /// Other debris mass in kg
    #[serde(skip_serializing_if = "Option::is_none")]
    pub other_debris_kg: Option<f64>,
    /// Microplastic particle count
    #[serde(skip_serializing_if = "Option::is_none")]
    pub microplastic_count: Option<u64>,
    /// Microplastic size range [min, max] in mm
    #[serde(skip_serializing_if = "Option::is_none")]
    pub microplastic_size_range_mm: Option<[f64; 2]>,
    /// Detailed debris categories
    #[serde(skip_serializing_if = "Option::is_none")]
    pub debris_categories: Option<DebrisCategories>,
}

/// Area/operation data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OperationArea {
    /// Area covered in km²
    pub swept_km2: f64,
    /// Operation duration in hours
    pub duration_hours: f64,
    /// Collection efficiency in kg/km²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub efficiency_kg_per_km2: Option<f64>,
    /// Collection rate in kg/hour
    #[serde(skip_serializing_if = "Option::is_none")]
    pub efficiency_kg_per_hour: Option<f64>,
}

/// Water conditions
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WaterConditions {
    /// Water temperature in Celsius
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature_celsius: Option<f64>,
    /// Salinity in parts per thousand
    #[serde(skip_serializing_if = "Option::is_none")]
    pub salinity_ppt: Option<f64>,
    /// Water pH
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ph: Option<f64>,
    /// Current speed in m/s
    #[serde(skip_serializing_if = "Option::is_none")]
    pub current_speed_m_per_s: Option<f64>,
    /// Current direction in degrees
    #[serde(skip_serializing_if = "Option::is_none")]
    pub current_direction_deg: Option<f64>,
    /// Wave height in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub wave_height_m: Option<f64>,
    /// Water visibility in meters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub visibility_m: Option<f64>,
}

/// Vessel information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VesselInfo {
    /// Vessel type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub vessel_type: Option<VesselType>,
    /// Vessel name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Fuel consumption in L/hour
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fuel_consumption_l_per_hour: Option<f64>,
    /// Number of crew members
    #[serde(skip_serializing_if = "Option::is_none")]
    pub crew_count: Option<u32>,
}

/// Ocean cleanup data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct OceanCleanupData {
    /// Operation type
    pub operation_type: CleanupOperationType,
    /// Collection data
    pub collection: CollectionData,
    /// Operation area/stats
    pub area: OperationArea,
    /// Water conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub water_conditions: Option<WaterConditions>,
    /// Vessel information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vessel: Option<VesselInfo>,
    /// Ocean zone
    #[serde(skip_serializing_if = "Option::is_none")]
    pub zone: Option<OceanZone>,
}

// ============================================================================
// Climate Model Types
// ============================================================================

/// Time frequency (CF Conventions)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimeFrequency {
    /// Fixed/constant value
    Fx,
    /// Annual
    Yr,
    /// Monthly
    Mon,
    /// Daily
    Day,
    /// 6-hourly
    #[serde(rename = "6hr")]
    SixHourly,
    /// 3-hourly
    #[serde(rename = "3hr")]
    ThreeHourly,
    /// Hourly
    #[serde(rename = "1hr")]
    Hourly,
    /// Sub-hourly
    Subhr,
}

impl Default for TimeFrequency {
    fn default() -> Self {
        TimeFrequency::Mon
    }
}

/// SSP scenario
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SspScenario {
    /// SSP1-1.9 (very low emissions)
    #[serde(rename = "SSP1-1.9")]
    Ssp119,
    /// SSP1-2.6 (low emissions)
    #[serde(rename = "SSP1-2.6")]
    Ssp126,
    /// SSP2-4.5 (middle of the road)
    #[serde(rename = "SSP2-4.5")]
    Ssp245,
    /// SSP3-7.0 (regional rivalry)
    #[serde(rename = "SSP3-7.0")]
    Ssp370,
    /// SSP5-8.5 (fossil-fueled development)
    #[serde(rename = "SSP5-8.5")]
    Ssp585,
}

impl Default for SspScenario {
    fn default() -> Self {
        SspScenario::Ssp245
    }
}

/// Grid type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GridType {
    /// Regular latitude-longitude
    RegularLatLon,
    /// Gaussian grid
    Gaussian,
    /// Curvilinear grid
    Curvilinear,
    /// Unstructured grid
    Unstructured,
}

/// Calendar type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalendarType {
    /// Standard Gregorian
    Gregorian,
    /// Standard calendar
    Standard,
    /// Proleptic Gregorian
    ProlepticGregorian,
    /// 365-day calendar (no leap years)
    #[serde(rename = "365_day")]
    NoLeap,
    /// 360-day calendar
    #[serde(rename = "360_day")]
    Calendar360Day,
}

/// Model information (CMIP6)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ModelInfo {
    /// Model identifier (source_id)
    pub source_id: String,
    /// Institution identifier
    #[serde(skip_serializing_if = "Option::is_none")]
    pub institution_id: Option<String>,
    /// Experiment identifier
    pub experiment_id: String,
    /// Ensemble member label (e.g., r1i1p1f1)
    pub variant_label: String,
    /// Grid label
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grid_label: Option<String>,
}

/// Variable information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct VariableInfo {
    /// Variable name
    pub name: String,
    /// Long descriptive name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub long_name: Option<String>,
    /// CF standard name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub standard_name: Option<String>,
    /// Variable units
    pub units: String,
    /// Cell methods
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cell_methods: Option<String>,
}

/// Grid information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct GridInfo {
    /// Resolution in degrees
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolution_deg: Option<f64>,
    /// Resolution in km
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolution_km: Option<f64>,
    /// Number of latitude points
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nlat: Option<u32>,
    /// Number of longitude points
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nlon: Option<u32>,
    /// Grid type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grid_type: Option<GridType>,
}

/// Time information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TimeInfo {
    /// Start date (YYYY-MM-DD)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_date: Option<String>,
    /// End date (YYYY-MM-DD)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_date: Option<String>,
    /// Time frequency
    pub frequency: TimeFrequency,
    /// Calendar type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calendar: Option<CalendarType>,
}

/// Model output value
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ModelValue {
    /// Data value
    pub data: f64,
    /// Anomaly from climatology
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anomaly: Option<f64>,
    /// Climatological mean
    #[serde(skip_serializing_if = "Option::is_none")]
    pub climatology: Option<f64>,
    /// 5th percentile
    #[serde(skip_serializing_if = "Option::is_none")]
    pub percentile_5: Option<f64>,
    /// 50th percentile (median)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub percentile_50: Option<f64>,
    /// 95th percentile
    #[serde(skip_serializing_if = "Option::is_none")]
    pub percentile_95: Option<f64>,
    /// Standard deviation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub standard_deviation: Option<f64>,
}

/// Scenario information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ScenarioInfo {
    /// SSP scenario
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ssp: Option<SspScenario>,
    /// Radiative forcing level in W/m²
    #[serde(skip_serializing_if = "Option::is_none")]
    pub forcing_level_w_per_m2: Option<f64>,
    /// Scenario description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// Reference information
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ReferenceInfo {
    /// ES-DOC reference URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub further_info_url: Option<String>,
    /// Dataset DOI
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doi: Option<String>,
    /// Citation string
    #[serde(skip_serializing_if = "Option::is_none")]
    pub citation: Option<String>,
}

/// Climate model data
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ClimateModelData {
    /// Model information
    pub model: ModelInfo,
    /// Variable information
    pub variable: VariableInfo,
    /// Grid information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grid: Option<GridInfo>,
    /// Time information
    pub time: TimeInfo,
    /// Value data
    pub value: ModelValue,
    /// Scenario information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scenario: Option<ScenarioInfo>,
    /// Reference information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference: Option<ReferenceInfo>,
}

// ============================================================================
// Union Data Type
// ============================================================================

/// Union type for all domain-specific data
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ClimateData {
    /// Carbon capture data
    CarbonCapture(CarbonCaptureData),
    /// Weather control data
    WeatherControl(WeatherControlData),
    /// Geoengineering data
    Geoengineering(GeoengineeringData),
    /// Vertical farming data
    VerticalFarming(VerticalFarmingData),
    /// Ocean cleanup data
    OceanCleanup(OceanCleanupData),
    /// Climate model data
    ClimateModel(ClimateModelData),
    /// Custom data (JSON value)
    Custom(serde_json::Value),
}

impl Default for ClimateData {
    fn default() -> Self {
        ClimateData::Custom(serde_json::Value::Null)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_data_type_display() {
        assert_eq!(DataType::CarbonCapture.to_string(), "carbon_capture");
        assert_eq!(DataType::ClimateModel.to_string(), "climate_model");
    }

    #[test]
    fn test_timestamp_now() {
        let ts = Timestamp::now();
        assert!(ts.unix_ms > 0);
        assert!(ts.iso8601.is_some());
    }

    #[test]
    fn test_location_validation() {
        let valid = Location::new(35.5, 127.0);
        assert!(valid.is_valid());

        let invalid = Location::new(100.0, 200.0);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn test_device_builder() {
        let device = Device::new("Climeworks", "Orca DAC")
            .with_serial("DAC-001")
            .with_firmware("2.1.0");

        assert_eq!(device.manufacturer, "Climeworks");
        assert_eq!(device.serial, Some("DAC-001".to_string()));
    }

    #[test]
    fn test_carbon_capture_serialization() {
        let data = CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 125.5,
            ..Default::default()
        };

        let json = serde_json::to_string(&data).unwrap();
        assert!(json.contains("dac"));
        assert!(json.contains("125.5"));
    }
}
