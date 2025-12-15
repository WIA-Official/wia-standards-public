//! Simulator adapters for testing and development
//!
//! These adapters generate simulated climate data for testing purposes.

use crate::core::ClimateAdapter;
use crate::error::{ClimateError, Result};
use crate::types::*;
use crate::ClimateMessage;
use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

/// Simulated carbon capture data adapter
///
/// Generates realistic carbon capture data for testing and development.
///
/// # Example
///
/// ```rust
/// use wia_climate::prelude::*;
///
/// #[tokio::main]
/// async fn main() {
///     let adapter = CarbonCaptureSimulator::new(
///         Location::new(64.0, -21.0),
///         Device::new("Climeworks", "Orca DAC Simulator"),
///     );
///
///     let message = adapter.read().await.unwrap();
///     println!("Capture rate: {:?}", message.data);
/// }
/// ```
pub struct CarbonCaptureSimulator {
    location: Location,
    device: Device,
    sequence: AtomicU64,
    is_streaming: AtomicBool,
    /// Base capture rate in kg/hour
    pub base_capture_rate: f64,
    /// Variation factor (0.0 to 1.0)
    pub variation: f64,
}

impl CarbonCaptureSimulator {
    /// Create a new carbon capture simulator
    pub fn new(location: Location, device: Device) -> Self {
        Self {
            location,
            device,
            sequence: AtomicU64::new(0),
            is_streaming: AtomicBool::new(false),
            base_capture_rate: 125.0,
            variation: 0.1,
        }
    }

    /// Set the base capture rate
    pub fn with_capture_rate(mut self, rate: f64) -> Self {
        self.base_capture_rate = rate;
        self
    }

    /// Set the variation factor
    pub fn with_variation(mut self, variation: f64) -> Self {
        self.variation = variation.clamp(0.0, 1.0);
        self
    }

    fn generate_capture_rate(&self) -> f64 {
        let variation = (rand_simple() - 0.5) * 2.0 * self.variation;
        self.base_capture_rate * (1.0 + variation)
    }
}

#[async_trait]
impl ClimateAdapter for CarbonCaptureSimulator {
    fn name(&self) -> &str {
        "CarbonCaptureSimulator"
    }

    fn is_connected(&self) -> bool {
        true
    }

    async fn read(&self) -> Result<ClimateMessage> {
        let seq = self.sequence.fetch_add(1, Ordering::SeqCst);

        let data = CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: self.generate_capture_rate(),
            co2_concentration_ppm: Some(415.0 + rand_simple() * 5.0),
            co2_purity_percentage: Some(99.0 + rand_simple()),
            energy_consumption_kwh: Some(2000.0 + rand_simple() * 500.0),
            sorbent_status: Some(SorbentStatus {
                sorbent_type: Some(SorbentType::SolidAmine),
                efficiency_percentage: Some(85.0 + rand_simple() * 5.0),
                cycles_completed: Some((seq as u32) % 10000),
                temperature_celsius: Some(80.0 + rand_simple() * 20.0),
            }),
            storage: Some(StorageData {
                method: Some(StorageMethod::Geological),
                pressure_mpa: Some(10.0 + rand_simple() * 2.0),
                depth_m: Some(2000.0),
                formation_type: Some(FormationType::Basalt),
                injection_rate_kg_per_hour: Some(self.generate_capture_rate() * 0.95),
            }),
            cumulative_captured_tonnes: Some((seq as f64) * self.base_capture_rate / 1000.0),
        };

        ClimateMessage::builder()
            .location(self.location.clone())
            .device(self.device.clone())
            .carbon_capture_data(data)
            .meta(
                Metadata::with_quality(0.95)
                    .source(DataSource::Sensor)
                    .processing_level(ProcessingLevel::Calibrated),
            )
            .build()
    }

    async fn start_stream(&mut self) -> Result<()> {
        self.is_streaming.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn stop_stream(&mut self) -> Result<()> {
        self.is_streaming.store(false, Ordering::SeqCst);
        Ok(())
    }
}

/// Simulated vertical farming data adapter
pub struct VerticalFarmingSimulator {
    location: Location,
    device: Device,
    sequence: AtomicU64,
    is_streaming: AtomicBool,
    /// Target temperature in Celsius
    pub target_temperature: f64,
    /// Target humidity percentage
    pub target_humidity: f64,
}

impl VerticalFarmingSimulator {
    /// Create a new vertical farming simulator
    pub fn new(location: Location, device: Device) -> Self {
        Self {
            location,
            device,
            sequence: AtomicU64::new(0),
            is_streaming: AtomicBool::new(false),
            target_temperature: 22.0,
            target_humidity: 65.0,
        }
    }

    /// Set target temperature
    pub fn with_temperature(mut self, temp: f64) -> Self {
        self.target_temperature = temp;
        self
    }

    /// Set target humidity
    pub fn with_humidity(mut self, humidity: f64) -> Self {
        self.target_humidity = humidity;
        self
    }
}

#[async_trait]
impl ClimateAdapter for VerticalFarmingSimulator {
    fn name(&self) -> &str {
        "VerticalFarmingSimulator"
    }

    fn is_connected(&self) -> bool {
        true
    }

    async fn read(&self) -> Result<ClimateMessage> {
        let seq = self.sequence.fetch_add(1, Ordering::SeqCst);
        let days = (seq as u32) % 30 + 1;

        let data = VerticalFarmingData {
            system_type: FarmingSystemType::Hydroponics,
            environment: FarmingEnvironment {
                temperature_celsius: self.target_temperature + (rand_simple() - 0.5) * 2.0,
                humidity_percentage: self.target_humidity + (rand_simple() - 0.5) * 5.0,
                co2_ppm: Some(800.0 + rand_simple() * 100.0),
                vpd_kpa: Some(0.8 + rand_simple() * 0.2),
                air_flow_m3_per_hour: Some(500.0 + rand_simple() * 100.0),
            },
            lighting: LightingData {
                light_type: Some(LightSourceType::Led),
                ppfd_umol_per_m2_s: 450.0 + rand_simple() * 50.0,
                dli_mol_per_m2_day: Some(25.0 + rand_simple() * 5.0),
                spectrum: Some(LightSpectrum::FullSpectrum),
                photoperiod_hours: 16.0,
                energy_consumption_kwh: Some(50.0 + rand_simple() * 10.0),
            },
            nutrient_solution: NutrientSolution {
                ph: 6.0 + (rand_simple() - 0.5) * 0.2,
                ec_ms_per_cm: 1.8 + (rand_simple() - 0.5) * 0.2,
                temperature_celsius: Some(20.0 + rand_simple() * 2.0),
                dissolved_oxygen_mg_per_l: Some(8.0 + rand_simple()),
                elements: Some(NutrientElements {
                    nitrogen_ppm: Some(150.0 + rand_simple() * 20.0),
                    phosphorus_ppm: Some(50.0 + rand_simple() * 10.0),
                    potassium_ppm: Some(200.0 + rand_simple() * 20.0),
                    calcium_ppm: Some(180.0 + rand_simple() * 20.0),
                    magnesium_ppm: Some(50.0 + rand_simple() * 10.0),
                    ..Default::default()
                }),
            },
            crop: Some(CropData {
                species: Some("lactuca_sativa".to_string()),
                variety: Some("butterhead".to_string()),
                growth_stage: Some(match days {
                    1..=7 => GrowthStage::Germination,
                    8..=14 => GrowthStage::Seedling,
                    15..=25 => GrowthStage::Vegetative,
                    _ => GrowthStage::Harvest,
                }),
                days_after_planting: Some(days),
                plant_count: Some(500),
                density_plants_per_m2: Some(25.0),
            }),
            yield_data: if days >= 25 {
                Some(YieldData {
                    fresh_weight_kg: Some(40.0 + rand_simple() * 10.0),
                    dry_weight_kg: Some(2.0 + rand_simple() * 0.5),
                    area_m2: Some(20.0),
                    kg_per_m2: Some(2.0 + rand_simple() * 0.5),
                    harvest_cycle_days: Some(28),
                })
            } else {
                None
            },
        };

        ClimateMessage::builder()
            .location(self.location.clone())
            .device(self.device.clone())
            .vertical_farming_data(data)
            .meta(
                Metadata::with_quality(0.92)
                    .source(DataSource::Sensor)
                    .processing_level(ProcessingLevel::Calibrated),
            )
            .build()
    }

    async fn start_stream(&mut self) -> Result<()> {
        self.is_streaming.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn stop_stream(&mut self) -> Result<()> {
        self.is_streaming.store(false, Ordering::SeqCst);
        Ok(())
    }
}

/// Simulated ocean cleanup data adapter
pub struct OceanCleanupSimulator {
    location: Location,
    device: Device,
    sequence: AtomicU64,
    is_streaming: AtomicBool,
    /// Zone for simulation
    pub zone: OceanZone,
}

impl OceanCleanupSimulator {
    /// Create a new ocean cleanup simulator
    pub fn new(location: Location, device: Device) -> Self {
        Self {
            location,
            device,
            sequence: AtomicU64::new(0),
            is_streaming: AtomicBool::new(false),
            zone: OceanZone::GreatPacificGarbagePatch,
        }
    }

    /// Set the ocean zone
    pub fn with_zone(mut self, zone: OceanZone) -> Self {
        self.zone = zone;
        self
    }
}

#[async_trait]
impl ClimateAdapter for OceanCleanupSimulator {
    fn name(&self) -> &str {
        "OceanCleanupSimulator"
    }

    fn is_connected(&self) -> bool {
        true
    }

    async fn read(&self) -> Result<ClimateMessage> {
        let _seq = self.sequence.fetch_add(1, Ordering::SeqCst);

        let total_mass = 200.0 + rand_simple() * 100.0;
        let plastic_ratio = 0.6 + rand_simple() * 0.2;

        let data = OceanCleanupData {
            operation_type: CleanupOperationType::FloatingBarrier,
            collection: CollectionData {
                total_mass_kg: total_mass,
                plastic_mass_kg: Some(total_mass * plastic_ratio),
                fishing_gear_kg: Some(total_mass * 0.15),
                other_debris_kg: Some(total_mass * (1.0 - plastic_ratio - 0.15)),
                microplastic_count: Some((rand_simple() * 200000.0) as u64),
                microplastic_size_range_mm: Some([0.1, 5.0]),
                debris_categories: Some(DebrisCategories {
                    bottles_count: Some((rand_simple() * 500.0) as u32),
                    bags_count: Some((rand_simple() * 300.0) as u32),
                    foam_kg: Some(rand_simple() * 20.0),
                    rope_m: Some(rand_simple() * 100.0),
                }),
            },
            area: OperationArea {
                swept_km2: 5.0 + rand_simple() * 10.0,
                duration_hours: 6.0 + rand_simple() * 4.0,
                efficiency_kg_per_km2: Some(total_mass / (5.0 + rand_simple() * 10.0)),
                efficiency_kg_per_hour: Some(total_mass / (6.0 + rand_simple() * 4.0)),
            },
            water_conditions: Some(WaterConditions {
                temperature_celsius: Some(18.0 + rand_simple() * 5.0),
                salinity_ppt: Some(35.0 + rand_simple() * 2.0),
                ph: Some(8.1 + (rand_simple() - 0.5) * 0.2),
                current_speed_m_per_s: Some(0.3 + rand_simple() * 0.4),
                current_direction_deg: Some(rand_simple() * 360.0),
                wave_height_m: Some(0.5 + rand_simple() * 2.0),
                visibility_m: Some(5.0 + rand_simple() * 10.0),
            }),
            vessel: Some(VesselInfo {
                vessel_type: Some(VesselType::SupportVessel),
                name: Some("Ocean Warrior".to_string()),
                fuel_consumption_l_per_hour: Some(40.0 + rand_simple() * 20.0),
                crew_count: Some(8),
            }),
            zone: Some(self.zone),
        };

        ClimateMessage::builder()
            .location(self.location.clone())
            .device(self.device.clone())
            .ocean_cleanup_data(data)
            .meta(
                Metadata::with_quality(0.88)
                    .source(DataSource::Sensor)
                    .processing_level(ProcessingLevel::Validated),
            )
            .build()
    }

    async fn start_stream(&mut self) -> Result<()> {
        self.is_streaming.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn stop_stream(&mut self) -> Result<()> {
        self.is_streaming.store(false, Ordering::SeqCst);
        Ok(())
    }
}

/// Simulated climate model data adapter
pub struct ClimateModelSimulator {
    location: Location,
    device: Device,
    sequence: AtomicU64,
    is_streaming: AtomicBool,
    /// SSP scenario
    pub scenario: SspScenario,
    /// Variable name
    pub variable: String,
}

impl ClimateModelSimulator {
    /// Create a new climate model simulator
    pub fn new(location: Location, device: Device) -> Self {
        Self {
            location,
            device,
            sequence: AtomicU64::new(0),
            is_streaming: AtomicBool::new(false),
            scenario: SspScenario::Ssp245,
            variable: "tas".to_string(),
        }
    }

    /// Set the SSP scenario
    pub fn with_scenario(mut self, scenario: SspScenario) -> Self {
        self.scenario = scenario;
        self
    }

    /// Set the variable name
    pub fn with_variable(mut self, variable: impl Into<String>) -> Self {
        self.variable = variable.into();
        self
    }
}

#[async_trait]
impl ClimateAdapter for ClimateModelSimulator {
    fn name(&self) -> &str {
        "ClimateModelSimulator"
    }

    fn is_connected(&self) -> bool {
        true
    }

    async fn read(&self) -> Result<ClimateMessage> {
        let seq = self.sequence.fetch_add(1, Ordering::SeqCst);
        let year = 2020 + (seq as u32) % 80;

        // Simulate temperature increase based on scenario
        let base_temp = 288.0; // ~15°C in Kelvin
        let warming = match self.scenario {
            SspScenario::Ssp119 => 0.5 + (year as f64 - 2020.0) * 0.005,
            SspScenario::Ssp126 => 0.5 + (year as f64 - 2020.0) * 0.01,
            SspScenario::Ssp245 => 0.5 + (year as f64 - 2020.0) * 0.02,
            SspScenario::Ssp370 => 0.5 + (year as f64 - 2020.0) * 0.03,
            SspScenario::Ssp585 => 0.5 + (year as f64 - 2020.0) * 0.04,
        };

        let data = ClimateModelData {
            model: ModelInfo {
                source_id: "CESM2".to_string(),
                institution_id: Some("NCAR".to_string()),
                experiment_id: format!("{:?}", self.scenario).to_lowercase(),
                variant_label: "r1i1p1f1".to_string(),
                grid_label: Some("gn".to_string()),
            },
            variable: VariableInfo {
                name: self.variable.clone(),
                long_name: Some("Near-Surface Air Temperature".to_string()),
                standard_name: Some("air_temperature".to_string()),
                units: "K".to_string(),
                cell_methods: Some("time: mean".to_string()),
            },
            grid: Some(GridInfo {
                resolution_deg: Some(1.0),
                resolution_km: Some(100.0),
                nlat: Some(180),
                nlon: Some(360),
                grid_type: Some(GridType::RegularLatLon),
            }),
            time: TimeInfo {
                start_date: Some(format!("{}-01-01", year)),
                end_date: Some(format!("{}-12-31", year)),
                frequency: TimeFrequency::Mon,
                calendar: Some(CalendarType::Gregorian),
            },
            value: ModelValue {
                data: base_temp + warming + (rand_simple() - 0.5) * 1.0,
                anomaly: Some(warming),
                climatology: Some(base_temp),
                percentile_5: Some(base_temp + warming - 2.0),
                percentile_50: Some(base_temp + warming),
                percentile_95: Some(base_temp + warming + 2.0),
                standard_deviation: Some(0.8 + rand_simple() * 0.4),
            },
            scenario: Some(ScenarioInfo {
                ssp: Some(self.scenario),
                forcing_level_w_per_m2: Some(match self.scenario {
                    SspScenario::Ssp119 => 1.9,
                    SspScenario::Ssp126 => 2.6,
                    SspScenario::Ssp245 => 4.5,
                    SspScenario::Ssp370 => 7.0,
                    SspScenario::Ssp585 => 8.5,
                }),
                description: Some(match self.scenario {
                    SspScenario::Ssp119 => "Very low emissions".to_string(),
                    SspScenario::Ssp126 => "Low emissions".to_string(),
                    SspScenario::Ssp245 => "Middle of the road".to_string(),
                    SspScenario::Ssp370 => "Regional rivalry".to_string(),
                    SspScenario::Ssp585 => "Fossil-fueled development".to_string(),
                }),
            }),
            reference: Some(ReferenceInfo {
                further_info_url: Some(
                    "https://furtherinfo.es-doc.org/CMIP6.NCAR.CESM2".to_string(),
                ),
                doi: Some("10.22033/ESGF/CMIP6.xxxx".to_string()),
                citation: None,
            }),
        };

        ClimateMessage::builder()
            .location(self.location.clone())
            .device(self.device.clone())
            .climate_model_data(data)
            .meta(
                Metadata::with_quality(0.99)
                    .source(DataSource::Model)
                    .processing_level(ProcessingLevel::Validated),
            )
            .build()
    }

    async fn start_stream(&mut self) -> Result<()> {
        self.is_streaming.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn stop_stream(&mut self) -> Result<()> {
        self.is_streaming.store(false, Ordering::SeqCst);
        Ok(())
    }
}

/// Simple pseudo-random number generator (0.0 to 1.0)
/// Uses a basic approach without external dependencies
fn rand_simple() -> f64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(0);
    ((nanos as f64) / 1_000_000_000.0).fract().abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_carbon_capture_simulator() {
        let simulator = CarbonCaptureSimulator::new(
            Location::new(64.0, -21.0),
            Device::new("Test", "Simulator"),
        );

        assert!(simulator.is_connected());
        assert_eq!(simulator.name(), "CarbonCaptureSimulator");

        let message = simulator.read().await.unwrap();
        assert_eq!(message.data_type, DataType::CarbonCapture);
    }

    #[tokio::test]
    async fn test_vertical_farming_simulator() {
        let simulator = VerticalFarmingSimulator::new(
            Location::new(37.5, 127.0),
            Device::new("Test", "Simulator"),
        );

        let message = simulator.read().await.unwrap();
        assert_eq!(message.data_type, DataType::VerticalFarming);
    }

    #[tokio::test]
    async fn test_ocean_cleanup_simulator() {
        let simulator = OceanCleanupSimulator::new(
            Location::new(35.0, -145.0),
            Device::new("Test", "Simulator"),
        )
        .with_zone(OceanZone::GreatPacificGarbagePatch);

        let message = simulator.read().await.unwrap();
        assert_eq!(message.data_type, DataType::OceanCleanup);
    }

    #[tokio::test]
    async fn test_climate_model_simulator() {
        let simulator = ClimateModelSimulator::new(
            Location::new(35.5, 127.0),
            Device::new("NCAR", "CESM2"),
        )
        .with_scenario(SspScenario::Ssp245);

        let message = simulator.read().await.unwrap();
        assert_eq!(message.data_type, DataType::ClimateModel);
    }
}
