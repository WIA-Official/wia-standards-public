//! Integration tests for WIA Climate library

use wia_climate::prelude::*;

#[test]
fn test_carbon_capture_message_creation() {
    let message = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0).with_altitude(100.0))
        .device(
            Device::new("Climeworks", "Orca DAC")
                .with_serial("ORCA-2024-001")
                .with_firmware("2.1.0"),
        )
        .carbon_capture_data(CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 125.5,
            co2_concentration_ppm: Some(415.0),
            co2_purity_percentage: Some(99.2),
            energy_consumption_kwh: Some(2500.0),
            sorbent_status: Some(SorbentStatus {
                sorbent_type: Some(SorbentType::SolidAmine),
                efficiency_percentage: Some(85.5),
                cycles_completed: Some(1250),
                temperature_celsius: Some(90.0),
            }),
            storage: Some(StorageData {
                method: Some(StorageMethod::Geological),
                pressure_mpa: Some(10.5),
                depth_m: Some(2000.0),
                formation_type: Some(FormationType::Basalt),
                injection_rate_kg_per_hour: Some(120.0),
            }),
            cumulative_captured_tonnes: Some(5000.0),
        })
        .meta(
            Metadata::with_quality(0.98)
                .source(DataSource::Sensor)
                .processing_level(ProcessingLevel::Validated),
        )
        .build()
        .expect("Failed to build message");

    assert_eq!(message.data_type, DataType::CarbonCapture);
    assert_eq!(message.version, "1.0.0");
    assert_eq!(message.location.latitude, 64.0);
    assert_eq!(message.device.manufacturer, "Climeworks");
}

#[test]
fn test_weather_control_message_creation() {
    let message = ClimateMessage::builder()
        .location(Location::new(35.0, 127.0))
        .device(Device::new("WMI", "Cloud Seeder X1"))
        .weather_control_data(WeatherControlData {
            operation_type: WeatherOperationType::CloudSeeding,
            seeding_agent: Some(SeedingAgent {
                agent_type: Some(SeedingAgentType::SilverIodide),
                mass_grams: Some(50.0),
                concentration_g_per_m3: Some(0.01),
            }),
            delivery_method: Some(DeliveryMethod::Aircraft),
            target_cloud: Some(TargetCloud {
                cloud_type: Some(CloudType::Cumulus),
                base_altitude_m: Some(2000.0),
                top_altitude_m: Some(5000.0),
                coverage_km2: Some(100.0),
            }),
            atmospheric_conditions: AtmosphericConditions {
                temperature_celsius: -5.0,
                humidity_percentage: 85.0,
                wind_speed_m_per_s: Some(15.0),
                wind_direction_deg: Some(270.0),
                pressure_hpa: Some(1013.25),
            },
            result: Some(WeatherResult {
                precipitation_mm: Some(12.5),
                duration_hours: Some(3.0),
                effectiveness_percentage: Some(15.0),
                baseline_precipitation_mm: Some(8.0),
            }),
        })
        .build()
        .expect("Failed to build message");

    assert_eq!(message.data_type, DataType::WeatherControl);
}

#[test]
fn test_vertical_farming_message_creation() {
    let message = ClimateMessage::builder()
        .location(Location::new(37.5665, 126.978))
        .device(Device::new("SmartFarm", "VerticalPro 5000"))
        .vertical_farming_data(VerticalFarmingData {
            system_type: FarmingSystemType::Hydroponics,
            environment: FarmingEnvironment {
                temperature_celsius: 22.5,
                humidity_percentage: 65.0,
                co2_ppm: Some(800.0),
                vpd_kpa: Some(0.85),
                air_flow_m3_per_hour: Some(500.0),
            },
            lighting: LightingData {
                light_type: Some(LightSourceType::Led),
                ppfd_umol_per_m2_s: 450.0,
                dli_mol_per_m2_day: Some(25.9),
                spectrum: Some(LightSpectrum::FullSpectrum),
                photoperiod_hours: 16.0,
                energy_consumption_kwh: Some(50.0),
            },
            nutrient_solution: NutrientSolution {
                ph: 6.0,
                ec_ms_per_cm: 1.8,
                temperature_celsius: Some(20.0),
                dissolved_oxygen_mg_per_l: Some(8.0),
                elements: Some(NutrientElements {
                    nitrogen_ppm: Some(150.0),
                    phosphorus_ppm: Some(50.0),
                    potassium_ppm: Some(200.0),
                    calcium_ppm: Some(180.0),
                    magnesium_ppm: Some(50.0),
                    ..Default::default()
                }),
            },
            crop: Some(CropData {
                species: Some("lactuca_sativa".to_string()),
                variety: Some("butterhead".to_string()),
                growth_stage: Some(GrowthStage::Vegetative),
                days_after_planting: Some(21),
                plant_count: Some(500),
                density_plants_per_m2: Some(25.0),
            }),
            yield_data: Some(YieldData {
                fresh_weight_kg: Some(42.5),
                dry_weight_kg: Some(2.1),
                area_m2: Some(20.0),
                kg_per_m2: Some(2.125),
                harvest_cycle_days: Some(28),
            }),
        })
        .build()
        .expect("Failed to build message");

    assert_eq!(message.data_type, DataType::VerticalFarming);
}

#[test]
fn test_ocean_cleanup_message_creation() {
    let message = ClimateMessage::builder()
        .location(Location::new(35.0, -145.0))
        .device(Device::new("The Ocean Cleanup", "System 03"))
        .ocean_cleanup_data(OceanCleanupData {
            operation_type: CleanupOperationType::FloatingBarrier,
            collection: CollectionData {
                total_mass_kg: 250.5,
                plastic_mass_kg: Some(180.2),
                fishing_gear_kg: Some(45.3),
                other_debris_kg: Some(25.0),
                microplastic_count: Some(150000),
                microplastic_size_range_mm: Some([0.1, 5.0]),
                debris_categories: Some(DebrisCategories {
                    bottles_count: Some(350),
                    bags_count: Some(200),
                    foam_kg: Some(15.0),
                    rope_m: Some(50.0),
                }),
            },
            area: OperationArea {
                swept_km2: 10.5,
                duration_hours: 8.0,
                efficiency_kg_per_km2: Some(23.86),
                efficiency_kg_per_hour: Some(31.31),
            },
            water_conditions: Some(WaterConditions {
                temperature_celsius: Some(18.5),
                salinity_ppt: Some(35.0),
                ph: Some(8.1),
                current_speed_m_per_s: Some(0.5),
                current_direction_deg: Some(180.0),
                wave_height_m: Some(1.2),
                visibility_m: Some(10.0),
            }),
            vessel: Some(VesselInfo {
                vessel_type: Some(VesselType::SupportVessel),
                name: Some("Ocean Warrior".to_string()),
                fuel_consumption_l_per_hour: Some(50.0),
                crew_count: Some(8),
            }),
            zone: Some(OceanZone::GreatPacificGarbagePatch),
        })
        .build()
        .expect("Failed to build message");

    assert_eq!(message.data_type, DataType::OceanCleanup);
}

#[test]
fn test_climate_model_message_creation() {
    let message = ClimateMessage::builder()
        .location(Location::new(35.5, 127.0))
        .device(Device::new("NCAR", "CESM2"))
        .climate_model_data(ClimateModelData {
            model: ModelInfo {
                source_id: "CESM2".to_string(),
                institution_id: Some("NCAR".to_string()),
                experiment_id: "ssp245".to_string(),
                variant_label: "r1i1p1f1".to_string(),
                grid_label: Some("gn".to_string()),
            },
            variable: VariableInfo {
                name: "tas".to_string(),
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
                start_date: Some("2050-01-01".to_string()),
                end_date: Some("2050-12-31".to_string()),
                frequency: TimeFrequency::Mon,
                calendar: Some(CalendarType::Gregorian),
            },
            value: ModelValue {
                data: 289.5,
                anomaly: Some(2.2),
                climatology: Some(287.3),
                percentile_5: Some(285.0),
                percentile_50: Some(289.5),
                percentile_95: Some(294.0),
                standard_deviation: Some(1.2),
            },
            scenario: Some(ScenarioInfo {
                ssp: Some(SspScenario::Ssp245),
                forcing_level_w_per_m2: Some(4.5),
                description: Some("Middle of the road".to_string()),
            }),
            reference: Some(ReferenceInfo {
                further_info_url: Some(
                    "https://furtherinfo.es-doc.org/CMIP6.NCAR.CESM2".to_string(),
                ),
                doi: Some("10.22033/ESGF/CMIP6.xxxx".to_string()),
                citation: None,
            }),
        })
        .build()
        .expect("Failed to build message");

    assert_eq!(message.data_type, DataType::ClimateModel);
}

#[test]
fn test_json_serialization_roundtrip() {
    let original = ClimateMessage::builder()
        .location(Location::new(64.0, -21.0))
        .device(Device::new("Test", "Device"))
        .carbon_capture_data(CarbonCaptureData {
            technology: CarbonCaptureTechnology::Dac,
            capture_rate_kg_per_hour: 100.0,
            ..Default::default()
        })
        .build()
        .expect("Failed to build message");

    // Serialize to JSON
    let json = original.to_json().expect("Failed to serialize");

    // Verify JSON contains expected fields
    assert!(json.contains("carbon_capture"));
    assert!(json.contains("1.0.0"));
    assert!(json.contains("64"));

    // Deserialize back
    let parsed = ClimateMessage::from_json(&json).expect("Failed to deserialize");

    assert_eq!(parsed.data_type, original.data_type);
    assert_eq!(parsed.location.latitude, original.location.latitude);
}

#[test]
fn test_validation_rejects_invalid_coordinates() {
    let result = ClimateMessage::builder()
        .location(Location::new(100.0, 200.0)) // Invalid coordinates
        .device(Device::new("Test", "Device"))
        .carbon_capture_data(CarbonCaptureData::default())
        .build();

    assert!(result.is_err());
    let err = result.unwrap_err();
    assert!(err.to_string().contains("Invalid coordinates"));
}

#[test]
fn test_validation_rejects_invalid_humidity() {
    let result = ClimateMessage::builder()
        .location(Location::new(35.0, 127.0))
        .device(Device::new("Test", "Device"))
        .weather_control_data(WeatherControlData {
            operation_type: WeatherOperationType::CloudSeeding,
            atmospheric_conditions: AtmosphericConditions {
                temperature_celsius: 20.0,
                humidity_percentage: 150.0, // Invalid: > 100%
                ..Default::default()
            },
            ..Default::default()
        })
        .build();

    assert!(result.is_err());
}

#[test]
fn test_validation_rejects_invalid_ph() {
    let result = ClimateMessage::builder()
        .location(Location::new(35.0, 127.0))
        .device(Device::new("Test", "Device"))
        .vertical_farming_data(VerticalFarmingData {
            system_type: FarmingSystemType::Hydroponics,
            environment: FarmingEnvironment {
                temperature_celsius: 22.0,
                humidity_percentage: 65.0,
                ..Default::default()
            },
            lighting: LightingData {
                ppfd_umol_per_m2_s: 400.0,
                photoperiod_hours: 16.0,
                ..Default::default()
            },
            nutrient_solution: NutrientSolution {
                ph: 15.0, // Invalid: > 14
                ec_ms_per_cm: 1.8,
                ..Default::default()
            },
            ..Default::default()
        })
        .build();

    assert!(result.is_err());
}

#[tokio::test]
async fn test_simulator_adapters() {
    // Test CarbonCaptureSimulator
    let cc_sim = CarbonCaptureSimulator::new(
        Location::new(64.0, -21.0),
        Device::new("Climeworks", "Orca DAC Simulator"),
    )
    .with_capture_rate(150.0)
    .with_variation(0.2);

    let cc_message = cc_sim.read().await.expect("Failed to read from simulator");
    assert_eq!(cc_message.data_type, DataType::CarbonCapture);

    // Test VerticalFarmingSimulator
    let vf_sim = VerticalFarmingSimulator::new(
        Location::new(37.5, 127.0),
        Device::new("SmartFarm", "Simulator"),
    )
    .with_temperature(24.0)
    .with_humidity(70.0);

    let vf_message = vf_sim.read().await.expect("Failed to read from simulator");
    assert_eq!(vf_message.data_type, DataType::VerticalFarming);

    // Test OceanCleanupSimulator
    let oc_sim = OceanCleanupSimulator::new(
        Location::new(35.0, -145.0),
        Device::new("Ocean Cleanup", "Simulator"),
    )
    .with_zone(OceanZone::GreatPacificGarbagePatch);

    let oc_message = oc_sim.read().await.expect("Failed to read from simulator");
    assert_eq!(oc_message.data_type, DataType::OceanCleanup);

    // Test ClimateModelSimulator
    let cm_sim = ClimateModelSimulator::new(
        Location::new(35.5, 127.0),
        Device::new("NCAR", "CESM2 Simulator"),
    )
    .with_scenario(SspScenario::Ssp245)
    .with_variable("tas");

    let cm_message = cm_sim.read().await.expect("Failed to read from simulator");
    assert_eq!(cm_message.data_type, DataType::ClimateModel);
}
