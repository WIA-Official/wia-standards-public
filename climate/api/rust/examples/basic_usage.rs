//! Basic usage example for WIA Climate library
//!
//! This example demonstrates how to create, serialize, and validate
//! climate data messages using the WIA Climate Standard.

use wia_climate::prelude::*;

fn main() -> Result<()> {
    println!("=== WIA Climate Standard - Basic Usage Example ===\n");

    // 1. Create a Carbon Capture message
    println!("1. Creating Carbon Capture message...");
    let carbon_message = ClimateMessage::builder()
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
            ..Default::default()
        })
        .meta(Metadata::with_quality(0.98).source(DataSource::Sensor))
        .build()?;

    println!("   Type: {:?}", carbon_message.data_type);
    println!("   Location: ({}, {})",
        carbon_message.location.latitude,
        carbon_message.location.longitude
    );
    println!();

    // 2. Serialize to JSON
    println!("2. Serializing to JSON...");
    let json = carbon_message.to_json_pretty()?;
    println!("{}\n", json);

    // 3. Deserialize from JSON
    println!("3. Deserializing from JSON...");
    let parsed = ClimateMessage::from_json(&json)?;
    println!("   Parsed type: {:?}", parsed.data_type);
    println!("   Version: {}", parsed.version);
    println!();

    // 4. Create a Vertical Farming message
    println!("4. Creating Vertical Farming message...");
    let farming_message = ClimateMessage::builder()
        .location(Location::new(37.5665, 126.978))
        .device(Device::new("SmartFarm", "VerticalPro 5000"))
        .vertical_farming_data(VerticalFarmingData {
            system_type: FarmingSystemType::Hydroponics,
            environment: FarmingEnvironment {
                temperature_celsius: 22.5,
                humidity_percentage: 65.0,
                co2_ppm: Some(800.0),
                vpd_kpa: Some(0.85),
                ..Default::default()
            },
            lighting: LightingData {
                light_type: Some(LightSourceType::Led),
                ppfd_umol_per_m2_s: 450.0,
                photoperiod_hours: 16.0,
                spectrum: Some(LightSpectrum::FullSpectrum),
                ..Default::default()
            },
            nutrient_solution: NutrientSolution {
                ph: 6.0,
                ec_ms_per_cm: 1.8,
                temperature_celsius: Some(20.0),
                ..Default::default()
            },
            crop: Some(CropData {
                species: Some("lactuca_sativa".to_string()),
                variety: Some("butterhead".to_string()),
                growth_stage: Some(GrowthStage::Vegetative),
                days_after_planting: Some(21),
                ..Default::default()
            }),
            ..Default::default()
        })
        .build()?;

    println!("   Type: {:?}", farming_message.data_type);
    println!("   System: {:?}",
        if let ClimateData::VerticalFarming(ref data) = farming_message.data {
            format!("{:?}", data.system_type)
        } else {
            "N/A".to_string()
        }
    );
    println!();

    // 5. Create an Ocean Cleanup message
    println!("5. Creating Ocean Cleanup message...");
    let cleanup_message = ClimateMessage::builder()
        .location(Location::new(35.0, -145.0))
        .device(Device::new("The Ocean Cleanup", "System 03"))
        .ocean_cleanup_data(OceanCleanupData {
            operation_type: CleanupOperationType::FloatingBarrier,
            collection: CollectionData {
                total_mass_kg: 250.5,
                plastic_mass_kg: Some(180.2),
                fishing_gear_kg: Some(45.3),
                ..Default::default()
            },
            area: OperationArea {
                swept_km2: 10.5,
                duration_hours: 8.0,
                efficiency_kg_per_km2: Some(23.86),
                ..Default::default()
            },
            zone: Some(OceanZone::GreatPacificGarbagePatch),
            ..Default::default()
        })
        .build()?;

    println!("   Type: {:?}", cleanup_message.data_type);
    println!("   Zone: {:?}",
        if let ClimateData::OceanCleanup(ref data) = cleanup_message.data {
            format!("{:?}", data.zone)
        } else {
            "N/A".to_string()
        }
    );
    println!();

    // 6. Validation example
    println!("6. Validation example (invalid coordinates)...");
    let invalid_result = ClimateMessage::builder()
        .location(Location::new(100.0, 200.0)) // Invalid!
        .device(Device::new("Test", "Device"))
        .carbon_capture_data(CarbonCaptureData::default())
        .build();

    match invalid_result {
        Ok(_) => println!("   Unexpectedly succeeded"),
        Err(e) => println!("   Validation error: {}", e),
    }
    println!();

    println!("=== Example Complete ===");
    println!("\n弘益人間 - Benefit All Humanity 🌍");

    Ok(())
}
