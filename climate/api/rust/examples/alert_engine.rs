//! Alert Engine Example
//!
//! This example demonstrates how to set up alert rules that monitor
//! climate data for threshold violations.
//!
//! Run with: cargo run --example alert_engine

use std::time::Duration;
use wia_climate::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA Climate - Alert Engine Example ===\n");

    // Create alert engine
    let mut engine = AlertEngine::new();

    // Add alert rules
    println!("--- Configuring Alert Rules ---\n");

    // Rule 1: Low carbon capture rate
    let low_capture_rule = AlertRule::builder("capture-rate-low", "Low Carbon Capture Rate")
        .description("Alert when capture rate falls below threshold")
        .data_type(DataType::CarbonCapture)
        .condition("CarbonCapture.capture_rate_kg_per_hour", ComparisonOperator::LessThan, 100.0)
        .duration_secs(0) // Immediate alert for demo
        .severity(AlertSeverity::Warning)
        .notify("webhook-alerts")
        .build();

    println!("  Added: {} ({:?})", low_capture_rule.name, low_capture_rule.severity);
    engine.add_rule(low_capture_rule);

    // Rule 2: High temperature in vertical farm
    let high_temp_rule = AlertRule::builder("farm-temp-high", "High Temperature Alert")
        .description("Alert when farm temperature exceeds safe range")
        .data_type(DataType::VerticalFarming)
        .condition("VerticalFarming.environmental_conditions.temperature_celsius", ComparisonOperator::GreaterThan, 30.0)
        .duration_secs(0)
        .severity(AlertSeverity::Critical)
        .notify("webhook-alerts")
        .notify("pagerduty")
        .build();

    println!("  Added: {} ({:?})", high_temp_rule.name, high_temp_rule.severity);
    engine.add_rule(high_temp_rule);

    // Rule 3: Large debris collection (info)
    let debris_rule = AlertRule::builder("large-collection", "Large Debris Collection")
        .description("Notify when collection exceeds threshold")
        .data_type(DataType::OceanCleanup)
        .condition("OceanCleanup.debris_collected_kg", ComparisonOperator::GreaterThan, 1000.0)
        .duration_secs(0)
        .severity(AlertSeverity::Info)
        .notify("slack-notifications")
        .build();

    println!("  Added: {} ({:?})", debris_rule.name, debris_rule.severity);
    engine.add_rule(debris_rule);

    println!("\nTotal rules: {}\n", engine.rules().len());

    // Create test messages
    println!("--- Processing Messages ---\n");

    let messages = vec![
        // Normal carbon capture (should not trigger)
        ("Normal capture rate", ClimateMessage::builder()
            .location(Location::new(64.0, -21.0))
            .device(Device::new("Climeworks", "Orca DAC"))
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 125.5,
                ..Default::default()
            })
            .build()?),

        // Low carbon capture (should trigger warning)
        ("Low capture rate", ClimateMessage::builder()
            .location(Location::new(64.0, -21.0))
            .device(Device::new("Climeworks", "Orca DAC"))
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 85.0, // Below threshold
                ..Default::default()
            })
            .build()?),

        // High temperature farm (should trigger critical)
        ("High temperature farm", ClimateMessage::builder()
            .location(Location::new(1.35, 103.82))
            .device(Device::new("SkyGreens", "Tower-A"))
            .vertical_farming_data(VerticalFarmingData {
                facility_type: FacilityType::VerticalTower,
                crop_type: "Lettuce".to_string(),
                growing_area_sqm: 100.0,
                environmental_conditions: Some(EnvironmentalConditions {
                    temperature_celsius: 35.0, // Above threshold
                    humidity_percentage: 65.0,
                    ..Default::default()
                }),
                ..Default::default()
            })
            .build()?),

        // Large debris collection (should trigger info)
        ("Large debris collection", ClimateMessage::builder()
            .location(Location::new(35.0, -140.0))
            .device(Device::new("Ocean Cleanup", "System 002"))
            .ocean_cleanup_data(OceanCleanupData {
                operation_type: OceanOperationType::PlasticCollection,
                debris_collected_kg: Some(2500.0), // Above threshold
                ..Default::default()
            })
            .build()?),
    ];

    for (description, message) in messages {
        println!("Message: {}", description);
        println!("  Type: {}", message.data_type);

        let events = engine.evaluate(&message);

        if events.is_empty() {
            println!("  Result: No alerts triggered");
        } else {
            for event in &events {
                println!("  🚨 ALERT: {} [{:?}]",
                    event.rule_name,
                    event.severity
                );
                println!("     Condition: {} {} {}",
                    event.condition.field,
                    event.condition.operator,
                    event.condition.threshold
                );
                println!("     Current value: {}", event.current_value);
            }
        }
        println!();
    }

    // Show firing alerts summary
    println!("--- Currently Firing Alerts ---\n");
    let firing = engine.firing_alerts();
    if firing.is_empty() {
        println!("No alerts currently firing.");
    } else {
        for state in firing {
            let rule = engine.get_rule(&state.rule_id).unwrap();
            println!("  {} ({:?})", rule.name, rule.severity);
            println!("    Last value: {:?}", state.last_value);
            println!("    Firing since: {:?}", state.firing_since);
        }
    }

    println!("\n弘益人間 - Benefit All Humanity 🌍");
    Ok(())
}
