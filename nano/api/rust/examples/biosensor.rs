//! Example: Nanosensor for Glucose Monitoring
//!
//! This example demonstrates using nanosensors for continuous
//! glucose monitoring in a simulated physiological environment.

use wia_nano::prelude::*;
use wia_nano::systems::StandardNanosensor;

#[tokio::main]
async fn main() -> NanoResult<()> {
    println!("=== WIA Nano SDK: Biosensor Example ===\n");

    // Create a chemical sensor for glucose detection
    let mut sensor = StandardNanosensor::builder("glucose-sensor-001", SensorType::Chemical)
        .with_environment(Environment::physiological())
        .with_performance(SensorPerformance {
            detection_limit: 1e-6, // 1 μM
            dynamic_range_min: 1e-6,
            dynamic_range_max: 30e-3, // 30 mM (diabetic range)
            sensitivity: Some(0.95),
            specificity: Some(0.98),
            response_time_ms: Some(50.0),
        })
        .build();

    // Initialize
    sensor.initialize().await?;
    println!("✓ Glucose sensor initialized");
    println!("  Detection limit: 1 μM");
    println!("  Dynamic range: 1 μM - 30 mM");
    println!("  Specificity: 98%\n");

    // Set detection threshold for hypoglycemia
    sensor.set_threshold(DetectionThreshold {
        lower_bound: Some(3.9e-3), // 3.9 mM (70 mg/dL)
        upper_bound: Some(10.0e-3), // 10 mM (180 mg/dL)
        unit: "M".to_string(),
        hysteresis: Some(0.5e-3),
        trigger_mode: TriggerMode::OutOfRange,
    });
    println!("✓ Alert threshold set:");
    println!("  Low: < 3.9 mM (hypoglycemia)");
    println!("  High: > 10.0 mM (hyperglycemia)\n");

    // Calibrate sensor
    let calibration = sensor.calibrate(CalibrationReference {
        reference_type: ReferenceType::Standard,
        known_value: 5.5e-3, // 5.5 mM standard
        unit: "M".to_string(),
        tolerance: Some(0.1e-3),
    }).await?;
    println!("✓ Calibration complete:");
    println!("  R² = {:.4}", calibration.r_squared.unwrap_or(0.0));

    // Start monitoring
    println!("\nStarting continuous monitoring...\n");
    sensor.start_monitoring(1000).await?; // 1 Hz

    // Simulate readings over time
    for i in 1..=10 {
        // Take reading
        let reading = sensor.read().await?;

        // Convert to mg/dL for display
        let mg_dl = reading.value * 18.0 * 1000.0; // M to mg/dL

        // Check for alerts
        let alert = if sensor.is_target_detected() {
            if mg_dl < 70.0 {
                "⚠️  HYPOGLYCEMIA"
            } else {
                "⚠️  HYPERGLYCEMIA"
            }
        } else {
            "✓ Normal"
        };

        println!("  Reading {}: {:.1} mg/dL ({:.2} mM) - {}",
                 i, mg_dl, reading.value * 1000.0, alert);

        // Small delay between readings
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }

    // Burst reading for detailed analysis
    println!("\nPerforming burst reading (10 samples)...");
    let burst = sensor.read_burst(10).await?;

    let values: Vec<f64> = burst.iter().map(|r| r.value * 1000.0).collect();
    let mean: f64 = values.iter().sum::<f64>() / values.len() as f64;
    let variance: f64 = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / values.len() as f64;
    let std_dev = variance.sqrt();

    println!("  Mean: {:.3} mM", mean);
    println!("  Std Dev: {:.4} mM", std_dev);
    println!("  CV: {:.2}%", (std_dev / mean) * 100.0);

    // Stop monitoring
    sensor.stop_monitoring().await?;

    // Final status
    let message = sensor.to_message()?;
    println!("\nSensor Status:");
    println!("{}", serde_json::to_string_pretty(&message)?);

    // Shutdown
    sensor.shutdown().await?;
    println!("\n✓ Monitoring session complete!");

    Ok(())
}
