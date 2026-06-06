//! Basic usage of the WIA Perception Clarity SDK.
//!
//! Demonstrates the three responsibilities of the standard:
//! 1. MEASURE — compute the PCI for a rainy RGB camera from its damage axes.
//! 2. STATE   — map the PCI to a clarity state and required safe actions.
//! 3. REPORT  — build a `SensorClarityReport` and serialize it to JSON.

use wia_perception_clarity::prelude::*;

fn main() -> Result<()> {
    println!("=== WIA Perception Clarity SDK — Basic Usage ===\n");

    // ------------------------------------------------------------------
    // 1. MEASURE — rainy front camera.
    // ------------------------------------------------------------------
    println!("1. Measuring PCI for a rainy rgb_camera...");
    let cam_axes = ClarityAxes::new(0.18, 0.22, 0.40);
    let cam_weights = PciWeights::default_for(SensorClass::RgbCamera);
    let cam_pci = compute_pci(&cam_axes, &cam_weights);
    println!("   axes = {:?}", cam_axes);
    println!("   weights = {:?} (sum {:.2})", cam_weights, cam_weights.sum());
    println!("   PCI = {}", cam_pci.value());

    // ------------------------------------------------------------------
    // 2. STATE — derive the state and the safe actions it requires.
    // ------------------------------------------------------------------
    let state = pci_to_state(cam_pci);
    println!("\n2. State = {:?}", state);
    println!("   required safe actions: {:?}", required_safe_actions(state));

    // ------------------------------------------------------------------
    // 3. REPORT — build and serialize a report (camera + clear LiDAR).
    // ------------------------------------------------------------------
    println!("\n3. Building a SensorClarityReport...");
    let front_cam = SensorBuilder::from_axes("front-cam-main", SensorClass::RgbCamera, cam_axes)
        .add_contaminant(Contaminant::new(ContaminantType::RainFilm, 0.35))
        .dwell_seconds(47.0)
        .confidence(0.92)
        .build()?;

    let roof_lidar = SensorBuilder::from_axes(
        "roof-lidar-1",
        SensorClass::LidarWindow,
        ClarityAxes::clean(),
    )
    .confidence(0.98)
    .build()?;

    let report = ReportBuilder::new("veh-seoul-0421", AgentType::Vehicle)
        .conformance_level(ConformanceLevel::Lc)
        .add_sensor(front_cam)
        .add_sensor(roof_lidar)
        .build()?;

    println!("   worst sensor state = {:?}", report.worst_state());

    let json = serde_json::to_string_pretty(&report)?;
    println!("\n--- SensorClarityReport (JSON) ---\n{}", json);

    Ok(())
}
