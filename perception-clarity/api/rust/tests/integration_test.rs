//! Integration tests for the WIA Perception Clarity SDK.

use wia_perception_clarity::prelude::*;

// ============================================================================
// PCI computation — Phase 1 worked examples
// ============================================================================

#[test]
fn rain_camera_yields_degraded() {
    // Phase 1 §4.2: OCC 0.18, DIST 0.22, MTF 0.40, weights 0.40/0.25/0.35.
    // 100 - (0.072 + 0.055 + 0.140) * 100 = 73.3 -> 73, within the degraded band.
    let axes = ClarityAxes::new(0.18, 0.22, 0.40);
    let weights = PciWeights::default_for(SensorClass::RgbCamera);
    let pci = compute_pci(&axes, &weights);

    assert_eq!(pci.value(), 73);
    assert_eq!(pci.state(), ClarityState::Degraded);
}

#[test]
fn clean_sensor_yields_clear_100() {
    for class in [
        SensorClass::RgbCamera,
        SensorClass::IrThermal,
        SensorClass::LidarWindow,
        SensorClass::RadarRadome,
        SensorClass::Ultrasonic,
    ] {
        let pci = compute_pci(&ClarityAxes::clean(), &PciWeights::default_for(class));
        assert_eq!(pci.value(), 100, "clean {:?} should be PCI 100", class);
        assert_eq!(pci.state(), ClarityState::Clear);
    }
}

#[test]
fn mud_lidar_yields_blind() {
    // Phase 1 §4.3: OCC 0.81, DIST 0.88, MTF 0.30, lidar weights 0.45/0.45/0.10.
    let axes = ClarityAxes::new(0.81, 0.88, 0.30);
    let weights = PciWeights::default_for(SensorClass::LidarWindow);
    let pci = compute_pci(&axes, &weights);

    assert!(pci.value() <= 29, "expected blind band, got PCI {}", pci.value());
    assert_eq!(pci.state(), ClarityState::Blind);
}

// ============================================================================
// Weight tables — Phase 1 §3.2
// ============================================================================

#[test]
fn all_default_weight_sets_sum_to_one() {
    for class in [
        SensorClass::RgbCamera,
        SensorClass::IrThermal,
        SensorClass::LidarWindow,
        SensorClass::RadarRadome,
        SensorClass::Ultrasonic,
    ] {
        let w = PciWeights::default_for(class);
        assert!(w.is_valid(), "{:?} weights must sum to 1.00, got {}", class, w.sum());
        assert!((w.sum() - 1.0).abs() <= 0.001);
    }
}

// ============================================================================
// State ordering invariant — Phase 1 §2.2
// ============================================================================

#[test]
fn clarity_state_ordering_is_clear_best_blind_worst() {
    assert!(ClarityState::Clear > ClarityState::Degraded);
    assert!(ClarityState::Degraded > ClarityState::Obstructed);
    assert!(ClarityState::Obstructed > ClarityState::Blind);

    let states = [
        ClarityState::Clear,
        ClarityState::Blind,
        ClarityState::Degraded,
    ];
    assert_eq!(states.iter().min().copied(), Some(ClarityState::Blind));
}

// ============================================================================
// Report build + JSON round-trip
// ============================================================================

#[test]
fn report_serializes_with_correct_wire_values() {
    let cam = SensorBuilder::from_axes("front-cam-main", SensorClass::RgbCamera, ClarityAxes::new(0.18, 0.22, 0.40))
        .add_contaminant(Contaminant::new(ContaminantType::RainFilm, 0.35))
        .dwell_seconds(47.0)
        .confidence(0.92)
        .build()
        .unwrap();

    let report = ReportBuilder::new("veh-seoul-0421", AgentType::Vehicle)
        .conformance_level(ConformanceLevel::Lc)
        .add_sensor(cam)
        .build()
        .unwrap();

    let json = serde_json::to_string(&report).unwrap();

    // snake_case enum wire values from Phase 1.
    assert!(json.contains("\"rgb_camera\""));
    assert!(json.contains("\"degraded\""));
    assert!(json.contains("\"rain_film\""));
    assert!(json.contains("\"vehicle\""));
    // Hyphenated conformance level.
    assert!(json.contains("\"L-C\""));
    // camelCase field names.
    assert!(json.contains("\"sensorId\""));
    assert!(json.contains("\"distanceDegradation\""));
    assert!(json.contains("\"dwellSeconds\""));

    // Round-trip back to a value.
    let back: SensorClarityReport = serde_json::from_str(&json).unwrap();
    assert_eq!(back.sensors.len(), 1);
    assert_eq!(back.sensors[0].state, ClarityState::Degraded);
    assert_eq!(back.worst_state(), ClarityState::Degraded);
}

// ============================================================================
// Consistency rules — Phase 1 §6.3
// ============================================================================

#[test]
fn dwell_on_clear_is_rejected() {
    let result = SensorBuilder::new("u", SensorClass::Ultrasonic, PciIndex::new(95))
        .dwell_seconds(10.0)
        .build();
    assert!(result.is_err());
}

#[test]
fn empty_report_is_rejected() {
    let result = ReportBuilder::new("veh-1", AgentType::Vehicle).build();
    assert!(result.is_err());
}

// ============================================================================
// Integrations
// ============================================================================

#[test]
fn integrations_map_states_to_actions() {
    let blind_lidar = SensorBuilder::from_axes("nav-lidar", SensorClass::LidarWindow, ClarityAxes::new(0.81, 0.88, 0.30))
        .dwell_seconds(312.0)
        .build()
        .unwrap();
    let report = ReportBuilder::new("amr-site-12", AgentType::Amr)
        .add_sensor(blind_lidar.clone())
        .build()
        .unwrap();

    // Auto: blind -> minimal-risk maneuver.
    assert_eq!(sae_fallback(&report), SaeFallback::MinimalRiskManeuver);
    // Drone: blind -> land immediately.
    assert_eq!(drone_action(&report), DroneAction::LandImmediately);
    // Fusion: blind sensor excluded.
    assert_eq!(fusion_weight(&blind_lidar).value(), 0.0);
    // Vision AI: blind -> inference skipped.
    assert!(!gate_inference(&blind_lidar, PciIndex::new(50)).run_inference);
}
