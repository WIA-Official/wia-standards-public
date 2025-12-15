//! Integration tests for WIA Physics SDK

use wia_physics::prelude::*;

// ============================================================================
// Type Tests
// ============================================================================

#[test]
fn test_measurement_with_uncertainty() {
    let m = Measurement::with_uncertainty(100.0, 3.0, 4.0, "GeV");
    assert_eq!(m.value, 100.0);
    assert!((m.uncertainty.total - 5.0).abs() < 1e-10);
    assert_eq!(m.uncertainty.statistical, Some(3.0));
    assert_eq!(m.uncertainty.systematic, Some(4.0));
}

#[test]
fn test_measurement_compatibility() {
    let m1 = Measurement::new(100.0, 5.0, "GeV");
    let m2 = Measurement::new(108.0, 5.0, "GeV");
    let m3 = Measurement::new(120.0, 5.0, "GeV");

    assert!(m1.is_compatible_with(&m2)); // Within 2-sigma
    assert!(!m1.is_compatible_with(&m3)); // Beyond 2-sigma
}

#[test]
fn test_metadata_generation() {
    let meta1 = Metadata::new();
    let meta2 = Metadata::new();

    assert_ne!(meta1.id, meta2.id); // UUIDs should be unique
    assert_eq!(meta1.license, "CC-BY-4.0");
}

#[test]
fn test_quality_flag_serialization() {
    let flag = QualityFlag::Good;
    let json = serde_json::to_string(&flag).unwrap();
    assert_eq!(json, "\"GOOD\"");

    let parsed: QualityFlag = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed, QualityFlag::Good);
}

// ============================================================================
// Fusion Tests
// ============================================================================

#[test]
fn test_fusion_data_builder() {
    let data = FusionDataBuilder::new()
        .experiment("Test Experiment")
        .plasma_simple(150e6, "K", 1e20, "m^-3")
        .tokamak(5.3, 6.2, 2.0)
        .quality(QualityFlag::Good)
        .build()
        .unwrap();

    assert_eq!(data.plasma.temperature.value, 150e6);
    assert_eq!(data.plasma.density.unit, "m^-3");
    assert!(data.magnetic_configuration.is_some());

    let config = data.magnetic_configuration.unwrap();
    assert_eq!(config.confinement_type, ConfinementType::Tokamak);
}

#[test]
fn test_fusion_data_serialization() {
    let data = FusionDataBuilder::new()
        .experiment("ITER")
        .plasma_simple(150e6, "K", 1e20, "m^-3")
        .build()
        .unwrap();

    let json = serde_json::to_string_pretty(&data).unwrap();
    assert!(json.contains("ITER"));
    assert!(json.contains("150000000"));

    let parsed: FusionData = serde_json::from_str(&json).unwrap();
    assert_eq!(parsed.plasma.temperature.value, data.plasma.temperature.value);
}

#[test]
fn test_fuel_composition() {
    let fuel = FuelComposition {
        fuel_type: FuelType::DT,
        deuterium_fraction: Some(0.5),
        tritium_fraction: Some(0.5),
        helium3_fraction: None,
        impurity_fraction: Some(0.02),
        z_effective: Some(Measurement::new(1.5, 0.1, "")),
    };

    let json = serde_json::to_string(&fuel).unwrap();
    assert!(json.contains("D-T"));
}

// ============================================================================
// Particle Physics Tests
// ============================================================================

#[test]
fn test_four_momentum_calculations() {
    // Higgs boson at rest
    let higgs = FourMomentum {
        e: Measurement::new(125.0, 0.1, "GeV"),
        px: Measurement::new(0.0, 0.01, "GeV"),
        py: Measurement::new(0.0, 0.01, "GeV"),
        pz: Measurement::new(0.0, 0.01, "GeV"),
        pt: None,
        eta: None,
        phi: None,
        mass: None,
    };

    assert!((higgs.invariant_mass() - 125.0).abs() < 0.1);
    assert!(higgs.transverse_momentum().abs() < 0.1);
}

#[test]
fn test_particle_properties() {
    let electron = ParticleProperties {
        name: "electron".to_string(),
        pdg_id: 11,
        mass: Measurement::new(0.000511, 0.0000001, "GeV"),
        charge: -1.0,
        spin: 0.5,
        parity: Some(-1),
        lifetime: None,
        width: None,
        particle_type: Some(ParticleType::Lepton),
    };

    let json = serde_json::to_string(&electron).unwrap();
    assert!(json.contains("electron"));
    assert!(json.contains("11"));
}

#[test]
fn test_collision_event() {
    let event = CollisionEvent {
        event_id: "test-001".to_string(),
        run_number: Some(12345),
        event_number: Some(1),
        timestamp: None,
        collision: CollisionInfo {
            sqrt_s: Measurement::new(13600.0, 1.0, "GeV"),
            beam1: "proton".to_string(),
            beam2: "proton".to_string(),
            collision_type: Some("pp".to_string()),
        },
        jets: vec![],
        electrons: vec![],
        muons: vec![],
        photons: vec![],
        missing_et: None,
        trigger: vec!["HLT_Jet400".to_string()],
    };

    let json = serde_json::to_string(&event).unwrap();
    assert!(json.contains("test-001"));
    assert!(json.contains("13600"));
}

// ============================================================================
// Dark Matter Tests
// ============================================================================

#[test]
fn test_exclusion_limit() {
    let limit = ExclusionLimit {
        dm_candidate: DarkMatterCandidate::Wimp,
        dm_mass: Some(Measurement::new(100.0, 10.0, "GeV")),
        mass_range: None,
        cross_section_limit: Some(Measurement::new(1e-47, 1e-48, "cm^2")),
        coupling_limit: None,
        confidence_level: 0.90,
        interaction_type: InteractionType::SI,
        exposure: Some(Measurement::new(1000.0, 10.0, "kg·day")),
    };

    let json = serde_json::to_string(&limit).unwrap();
    assert!(json.contains("wimp"));
    assert!(json.contains("0.9"));
}

#[test]
fn test_axion_search() {
    let search = AxionSearch {
        search_type: "haloscope".to_string(),
        frequency_range: Some(Range {
            min: 1e9,
            max: 2e9,
            unit: Some("Hz".to_string()),
        }),
        axion_mass_range: Some(Range {
            min: 1e-6,
            max: 1e-5,
            unit: Some("eV".to_string()),
        }),
        coupling_type: Some("photon".to_string()),
        gagg_limit: Some(Measurement::new(1e-15, 1e-16, "GeV^-1")),
        magnetic_field: Some(Measurement::new(8.0, 0.1, "T")),
        quality_factor: Some(Measurement::new(100000.0, 10000.0, "")),
    };

    let json = serde_json::to_string(&search).unwrap();
    assert!(json.contains("haloscope"));
}

// ============================================================================
// Physics Calculations Tests
// ============================================================================

#[test]
fn test_fusion_physics_calculations() {
    // Q-factor
    let q = FusionPhysics::q_factor(500.0, 50.0).unwrap();
    assert!((q - 10.0).abs() < 1e-10);

    // Triple product
    let tp = FusionPhysics::triple_product(1e20, 15.0, 3.0);
    assert!((tp - 4.5e21).abs() < 1e20);

    // Aspect ratio
    let ar = FusionPhysics::aspect_ratio(6.2, 2.0).unwrap();
    assert!((ar - 3.1).abs() < 1e-10);

    // Tokamak volume
    let v = FusionPhysics::tokamak_volume(6.2, 2.0);
    assert!(v > 400.0 && v < 600.0); // ~490 m³
}

#[test]
fn test_particle_physics_calculations() {
    // Invariant mass (particle with E=130, p=50 has m=120)
    let mass = ParticlePhysics::invariant_mass(130.0, 30.0, 40.0, 0.0);
    assert!((mass - 120.0).abs() < 0.1);

    // Transverse momentum
    let pt = ParticlePhysics::transverse_momentum(30.0, 40.0);
    assert!((pt - 50.0).abs() < 1e-10);

    // Significance
    let sig = ParticlePhysics::significance(100.0, 25.0).unwrap();
    assert!((sig - 20.0).abs() < 1e-10);
    assert!(ParticlePhysics::is_discovery(sig));
}

#[test]
fn test_unit_conversions() {
    // eV to Kelvin
    let kelvin = UnitConverter::ev_to_kelvin(1.0);
    assert!((kelvin - 11604.518).abs() < 1.0);

    // keV to Kelvin
    let kelvin = UnitConverter::kev_to_kelvin(10.0);
    assert!(kelvin > 1e8); // ~116 million K

    // Tesla to Gauss
    let gauss = UnitConverter::tesla_to_gauss(1.0);
    assert_eq!(gauss, 10000.0);
}

#[test]
fn test_quantum_gravity_calculations() {
    // Schwarzschild radius of Earth (~9mm)
    let r_s = QuantumGravityPhysics::schwarzschild_radius(5.97e24);
    assert!(r_s > 0.008 && r_s < 0.01);

    // Hawking temperature (solar mass BH is very cold)
    let t = QuantumGravityPhysics::hawking_temperature(1.989e30);
    assert!(t < 1e-6);
}

// ============================================================================
// Simulator Tests
// ============================================================================

#[tokio::test]
async fn test_fusion_simulator() {
    let mut sim = FusionSimulator::iter_like();

    // Initial state
    assert_eq!(sim.current_time(), 0.0);

    // Run simulation
    for _ in 0..100 {
        sim.step().await.unwrap();
    }

    assert!(sim.current_time() > 0.0);

    // Get state
    let state = sim.get_state().unwrap();
    assert!(state.plasma.temperature.value > 0.0);
    assert_eq!(state.quality, QualityFlag::Simulated);

    // Reset
    sim.reset();
    assert_eq!(sim.current_time(), 0.0);
}

#[test]
fn test_event_generator() {
    let mut gen = EventGenerator::lhc_run3();

    let event1 = gen.generate_event();
    let event2 = gen.generate_event();

    assert_ne!(event1.event_id, event2.event_id);
    assert_eq!(event1.collision.sqrt_s.value, 13600.0);
    assert!(!event1.jets.is_empty());
}

#[test]
fn test_dark_matter_generator() {
    let mut gen = DarkMatterGenerator::new(100.0, 1e-45);

    let data = gen.generate_data().unwrap();
    assert!(data.detection_event.is_some());

    let event = data.detection_event.unwrap();
    assert!(event.recoil_energy.value > 0.0);
    assert_eq!(event.signal_type, SignalType::NuclearRecoil);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

#[test]
fn test_error_types() {
    let err = PhysicsError::validation("test error");
    assert_eq!(err.to_string(), "Validation error: test error");

    let err = PhysicsError::unit_conversion("keV", "kg");
    assert!(err.to_string().contains("keV"));
    assert!(err.to_string().contains("kg"));
}

#[test]
fn test_fusion_builder_validation() {
    // Should fail without plasma parameters
    let result = FusionDataBuilder::new()
        .experiment("Test")
        .build();

    assert!(result.is_err());
}

// ============================================================================
// JSON Schema Compliance Tests
// ============================================================================

#[test]
fn test_full_fusion_json_structure() {
    let data = FusionDataBuilder::new()
        .experiment("ITER")
        .plasma_simple(150e6, "K", 1e20, "m^-3")
        .tokamak(5.3, 6.2, 2.0)
        .energy_balance(EnergyBalance {
            input_power: Some(Measurement::new(50.0, 1.0, "MW")),
            fusion_power: Some(Measurement::new(500.0, 10.0, "MW")),
            q_factor: Some(Measurement::new(10.0, 0.5, "")),
            neutron_power: None,
            alpha_heating_power: None,
            radiation_loss: None,
            stored_energy: None,
        })
        .quality(QualityFlag::Simulated)
        .build()
        .unwrap();

    let json = serde_json::to_value(&data).unwrap();

    // Check structure
    assert!(json.get("metadata").is_some());
    assert!(json.get("plasma").is_some());
    assert!(json.get("magnetic_configuration").is_some());
    assert!(json.get("energy_balance").is_some());

    // Check nested structure
    let plasma = json.get("plasma").unwrap();
    assert!(plasma.get("temperature").is_some());
    assert!(plasma.get("density").is_some());

    let temp = plasma.get("temperature").unwrap();
    assert!(temp.get("value").is_some());
    assert!(temp.get("uncertainty").is_some());
    assert!(temp.get("unit").is_some());
}
