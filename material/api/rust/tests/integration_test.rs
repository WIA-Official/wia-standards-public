//! Integration tests for WIA Material SDK

use wia_material::prelude::*;
use wia_material::{schema_url, VERSION};

#[tokio::test]
async fn test_create_superconductor() {
    let client = WiaMaterial::new();

    let props = SuperconductorProperties {
        critical_temperature_k: 93.0,
        critical_pressure_pa: Some(101325.0),
        critical_current_density_a_m2: Some(1e10),
        critical_magnetic_field_t: Some(100.0),
        meissner_effect: Some(true),
        superconductor_type: Some(wia_material::SuperconductorType::TypeII),
        coherence_length_nm: Some(1.5),
        penetration_depth_nm: Some(150.0),
    };

    let material = client
        .create_superconductor("YBCO", "YBa2Cu3O7-x", props)
        .await
        .unwrap();

    assert_eq!(material.material_type, MaterialType::Superconductor);
    assert_eq!(material.identity.name, "YBCO");
    assert!(material.material_id.starts_with("wia-mat-"));
}

#[tokio::test]
async fn test_create_topological_insulator() {
    let client = WiaMaterial::new();

    let props = TopologicalInsulatorProperties {
        band_gap_ev: 0.3,
        z2_invariant: Some(vec![1, 0, 0, 0]),
        dirac_point_ev: Some(-0.1),
        surface_state: Some(wia_material::SurfaceState {
            fermi_velocity_m_s: Some(5e5),
            spin_texture: Some("helical".to_string()),
            surface_conductivity_s: None,
        }),
        spin_hall_angle: Some(0.3),
        spin_diffusion_length_nm: Some(10.0),
    };

    let material = client
        .create_topological_insulator("Bismuth Selenide", "Bi2Se3", props)
        .await
        .unwrap();

    assert_eq!(material.material_type, MaterialType::TopologicalInsulator);
    assert_eq!(material.identity.formula, "Bi2Se3");
}

#[tokio::test]
async fn test_create_memristor() {
    let client = WiaMaterial::new();

    let props = MemristorProperties {
        material: "TiO2".to_string(),
        structure: Some(wia_material::MemristorStructure::MetalInsulatorMetal),
        resistance_high_ohm: 1e6,
        resistance_low_ohm: 1e3,
        on_off_ratio: Some(1000.0),
        set_voltage_v: Some(1.0),
        reset_voltage_v: Some(-0.8),
        endurance_cycles: Some(1e12),
        neuromorphic: Some(wia_material::NeuromorphicProperties {
            synaptic_weight: Some(0.5),
            plasticity: Some("stdp".to_string()),
            learning_rate: Some(0.01),
            analog_states: Some(128),
        }),
    };

    let material = client
        .create_memristor("TiO2 Memristor", "TiO2", props)
        .await
        .unwrap();

    assert_eq!(material.material_type, MaterialType::Memristor);
}

#[tokio::test]
async fn test_create_metamaterial() {
    let client = WiaMaterial::new();

    let props = MetamaterialProperties {
        metamaterial_type: wia_material::MetamaterialType::Electromagnetic,
        unit_cell: Some(wia_material::UnitCell {
            cell_type: "split_ring_resonator".to_string(),
            period_um: Some(100.0),
            dimensions: None,
        }),
        permittivity_real: Some(-2.5),
        permittivity_imag: Some(0.1),
        permeability_real: Some(-1.2),
        permeability_imag: Some(0.05),
        refractive_index: Some(-1.73),
        operating_frequency_hz: Some(10e9),
        absorption_percent: Some(95.0),
    };

    let material = client
        .create_metamaterial("SRR Array", props)
        .await
        .unwrap();

    assert_eq!(material.material_type, MaterialType::Metamaterial);
}

#[tokio::test]
async fn test_material_builder() {
    let material = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("Test Material")
        .formula("TestFormula")
        .classification(vec!["test".to_string(), "sample".to_string()])
        .structure(Structure {
            crystal_system: Some(CrystalSystem::Cubic),
            space_group: Some("Fm-3m".to_string()),
            lattice_parameters: Some(LatticeParameters {
                a_angstrom: Some(5.0),
                b_angstrom: Some(5.0),
                c_angstrom: Some(5.0),
                alpha_degree: Some(90.0),
                beta_degree: Some(90.0),
                gamma_degree: Some(90.0),
            }),
        })
        .electrical_properties(ElectricalProperties {
            resistivity_ohm_m: Some(0.0),
            ..Default::default()
        })
        .measurement(Measurement {
            temperature_k: Some(77.0),
            pressure_pa: Some(101325.0),
            method: Some("four_probe".to_string()),
            ..Default::default()
        })
        .confidence(0.95)
        .notes("Test notes")
        .build()
        .unwrap();

    assert_eq!(material.identity.name, "Test Material");
    assert!(material.identity.classification.is_some());
    assert!(material.structure.is_some());
    assert!(material.measurement.is_some());
    assert!(material.meta.is_some());
}

#[tokio::test]
async fn test_simulator_adapter() {
    let mut adapter = SimulatorAdapter::new();

    // Connect
    adapter.connect().await.unwrap();
    assert!(adapter.is_connected());

    // Populate with sample data
    adapter.populate_sample_data().await.unwrap();

    // List materials
    let ids = adapter.list().await.unwrap();
    assert_eq!(ids.len(), 4);

    // Fetch specific material
    let ybco = adapter.fetch("wia-mat-sc000001").await.unwrap();
    assert_eq!(ybco.identity.name, "YBCO");

    // Search by type
    let query = MaterialQuery::new().with_type(MaterialType::TopologicalInsulator);
    let results = adapter.search(&query).await.unwrap();
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].identity.formula, "Bi2Se3");

    // Search by classification
    let query = MaterialQuery::new().with_classification("neuromorphic");
    let results = adapter.search(&query).await.unwrap();
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].material_type, MaterialType::Memristor);

    // Disconnect
    adapter.disconnect().await.unwrap();
    assert!(!adapter.is_connected());
}

#[tokio::test]
async fn test_material_serialization() {
    let client = WiaMaterial::new();

    let material = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("Test")
        .formula("Test")
        .build()
        .unwrap();

    // Serialize to JSON
    let json = client.to_json(&material).unwrap();
    assert!(json.contains("\"material_type\": \"superconductor\""));
    assert!(json.contains("\"version\": \"1.0.0\""));

    // Deserialize from JSON
    let parsed = client.parse_json(&json).unwrap();
    assert_eq!(parsed.identity.name, material.identity.name);
    assert_eq!(parsed.material_type, material.material_type);
}

#[tokio::test]
async fn test_get_materials_by_type() {
    let client = WiaMaterial::new();

    // Create materials of different types
    let sc = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("SC1")
        .formula("SC1")
        .build()
        .unwrap();

    let ti = MaterialBuilder::new()
        .material_type(MaterialType::TopologicalInsulator)
        .name("TI1")
        .formula("TI1")
        .build()
        .unwrap();

    let mr = MaterialBuilder::new()
        .material_type(MaterialType::Memristor)
        .name("MR1")
        .formula("MR1")
        .build()
        .unwrap();

    client.create_material(sc).await.unwrap();
    client.create_material(ti).await.unwrap();
    client.create_material(mr).await.unwrap();

    // Filter by type
    let superconductors = client
        .get_materials_by_type(MaterialType::Superconductor)
        .await;
    assert_eq!(superconductors.len(), 1);
    assert_eq!(superconductors[0].identity.name, "SC1");

    let all = client.get_materials().await;
    assert_eq!(all.len(), 3);
}

#[tokio::test]
async fn test_statistics() {
    let client = WiaMaterial::new();

    // Create materials
    for i in 0..3 {
        let material = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name(format!("SC{}", i))
            .formula(format!("SC{}", i))
            .build()
            .unwrap();
        client.create_material(material).await.unwrap();
    }

    for i in 0..2 {
        let material = MaterialBuilder::new()
            .material_type(MaterialType::Memristor)
            .name(format!("MR{}", i))
            .formula(format!("MR{}", i))
            .build()
            .unwrap();
        client.create_material(material).await.unwrap();
    }

    let stats = client.get_statistics().await;
    assert_eq!(stats.total_count, 5);
    assert_eq!(stats.superconductor_count, 3);
    assert_eq!(stats.memristor_count, 2);
}

#[test]
fn test_version_constant() {
    assert_eq!(VERSION, "1.0.0");
}

#[test]
fn test_schema_urls() {
    assert_eq!(
        schema_url(MaterialType::Superconductor),
        "https://wia.live/material/v1/superconductor.schema.json"
    );
    assert_eq!(
        schema_url(MaterialType::TopologicalInsulator),
        "https://wia.live/material/v1/topological-insulator.schema.json"
    );
}

#[test]
fn test_validation_errors() {
    let client = WiaMaterial::new();

    // Invalid confidence
    let mut material = MaterialBuilder::new()
        .material_type(MaterialType::Superconductor)
        .name("Test")
        .formula("Test")
        .build()
        .unwrap();

    material.meta = Some(Meta {
        confidence: Some(1.5), // Invalid: > 1.0
        validated: None,
        notes: None,
    });

    let result = client.validate(&material);
    assert!(result.is_err());
}
