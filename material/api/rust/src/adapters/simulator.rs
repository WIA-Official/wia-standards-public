//! Simulator adapter for testing and development
//!
//! This adapter provides simulated material data for testing purposes.

use super::{MaterialAdapter, MaterialQuery};
use crate::error::{MaterialError, MaterialResult};
use crate::types::*;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Simulator adapter for testing
pub struct SimulatorAdapter {
    name: String,
    connected: bool,
    materials: Arc<RwLock<HashMap<String, MaterialData>>>,
    /// Simulated latency in milliseconds
    latency_ms: u64,
}

impl SimulatorAdapter {
    /// Create a new simulator adapter
    pub fn new() -> Self {
        Self {
            name: "Simulator".to_string(),
            connected: false,
            materials: Arc::new(RwLock::new(HashMap::new())),
            latency_ms: 0,
        }
    }

    /// Create a simulator with artificial latency
    pub fn with_latency(mut self, latency_ms: u64) -> Self {
        self.latency_ms = latency_ms;
        self
    }

    /// Simulate network latency
    async fn simulate_latency(&self) {
        if self.latency_ms > 0 {
            tokio::time::sleep(tokio::time::Duration::from_millis(self.latency_ms)).await;
        }
    }

    /// Populate with sample data
    pub async fn populate_sample_data(&self) -> MaterialResult<()> {
        // Sample YBCO superconductor
        let ybco = MaterialData {
            schema: Some("https://wia.live/material/v1/superconductor.schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_type: MaterialType::Superconductor,
            material_id: "wia-mat-sc000001".to_string(),
            timestamp: Timestamp::default(),
            identity: Identity {
                name: "YBCO".to_string(),
                formula: "YBa2Cu3O7-x".to_string(),
                classification: Some(vec![
                    "cuprate".to_string(),
                    "high_tc".to_string(),
                    "type_ii".to_string(),
                ]),
            },
            structure: Some(Structure {
                crystal_system: Some(CrystalSystem::Orthorhombic),
                space_group: Some("Pmmm".to_string()),
                lattice_parameters: Some(LatticeParameters {
                    a_angstrom: Some(3.82),
                    b_angstrom: Some(3.89),
                    c_angstrom: Some(11.68),
                    alpha_degree: Some(90.0),
                    beta_degree: Some(90.0),
                    gamma_degree: Some(90.0),
                }),
            }),
            properties: Properties {
                electrical: Some(ElectricalProperties {
                    resistivity_ohm_m: Some(0.0),
                    ..Default::default()
                }),
                domain_specific: Some(serde_json::json!({
                    "critical_temperature_k": 93.0,
                    "critical_pressure_pa": 101325,
                    "critical_magnetic_field_t": 100.0,
                    "meissner_effect": true,
                    "superconductor_type": "type_ii"
                })),
                ..Default::default()
            },
            measurement: Some(Measurement {
                temperature_k: Some(77.0),
                pressure_pa: Some(101325.0),
                method: Some("four_probe".to_string()),
                ..Default::default()
            }),
            provenance: Some(Provenance {
                source: Some("DOI:10.1038/nature00000".to_string()),
                lab: Some("WIA Materials Lab".to_string()),
                ..Default::default()
            }),
            external_references: None,
            meta: Some(Meta {
                confidence: Some(0.98),
                validated: Some(true),
                notes: Some("High-quality single crystal sample".to_string()),
            }),
        };

        // Sample Bi2Se3 topological insulator
        let bi2se3 = MaterialData {
            schema: Some("https://wia.live/material/v1/topological-insulator.schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_type: MaterialType::TopologicalInsulator,
            material_id: "wia-mat-ti000001".to_string(),
            timestamp: Timestamp::default(),
            identity: Identity {
                name: "Bismuth Selenide".to_string(),
                formula: "Bi2Se3".to_string(),
                classification: Some(vec![
                    "chalcogenide".to_string(),
                    "3d_topological_insulator".to_string(),
                    "strong_ti".to_string(),
                ]),
            },
            structure: Some(Structure {
                crystal_system: Some(CrystalSystem::Rhombohedral),
                space_group: Some("R-3m".to_string()),
                lattice_parameters: Some(LatticeParameters {
                    a_angstrom: Some(4.14),
                    c_angstrom: Some(28.64),
                    ..Default::default()
                }),
            }),
            properties: Properties {
                electrical: Some(ElectricalProperties {
                    band_gap_ev: Some(0.3),
                    carrier_mobility_cm2_vs: Some(1000.0),
                    ..Default::default()
                }),
                domain_specific: Some(serde_json::json!({
                    "band_gap_ev": 0.3,
                    "z2_invariant": [1, 0, 0, 0],
                    "dirac_point_ev": -0.1,
                    "surface_state": {
                        "fermi_velocity_m_s": 5e5,
                        "spin_texture": "helical"
                    },
                    "spin_hall_angle": 0.3
                })),
                ..Default::default()
            },
            measurement: Some(Measurement {
                temperature_k: Some(10.0),
                method: Some("ARPES".to_string()),
                ..Default::default()
            }),
            provenance: None,
            external_references: None,
            meta: Some(Meta {
                confidence: Some(0.95),
                validated: Some(true),
                notes: None,
            }),
        };

        // Sample TiO2 memristor
        let tio2_memristor = MaterialData {
            schema: Some("https://wia.live/material/v1/memristor.schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_type: MaterialType::Memristor,
            material_id: "wia-mat-mr000001".to_string(),
            timestamp: Timestamp::default(),
            identity: Identity {
                name: "TiO2 Memristor".to_string(),
                formula: "TiO2".to_string(),
                classification: Some(vec![
                    "oxide".to_string(),
                    "resistive_switching".to_string(),
                    "neuromorphic".to_string(),
                ]),
            },
            structure: None,
            properties: Properties {
                domain_specific: Some(serde_json::json!({
                    "material": "TiO2",
                    "structure": "metal_insulator_metal",
                    "resistance_high_ohm": 1e6,
                    "resistance_low_ohm": 1e3,
                    "on_off_ratio": 1000,
                    "set_voltage_v": 1.0,
                    "reset_voltage_v": -0.8,
                    "endurance_cycles": 1e12,
                    "neuromorphic": {
                        "synaptic_weight": 0.5,
                        "plasticity": "stdp",
                        "analog_states": 128
                    }
                })),
                ..Default::default()
            },
            measurement: None,
            provenance: Some(Provenance {
                lab: Some("HP Labs".to_string()),
                ..Default::default()
            }),
            external_references: None,
            meta: Some(Meta {
                confidence: Some(0.92),
                validated: Some(true),
                notes: None,
            }),
        };

        // Sample metamaterial
        let srr_metamaterial = MaterialData {
            schema: Some("https://wia.live/material/v1/metamaterial.schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_type: MaterialType::Metamaterial,
            material_id: "wia-mat-mm000001".to_string(),
            timestamp: Timestamp::default(),
            identity: Identity {
                name: "Split Ring Resonator Array".to_string(),
                formula: "Cu/FR4".to_string(),
                classification: Some(vec![
                    "electromagnetic".to_string(),
                    "negative_index".to_string(),
                    "microwave".to_string(),
                ]),
            },
            structure: None,
            properties: Properties {
                domain_specific: Some(serde_json::json!({
                    "metamaterial_type": "electromagnetic",
                    "unit_cell": {
                        "type": "split_ring_resonator",
                        "period_um": 100.0
                    },
                    "permittivity_real": -2.5,
                    "permeability_real": -1.2,
                    "refractive_index": -1.73,
                    "operating_frequency_hz": 10e9,
                    "absorption_percent": 95.0
                })),
                ..Default::default()
            },
            measurement: None,
            provenance: None,
            external_references: None,
            meta: Some(Meta {
                confidence: Some(0.90),
                validated: Some(true),
                notes: None,
            }),
        };

        // Store all samples
        let mut materials = self.materials.write().await;
        materials.insert(ybco.material_id.clone(), ybco);
        materials.insert(bi2se3.material_id.clone(), bi2se3);
        materials.insert(tio2_memristor.material_id.clone(), tio2_memristor);
        materials.insert(srr_metamaterial.material_id.clone(), srr_metamaterial);

        Ok(())
    }
}

impl Default for SimulatorAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl MaterialAdapter for SimulatorAdapter {
    fn name(&self) -> &str {
        &self.name
    }

    async fn connect(&mut self) -> MaterialResult<()> {
        self.simulate_latency().await;
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> MaterialResult<()> {
        self.simulate_latency().await;
        self.connected = false;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    async fn fetch(&self, id: &str) -> MaterialResult<MaterialData> {
        if !self.connected {
            return Err(MaterialError::ConnectionError(
                "Adapter not connected".to_string(),
            ));
        }

        self.simulate_latency().await;

        let materials = self.materials.read().await;
        materials
            .get(id)
            .cloned()
            .ok_or_else(|| MaterialError::NotFound(id.to_string()))
    }

    async fn store(&self, data: &MaterialData) -> MaterialResult<()> {
        if !self.connected {
            return Err(MaterialError::ConnectionError(
                "Adapter not connected".to_string(),
            ));
        }

        self.simulate_latency().await;

        let mut materials = self.materials.write().await;
        materials.insert(data.material_id.clone(), data.clone());
        Ok(())
    }

    async fn list(&self) -> MaterialResult<Vec<String>> {
        if !self.connected {
            return Err(MaterialError::ConnectionError(
                "Adapter not connected".to_string(),
            ));
        }

        self.simulate_latency().await;

        let materials = self.materials.read().await;
        Ok(materials.keys().cloned().collect())
    }

    async fn search(&self, query: &MaterialQuery) -> MaterialResult<Vec<MaterialData>> {
        if !self.connected {
            return Err(MaterialError::ConnectionError(
                "Adapter not connected".to_string(),
            ));
        }

        self.simulate_latency().await;

        let materials = self.materials.read().await;
        let mut results: Vec<MaterialData> = materials
            .values()
            .filter(|m| {
                // Filter by material type
                if let Some(ref mt) = query.material_type {
                    if m.material_type != *mt {
                        return false;
                    }
                }

                // Filter by name
                if let Some(ref name) = query.name_contains {
                    if !m.identity.name.to_lowercase().contains(&name.to_lowercase()) {
                        return false;
                    }
                }

                // Filter by formula
                if let Some(ref formula) = query.formula {
                    if m.identity.formula != *formula {
                        return false;
                    }
                }

                // Filter by classification
                if let Some(ref tag) = query.classification {
                    if let Some(ref classifications) = m.identity.classification {
                        if !classifications.iter().any(|c| c == tag) {
                            return false;
                        }
                    } else {
                        return false;
                    }
                }

                // Filter by confidence
                if let Some(min_conf) = query.min_confidence {
                    if let Some(ref meta) = m.meta {
                        if let Some(conf) = meta.confidence {
                            if conf < min_conf {
                                return false;
                            }
                        }
                    }
                }

                true
            })
            .cloned()
            .collect();

        // Apply offset and limit
        if let Some(offset) = query.offset {
            results = results.into_iter().skip(offset).collect();
        }
        if let Some(limit) = query.limit {
            results.truncate(limit);
        }

        Ok(results)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_simulator_connect() {
        let mut adapter = SimulatorAdapter::new();
        assert!(!adapter.is_connected());

        adapter.connect().await.unwrap();
        assert!(adapter.is_connected());

        adapter.disconnect().await.unwrap();
        assert!(!adapter.is_connected());
    }

    #[tokio::test]
    async fn test_simulator_populate_and_fetch() {
        let mut adapter = SimulatorAdapter::new();
        adapter.connect().await.unwrap();
        adapter.populate_sample_data().await.unwrap();

        let ybco = adapter.fetch("wia-mat-sc000001").await.unwrap();
        assert_eq!(ybco.identity.name, "YBCO");
        assert_eq!(ybco.material_type, MaterialType::Superconductor);
    }

    #[tokio::test]
    async fn test_simulator_search() {
        let mut adapter = SimulatorAdapter::new();
        adapter.connect().await.unwrap();
        adapter.populate_sample_data().await.unwrap();

        let query = MaterialQuery::new().with_type(MaterialType::Superconductor);
        let results = adapter.search(&query).await.unwrap();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].identity.name, "YBCO");
    }

    #[tokio::test]
    async fn test_simulator_list() {
        let mut adapter = SimulatorAdapter::new();
        adapter.connect().await.unwrap();
        adapter.populate_sample_data().await.unwrap();

        let ids = adapter.list().await.unwrap();
        assert_eq!(ids.len(), 4);
    }
}
