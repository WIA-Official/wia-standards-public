//! Materials Project provider implementation
//!
//! Provides access to the Materials Project database.
//! https://materialsproject.org

use async_trait::async_trait;
use chrono::Utc;
use serde::{Deserialize, Serialize};

use super::traits::{DataProvider, ProviderConfig, ProviderQuery, ProviderStatus};
use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::{
    CrystalSystem, ElectricalProperties, ExternalReferences, Identity, MaterialData, MaterialType,
    MechanicalProperties, Meta, Properties, Provenance, Structure, Timestamp,
};

/// Materials Project API base URL
pub const MP_API_BASE: &str = "https://api.materialsproject.org";

/// Materials Project material summary (simplified)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpMaterial {
    pub material_id: String,
    pub formula_pretty: String,
    pub composition: Option<MpComposition>,
    pub symmetry: Option<MpSymmetry>,
    pub band_gap: Option<f64>,
    pub formation_energy_per_atom: Option<f64>,
    pub energy_above_hull: Option<f64>,
    pub is_stable: Option<bool>,
    pub density: Option<f64>,
    pub density_atomic: Option<f64>,
    pub volume: Option<f64>,
    pub nsites: Option<i32>,
}

/// Materials Project composition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpComposition {
    pub elements: Vec<String>,
    pub nelements: i32,
}

/// Materials Project symmetry info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpSymmetry {
    pub crystal_system: Option<String>,
    pub symbol: Option<String>,
    pub number: Option<i32>,
    pub point_group: Option<String>,
}

/// Materials Project API response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpResponse {
    pub data: Vec<MpMaterial>,
}

/// Materials Project single response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MpSingleResponse {
    pub data: MpMaterial,
}

/// Materials Project provider
pub struct MaterialsProjectProvider {
    name: String,
    config: Option<ProviderConfig>,
    connected: bool,
    api_key: Option<String>,
}

impl MaterialsProjectProvider {
    /// Create a new Materials Project provider
    pub fn new() -> Self {
        Self {
            name: "materials_project".to_string(),
            config: None,
            connected: false,
            api_key: None,
        }
    }

    /// Create with an API key
    pub fn with_api_key(api_key: &str) -> Self {
        Self {
            name: "materials_project".to_string(),
            config: None,
            connected: false,
            api_key: Some(api_key.to_string()),
        }
    }

    /// Build query parameters
    fn build_query_params(&self, query: &ProviderQuery) -> Vec<(String, String)> {
        let mut params = Vec::new();

        if let Some(ref elements) = query.elements {
            params.push(("elements".to_string(), elements.join(",")));
        }

        if let Some(ref formula) = query.formula {
            params.push(("formula".to_string(), formula.clone()));
        }

        if let Some((min, max)) = query.band_gap_range {
            params.push(("band_gap_min".to_string(), min.to_string()));
            params.push(("band_gap_max".to_string(), max.to_string()));
        }

        let limit = query.limit.unwrap_or(100);
        params.push(("_limit".to_string(), limit.to_string()));

        if let Some(offset) = query.offset {
            params.push(("_skip".to_string(), offset.to_string()));
        }

        // Request specific fields
        params.push((
            "_fields".to_string(),
            "material_id,formula_pretty,composition,symmetry,band_gap,formation_energy_per_atom,density,volume,nsites".to_string(),
        ));

        params
    }

    /// Convert Materials Project material to WIA MaterialData
    fn convert_material(&self, mp: &MpMaterial) -> IntegrationResult<MaterialData> {
        let now = Utc::now();

        // Build structure
        let structure = self.convert_structure(mp);

        // Build properties
        let properties = self.convert_properties(mp);

        Ok(MaterialData {
            schema: Some("https://wia.live/material/v1/schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_id: format!("wia-mat-{:08x}", generate_id() as u32),
            material_type: MaterialType::Custom,
            timestamp: Timestamp {
                created: now,
                modified: None,
            },
            identity: Identity {
                name: mp.formula_pretty.clone(),
                formula: mp.formula_pretty.clone(),
                classification: Some(vec!["materials_project_import".to_string()]),
            },
            structure,
            properties,
            measurement: None,
            provenance: Some(Provenance {
                source: Some("Materials Project".to_string()),
                lab: Some("Lawrence Berkeley National Laboratory".to_string()),
                operator: None,
            }),
            external_references: Some(ExternalReferences {
                materials_project_id: Some(mp.material_id.clone()),
                icsd_id: None,
                cod_id: None,
                doi: None,
            }),
            meta: Some(Meta {
                confidence: Some(0.95),
                validated: Some(true),
                notes: Some(format!("Imported from Materials Project: {}", mp.material_id)),
            }),
        })
    }

    /// Convert MP structure data
    fn convert_structure(&self, mp: &MpMaterial) -> Option<Structure> {
        let crystal_system = mp.symmetry.as_ref().and_then(|s| {
            s.crystal_system
                .as_ref()
                .and_then(|cs| match cs.to_lowercase().as_str() {
                    "cubic" => Some(CrystalSystem::Cubic),
                    "tetragonal" => Some(CrystalSystem::Tetragonal),
                    "orthorhombic" => Some(CrystalSystem::Orthorhombic),
                    "hexagonal" => Some(CrystalSystem::Hexagonal),
                    "trigonal" | "rhombohedral" => Some(CrystalSystem::Trigonal),
                    "monoclinic" => Some(CrystalSystem::Monoclinic),
                    "triclinic" => Some(CrystalSystem::Triclinic),
                    _ => None,
                })
        });

        let space_group = mp.symmetry.as_ref().and_then(|s| s.symbol.clone());

        if crystal_system.is_some() || space_group.is_some() {
            Some(Structure {
                crystal_system,
                space_group,
                lattice_parameters: None,
            })
        } else {
            None
        }
    }

    /// Convert MP properties
    fn convert_properties(&self, mp: &MpMaterial) -> Properties {
        let electrical = mp.band_gap.map(|bg| ElectricalProperties {
            band_gap_ev: Some(bg),
            resistivity_ohm_m: None,
            conductivity_s_m: None,
            carrier_mobility_cm2_vs: None,
            carrier_concentration_cm3: None,
        });

        let mechanical = mp.density.map(|d| MechanicalProperties {
            density_kg_m3: Some(d * 1000.0), // g/cm³ to kg/m³
            youngs_modulus_gpa: None,
            poissons_ratio: None,
            hardness_gpa: None,
        });

        Properties {
            electrical,
            magnetic: None,
            thermal: None,
            optical: None,
            mechanical,
            domain_specific: None,
            custom: None,
        }
    }

    /// Simulate Materials Project API fetch (for development/testing)
    async fn simulate_fetch(&self, _url: &str) -> IntegrationResult<MpResponse> {
        // In production, this would make actual HTTP requests with API key
        // For now, return simulated data
        Ok(MpResponse {
            data: vec![
                MpMaterial {
                    material_id: "mp-13".to_string(),
                    formula_pretty: "Fe".to_string(),
                    composition: Some(MpComposition {
                        elements: vec!["Fe".to_string()],
                        nelements: 1,
                    }),
                    symmetry: Some(MpSymmetry {
                        crystal_system: Some("Cubic".to_string()),
                        symbol: Some("Im-3m".to_string()),
                        number: Some(229),
                        point_group: Some("m-3m".to_string()),
                    }),
                    band_gap: Some(0.0),
                    formation_energy_per_atom: Some(0.0),
                    energy_above_hull: Some(0.0),
                    is_stable: Some(true),
                    density: Some(7.874),
                    density_atomic: None,
                    volume: Some(11.82),
                    nsites: Some(2),
                },
                MpMaterial {
                    material_id: "mp-2534".to_string(),
                    formula_pretty: "Si".to_string(),
                    composition: Some(MpComposition {
                        elements: vec!["Si".to_string()],
                        nelements: 1,
                    }),
                    symmetry: Some(MpSymmetry {
                        crystal_system: Some("Cubic".to_string()),
                        symbol: Some("Fd-3m".to_string()),
                        number: Some(227),
                        point_group: Some("m-3m".to_string()),
                    }),
                    band_gap: Some(1.11),
                    formation_energy_per_atom: Some(0.0),
                    energy_above_hull: Some(0.0),
                    is_stable: Some(true),
                    density: Some(2.33),
                    density_atomic: None,
                    volume: Some(40.05),
                    nsites: Some(8),
                },
                MpMaterial {
                    material_id: "mp-19306".to_string(),
                    formula_pretty: "Fe2O3".to_string(),
                    composition: Some(MpComposition {
                        elements: vec!["Fe".to_string(), "O".to_string()],
                        nelements: 2,
                    }),
                    symmetry: Some(MpSymmetry {
                        crystal_system: Some("Trigonal".to_string()),
                        symbol: Some("R-3c".to_string()),
                        number: Some(167),
                        point_group: Some("-3m".to_string()),
                    }),
                    band_gap: Some(2.1),
                    formation_energy_per_atom: Some(-1.5),
                    energy_above_hull: Some(0.0),
                    is_stable: Some(true),
                    density: Some(5.26),
                    density_atomic: None,
                    volume: Some(301.0),
                    nsites: Some(30),
                },
            ],
        })
    }
}

impl Default for MaterialsProjectProvider {
    fn default() -> Self {
        Self::new()
    }
}

fn generate_id() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

#[async_trait]
impl DataProvider for MaterialsProjectProvider {
    fn name(&self) -> &str {
        &self.name
    }

    async fn connect(&mut self, config: ProviderConfig) -> IntegrationResult<()> {
        if config.api_key.is_none() && self.api_key.is_none() {
            // Allow connection without API key for simulation mode
            // In production, this would require an API key
        }

        self.api_key = config.api_key.clone().or_else(|| self.api_key.clone());
        self.config = Some(config);
        self.connected = true;
        Ok(())
    }

    async fn disconnect(&mut self) -> IntegrationResult<()> {
        self.connected = false;
        self.config = None;
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    async fn status(&self) -> IntegrationResult<ProviderStatus> {
        Ok(ProviderStatus {
            name: self.name.clone(),
            connected: self.connected,
            version: "2023.11".to_string(),
            available_count: Some(150000), // Approximate MP count
            last_connected: if self.connected {
                Some(Utc::now().to_rfc3339())
            } else {
                None
            },
        })
    }

    async fn search(&self, query: &ProviderQuery) -> IntegrationResult<Vec<MaterialData>> {
        if !self.connected {
            return Err(IntegrationError::ProviderNotConnected(self.name.clone()));
        }

        let params = self.build_query_params(query);
        let query_string = params
            .iter()
            .map(|(k, v)| format!("{}={}", k, urlencoding::encode(v)))
            .collect::<Vec<_>>()
            .join("&");

        let url = format!("{}/materials/summary?{}", MP_API_BASE, query_string);
        let response = self.simulate_fetch(&url).await?;

        let mut materials = Vec::new();
        for mp_material in response.data {
            match self.convert_material(&mp_material) {
                Ok(material) => materials.push(material),
                Err(e) => {
                    eprintln!(
                        "Failed to convert material {}: {}",
                        mp_material.material_id, e
                    );
                }
            }
        }

        // Apply additional filters
        if let Some(ref material_type) = query.material_type {
            materials.retain(|m| &m.material_type == material_type);
        }

        if let Some(ref name) = query.name_contains {
            let name_lower = name.to_lowercase();
            materials.retain(|m| m.identity.name.to_lowercase().contains(&name_lower));
        }

        Ok(materials)
    }

    async fn get_by_id(&self, external_id: &str) -> IntegrationResult<MaterialData> {
        if !self.connected {
            return Err(IntegrationError::ProviderNotConnected(self.name.clone()));
        }

        // Validate MP ID format
        if !external_id.starts_with("mp-") {
            return Err(IntegrationError::InvalidConfig(format!(
                "Invalid Materials Project ID format: {}",
                external_id
            )));
        }

        let url = format!("{}/materials/{}", MP_API_BASE, external_id);
        let response = self.simulate_fetch(&url).await?;

        if let Some(mp_material) = response.data.first() {
            self.convert_material(mp_material)
        } else {
            Err(IntegrationError::ExternalIdNotFound(external_id.to_string()))
        }
    }

    async fn get_by_wia_id(&self, _wia_id: &str) -> IntegrationResult<Option<MaterialData>> {
        if !self.connected {
            return Err(IntegrationError::ProviderNotConnected(self.name.clone()));
        }

        // MP doesn't track WIA IDs
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_query_params() {
        let provider = MaterialsProjectProvider::new();

        let query = ProviderQuery::new()
            .with_elements(vec!["Fe".to_string(), "O".to_string()])
            .with_limit(50);

        let params = provider.build_query_params(&query);

        assert!(params.iter().any(|(k, v)| k == "elements" && v == "Fe,O"));
        assert!(params.iter().any(|(k, v)| k == "_limit" && v == "50"));
    }

    #[tokio::test]
    async fn test_materials_project_connect() {
        let mut provider = MaterialsProjectProvider::new();

        let config = ProviderConfig::new(MP_API_BASE);
        provider.connect(config).await.unwrap();

        assert!(provider.is_connected());
    }

    #[tokio::test]
    async fn test_materials_project_search() {
        let mut provider = MaterialsProjectProvider::new();
        provider
            .connect(ProviderConfig::new(MP_API_BASE))
            .await
            .unwrap();

        let query = ProviderQuery::new().with_elements(vec!["Fe".to_string()]);

        let results = provider.search(&query).await.unwrap();
        assert!(!results.is_empty());
    }

    #[tokio::test]
    async fn test_materials_project_get_by_id() {
        let mut provider = MaterialsProjectProvider::new();
        provider
            .connect(ProviderConfig::new(MP_API_BASE))
            .await
            .unwrap();

        let material = provider.get_by_id("mp-13").await.unwrap();
        assert_eq!(material.identity.formula, "Fe");
    }

    #[tokio::test]
    async fn test_materials_project_status() {
        let mut provider = MaterialsProjectProvider::new();
        provider
            .connect(ProviderConfig::new(MP_API_BASE))
            .await
            .unwrap();

        let status = provider.status().await.unwrap();
        assert!(status.connected);
        assert!(status.available_count.unwrap() > 0);
    }
}
