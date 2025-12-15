//! OPTIMADE provider implementation
//!
//! Provides access to materials databases through the OPTIMADE API.
//! https://www.optimade.org

use async_trait::async_trait;
use chrono::Utc;
use serde::{Deserialize, Serialize};

use super::traits::{DataProvider, ProviderConfig, ProviderQuery, ProviderStatus};
use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::{
    CrystalSystem, ExternalReferences, Identity, LatticeParameters, MaterialData, MaterialType,
    Meta, Properties, Provenance, Structure, Timestamp,
};

/// Default OPTIMADE providers
pub const OPTIMADE_PROVIDERS: &[(&str, &str)] = &[
    ("materialsproject", "https://optimade.materialsproject.org"),
    ("aflow", "https://aflow.org/API/optimade"),
    ("oqmd", "https://oqmd.org/optimade"),
    ("cod", "https://www.crystallography.net/cod/optimade"),
    ("nomad", "https://nomad-lab.eu/prod/v1/api/optimade"),
];

/// OPTIMADE structure response (simplified)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimadeStructure {
    pub id: String,
    #[serde(rename = "type")]
    pub type_: String,
    pub attributes: OptimadeAttributes,
}

/// OPTIMADE structure attributes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimadeAttributes {
    pub chemical_formula_reduced: Option<String>,
    pub chemical_formula_descriptive: Option<String>,
    pub elements: Option<Vec<String>>,
    pub nelements: Option<i32>,
    pub nsites: Option<i32>,
    pub lattice_vectors: Option<Vec<Vec<f64>>>,
    pub cartesian_site_positions: Option<Vec<Vec<f64>>>,
    pub species_at_sites: Option<Vec<String>>,
    pub dimension_types: Option<Vec<i32>>,
}

/// OPTIMADE response wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimadeResponse {
    pub data: Vec<OptimadeStructure>,
    pub meta: OptimadeMeta,
}

/// OPTIMADE single response wrapper
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimadeSingleResponse {
    pub data: OptimadeStructure,
    pub meta: OptimadeMeta,
}

/// OPTIMADE meta information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimadeMeta {
    pub api_version: String,
    pub query: Option<OptimadeQuery>,
    pub data_returned: Option<i32>,
    pub data_available: Option<i32>,
}

/// OPTIMADE query info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimadeQuery {
    pub representation: Option<String>,
}

/// OPTIMADE provider implementation
pub struct OptimadeProvider {
    name: String,
    config: Option<ProviderConfig>,
    connected: bool,
    base_url: String,
    api_version: String,
}

impl OptimadeProvider {
    /// Create a new OPTIMADE provider
    pub fn new() -> Self {
        Self {
            name: "optimade".to_string(),
            config: None,
            connected: false,
            base_url: "https://providers.optimade.org".to_string(),
            api_version: "1.2.0".to_string(),
        }
    }

    /// Create with a specific provider URL
    pub fn with_provider(provider_url: &str) -> Self {
        Self {
            name: "optimade".to_string(),
            config: None,
            connected: false,
            base_url: provider_url.to_string(),
            api_version: "1.2.0".to_string(),
        }
    }

    /// Build OPTIMADE filter string from query
    fn build_filter(&self, query: &ProviderQuery) -> String {
        let mut filters = Vec::new();

        // Elements filter
        if let Some(ref elements) = query.elements {
            let elements_str = elements
                .iter()
                .map(|e| format!("\"{}\"", e))
                .collect::<Vec<_>>()
                .join(", ");
            filters.push(format!("elements HAS ALL {}", elements_str));
        }

        // Formula filter
        if let Some(ref formula) = query.formula {
            filters.push(format!("chemical_formula_reduced = \"{}\"", formula));
        }

        // Band gap filter
        if let Some((min, max)) = query.band_gap_range {
            filters.push(format!("band_gap >= {} AND band_gap <= {}", min, max));
        }

        if filters.is_empty() {
            String::new()
        } else {
            filters.join(" AND ")
        }
    }

    /// Build query URL with pagination
    fn build_url(&self, query: &ProviderQuery) -> String {
        let filter = self.build_filter(query);
        let limit = query.limit.unwrap_or(100);
        let offset = query.offset.unwrap_or(0);

        let mut url = format!("{}/v1/structures?page_limit={}", self.base_url, limit);

        if offset > 0 {
            url.push_str(&format!("&page_offset={}", offset));
        }

        if !filter.is_empty() {
            url.push_str(&format!("&filter={}", urlencoding::encode(&filter)));
        }

        url
    }

    /// Convert OPTIMADE structure to WIA MaterialData
    fn convert_structure(&self, optimade: &OptimadeStructure) -> IntegrationResult<MaterialData> {
        let now = Utc::now();
        let attrs = &optimade.attributes;

        // Extract formula
        let formula = attrs
            .chemical_formula_reduced
            .clone()
            .or_else(|| attrs.chemical_formula_descriptive.clone())
            .unwrap_or_else(|| "Unknown".to_string());

        // Build structure from lattice vectors
        let structure = if let Some(ref vectors) = attrs.lattice_vectors {
            Some(self.convert_lattice(vectors)?)
        } else {
            None
        };

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
                name: formula.clone(),
                formula,
                classification: Some(vec!["optimade_import".to_string()]),
            },
            structure,
            properties: Properties::default(),
            measurement: None,
            provenance: Some(Provenance {
                source: Some(format!("OPTIMADE: {}", self.base_url)),
                lab: None,
                operator: None,
            }),
            external_references: Some(ExternalReferences {
                materials_project_id: None,
                icsd_id: None,
                cod_id: None,
                doi: None,
            }),
            meta: Some(Meta {
                confidence: Some(0.9),
                validated: None,
                notes: Some(format!("Imported from OPTIMADE: {}", optimade.id)),
            }),
        })
    }

    /// Convert OPTIMADE lattice vectors to WIA structure
    fn convert_lattice(&self, vectors: &[Vec<f64>]) -> IntegrationResult<Structure> {
        if vectors.len() != 3 {
            return Err(IntegrationError::ConversionError(
                "Invalid lattice vectors: expected 3 vectors".to_string(),
            ));
        }

        // Calculate lattice parameters from vectors
        let a_vec = &vectors[0];
        let b_vec = &vectors[1];
        let c_vec = &vectors[2];

        let a = vector_length(a_vec);
        let b = vector_length(b_vec);
        let c = vector_length(c_vec);

        let alpha = angle_between(b_vec, c_vec);
        let beta = angle_between(a_vec, c_vec);
        let gamma = angle_between(a_vec, b_vec);

        let lattice_parameters = LatticeParameters {
            a_angstrom: Some(a),
            b_angstrom: Some(b),
            c_angstrom: Some(c),
            alpha_degree: Some(alpha),
            beta_degree: Some(beta),
            gamma_degree: Some(gamma),
        };

        Ok(Structure {
            crystal_system: Some(determine_crystal_system(a, b, c, alpha, beta, gamma)),
            space_group: None,
            lattice_parameters: Some(lattice_parameters),
        })
    }

    /// Simulate fetching from OPTIMADE (for development/testing)
    async fn simulate_fetch(&self, _url: &str) -> IntegrationResult<OptimadeResponse> {
        // In production, this would make actual HTTP requests
        // For now, return simulated data
        Ok(OptimadeResponse {
            data: vec![
                OptimadeStructure {
                    id: "optimade/1".to_string(),
                    type_: "structures".to_string(),
                    attributes: OptimadeAttributes {
                        chemical_formula_reduced: Some("Fe2O3".to_string()),
                        chemical_formula_descriptive: Some("Iron(III) oxide".to_string()),
                        elements: Some(vec!["Fe".to_string(), "O".to_string()]),
                        nelements: Some(2),
                        nsites: Some(5),
                        lattice_vectors: Some(vec![
                            vec![5.0, 0.0, 0.0],
                            vec![0.0, 5.0, 0.0],
                            vec![0.0, 0.0, 13.7],
                        ]),
                        cartesian_site_positions: None,
                        species_at_sites: None,
                        dimension_types: Some(vec![1, 1, 1]),
                    },
                },
                OptimadeStructure {
                    id: "optimade/2".to_string(),
                    type_: "structures".to_string(),
                    attributes: OptimadeAttributes {
                        chemical_formula_reduced: Some("TiO2".to_string()),
                        chemical_formula_descriptive: Some("Titanium dioxide".to_string()),
                        elements: Some(vec!["Ti".to_string(), "O".to_string()]),
                        nelements: Some(2),
                        nsites: Some(6),
                        lattice_vectors: Some(vec![
                            vec![4.594, 0.0, 0.0],
                            vec![0.0, 4.594, 0.0],
                            vec![0.0, 0.0, 2.958],
                        ]),
                        cartesian_site_positions: None,
                        species_at_sites: None,
                        dimension_types: Some(vec![1, 1, 1]),
                    },
                },
            ],
            meta: OptimadeMeta {
                api_version: "1.2.0".to_string(),
                query: None,
                data_returned: Some(2),
                data_available: Some(1000),
            },
        })
    }
}

impl Default for OptimadeProvider {
    fn default() -> Self {
        Self::new()
    }
}

// Helper functions

fn generate_id() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

fn vector_length(v: &[f64]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn dot_product(a: &[f64], b: &[f64]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn angle_between(a: &[f64], b: &[f64]) -> f64 {
    let cos_angle = dot_product(a, b) / (vector_length(a) * vector_length(b));
    cos_angle.clamp(-1.0, 1.0).acos().to_degrees()
}

fn determine_crystal_system(
    a: f64,
    b: f64,
    c: f64,
    alpha: f64,
    beta: f64,
    gamma: f64,
) -> CrystalSystem {
    const TOLERANCE: f64 = 0.5; // degrees

    let angles_90 = |angle: f64| (angle - 90.0).abs() < TOLERANCE;
    let angles_120 = |angle: f64| (angle - 120.0).abs() < TOLERANCE;
    let lengths_equal = |l1: f64, l2: f64| (l1 - l2).abs() / l1 < 0.01;

    if lengths_equal(a, b) && lengths_equal(b, c) {
        if angles_90(alpha) && angles_90(beta) && angles_90(gamma) {
            CrystalSystem::Cubic
        } else if (alpha - beta).abs() < TOLERANCE && (beta - gamma).abs() < TOLERANCE {
            CrystalSystem::Trigonal
        } else {
            CrystalSystem::Triclinic
        }
    } else if lengths_equal(a, b) {
        if angles_90(alpha) && angles_90(beta) && angles_120(gamma) {
            CrystalSystem::Hexagonal
        } else if angles_90(alpha) && angles_90(beta) && angles_90(gamma) {
            CrystalSystem::Tetragonal
        } else {
            CrystalSystem::Triclinic
        }
    } else if angles_90(alpha) && angles_90(beta) && angles_90(gamma) {
        if lengths_equal(a, b) || lengths_equal(b, c) || lengths_equal(a, c) {
            CrystalSystem::Tetragonal
        } else {
            CrystalSystem::Orthorhombic
        }
    } else if angles_90(alpha) && angles_90(gamma) && !angles_90(beta) {
        CrystalSystem::Monoclinic
    } else {
        CrystalSystem::Triclinic
    }
}

#[async_trait]
impl DataProvider for OptimadeProvider {
    fn name(&self) -> &str {
        &self.name
    }

    async fn connect(&mut self, config: ProviderConfig) -> IntegrationResult<()> {
        self.base_url = config.base_url.clone();
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
            version: self.api_version.clone(),
            available_count: None,
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

        let url = self.build_url(query);
        let response = self.simulate_fetch(&url).await?;

        let mut materials = Vec::new();
        for structure in response.data {
            match self.convert_structure(&structure) {
                Ok(material) => materials.push(material),
                Err(e) => {
                    // Log error but continue with other structures
                    eprintln!("Failed to convert structure {}: {}", structure.id, e);
                }
            }
        }

        // Apply additional filters that OPTIMADE doesn't support
        if let Some(ref material_type) = query.material_type {
            materials.retain(|m| &m.material_type == material_type);
        }

        Ok(materials)
    }

    async fn get_by_id(&self, external_id: &str) -> IntegrationResult<MaterialData> {
        if !self.connected {
            return Err(IntegrationError::ProviderNotConnected(self.name.clone()));
        }

        // Simulate fetching single structure
        let response = self
            .simulate_fetch(&format!(
                "{}/v1/structures/{}",
                self.base_url, external_id
            ))
            .await?;

        if let Some(structure) = response.data.first() {
            self.convert_structure(structure)
        } else {
            Err(IntegrationError::ExternalIdNotFound(external_id.to_string()))
        }
    }

    async fn get_by_wia_id(&self, _wia_id: &str) -> IntegrationResult<Option<MaterialData>> {
        if !self.connected {
            return Err(IntegrationError::ProviderNotConnected(self.name.clone()));
        }

        // OPTIMADE doesn't track WIA IDs
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_filter() {
        let provider = OptimadeProvider::new();

        let query =
            ProviderQuery::new().with_elements(vec!["Fe".to_string(), "O".to_string()]);

        let filter = provider.build_filter(&query);
        assert!(filter.contains("elements HAS ALL"));
        assert!(filter.contains("Fe"));
        assert!(filter.contains("O"));
    }

    #[test]
    fn test_build_filter_formula() {
        let provider = OptimadeProvider::new();

        let query = ProviderQuery::new().with_formula("Fe2O3");

        let filter = provider.build_filter(&query);
        assert!(filter.contains("chemical_formula_reduced"));
        assert!(filter.contains("Fe2O3"));
    }

    #[tokio::test]
    async fn test_optimade_provider_connect() {
        let mut provider = OptimadeProvider::new();

        let config = ProviderConfig::new("https://test.optimade.org");
        provider.connect(config).await.unwrap();

        assert!(provider.is_connected());
    }

    #[tokio::test]
    async fn test_optimade_provider_search() {
        let mut provider = OptimadeProvider::new();
        provider
            .connect(ProviderConfig::new("https://test.optimade.org"))
            .await
            .unwrap();

        let query =
            ProviderQuery::new().with_elements(vec!["Fe".to_string(), "O".to_string()]);

        let results = provider.search(&query).await.unwrap();
        assert!(!results.is_empty());
    }

    #[test]
    fn test_vector_length() {
        assert!((vector_length(&[3.0, 4.0, 0.0]) - 5.0).abs() < 0.001);
        assert!((vector_length(&[1.0, 1.0, 1.0]) - 1.732).abs() < 0.01);
    }

    #[test]
    fn test_determine_crystal_system() {
        // Cubic
        assert_eq!(
            determine_crystal_system(5.0, 5.0, 5.0, 90.0, 90.0, 90.0),
            CrystalSystem::Cubic
        );

        // Tetragonal
        assert_eq!(
            determine_crystal_system(5.0, 5.0, 7.0, 90.0, 90.0, 90.0),
            CrystalSystem::Tetragonal
        );

        // Orthorhombic
        assert_eq!(
            determine_crystal_system(5.0, 6.0, 7.0, 90.0, 90.0, 90.0),
            CrystalSystem::Orthorhombic
        );
    }
}
