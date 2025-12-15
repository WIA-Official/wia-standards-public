//! Material core functionality
//!
//! This module provides the main WiaMaterial client and material operations.

use crate::adapters::MaterialAdapter;
use crate::error::{MaterialError, MaterialResult};
use crate::types::*;
use async_trait::async_trait;
use chrono::Utc;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

/// Configuration for WiaMaterial client
#[derive(Debug, Clone)]
pub struct MaterialConfig {
    /// Schema URL for validation
    pub schema_url: String,
    /// Version string
    pub version: String,
    /// Enable strict validation
    pub strict_validation: bool,
    /// Default confidence threshold
    pub default_confidence: f64,
}

impl Default for MaterialConfig {
    fn default() -> Self {
        Self {
            schema_url: "https://wia.live/material/v1/schema.json".to_string(),
            version: "1.0.0".to_string(),
            strict_validation: true,
            default_confidence: 0.95,
        }
    }
}

/// Builder for MaterialData
#[derive(Debug, Default)]
pub struct MaterialBuilder {
    material_type: Option<MaterialType>,
    name: Option<String>,
    formula: Option<String>,
    classification: Option<Vec<String>>,
    structure: Option<Structure>,
    properties: Properties,
    measurement: Option<Measurement>,
    provenance: Option<Provenance>,
    external_references: Option<ExternalReferences>,
    confidence: Option<f64>,
    notes: Option<String>,
}

impl MaterialBuilder {
    /// Create a new MaterialBuilder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set material type
    pub fn material_type(mut self, material_type: MaterialType) -> Self {
        self.material_type = Some(material_type);
        self
    }

    /// Set material name
    pub fn name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set chemical formula
    pub fn formula(mut self, formula: impl Into<String>) -> Self {
        self.formula = Some(formula.into());
        self
    }

    /// Set classification tags
    pub fn classification(mut self, tags: Vec<String>) -> Self {
        self.classification = Some(tags);
        self
    }

    /// Set crystal structure
    pub fn structure(mut self, structure: Structure) -> Self {
        self.structure = Some(structure);
        self
    }

    /// Set electrical properties
    pub fn electrical_properties(mut self, props: ElectricalProperties) -> Self {
        self.properties.electrical = Some(props);
        self
    }

    /// Set magnetic properties
    pub fn magnetic_properties(mut self, props: MagneticProperties) -> Self {
        self.properties.magnetic = Some(props);
        self
    }

    /// Set thermal properties
    pub fn thermal_properties(mut self, props: ThermalProperties) -> Self {
        self.properties.thermal = Some(props);
        self
    }

    /// Set domain-specific properties (JSON value)
    pub fn domain_specific(mut self, props: serde_json::Value) -> Self {
        self.properties.domain_specific = Some(props);
        self
    }

    /// Set measurement conditions
    pub fn measurement(mut self, measurement: Measurement) -> Self {
        self.measurement = Some(measurement);
        self
    }

    /// Set provenance information
    pub fn provenance(mut self, provenance: Provenance) -> Self {
        self.provenance = Some(provenance);
        self
    }

    /// Set confidence level
    pub fn confidence(mut self, confidence: f64) -> Self {
        self.confidence = Some(confidence);
        self
    }

    /// Set notes
    pub fn notes(mut self, notes: impl Into<String>) -> Self {
        self.notes = Some(notes.into());
        self
    }

    /// Build the MaterialData
    pub fn build(self) -> MaterialResult<MaterialData> {
        let material_type = self
            .material_type
            .ok_or_else(|| MaterialError::MissingField("material_type".to_string()))?;

        let name = self
            .name
            .ok_or_else(|| MaterialError::MissingField("name".to_string()))?;

        let formula = self
            .formula
            .ok_or_else(|| MaterialError::MissingField("formula".to_string()))?;

        // Generate unique material ID
        let uuid_part = Uuid::new_v4().to_string().replace("-", "");
        let material_id = format!("wia-mat-{}", &uuid_part[..8]);

        let meta = if self.confidence.is_some() || self.notes.is_some() {
            Some(Meta {
                confidence: self.confidence,
                validated: Some(false),
                notes: self.notes,
            })
        } else {
            None
        };

        Ok(MaterialData {
            schema: Some("https://wia.live/material/v1/schema.json".to_string()),
            version: "1.0.0".to_string(),
            material_type,
            material_id,
            timestamp: Timestamp::default(),
            identity: Identity {
                name,
                formula,
                classification: self.classification,
            },
            structure: self.structure,
            properties: self.properties,
            measurement: self.measurement,
            provenance: self.provenance,
            external_references: self.external_references,
            meta,
        })
    }
}

/// Event types for material operations
#[derive(Debug, Clone)]
pub enum MaterialEvent {
    /// Material data created
    Created(String),
    /// Material data updated
    Updated(String),
    /// Material data validated
    Validated(String, bool),
    /// Adapter connected
    AdapterConnected(String),
    /// Adapter disconnected
    AdapterDisconnected(String),
    /// Error occurred
    Error(String),
}

/// Event handler trait
#[async_trait]
pub trait MaterialEventHandler: Send + Sync {
    async fn on_event(&self, event: MaterialEvent);
}

/// Main WiaMaterial client
pub struct WiaMaterial {
    config: MaterialConfig,
    adapter: Option<Arc<RwLock<Box<dyn MaterialAdapter>>>>,
    materials: Arc<RwLock<Vec<MaterialData>>>,
    event_handlers: Arc<RwLock<Vec<Box<dyn MaterialEventHandler>>>>,
}

impl WiaMaterial {
    /// Create a new WiaMaterial client with default configuration
    pub fn new() -> Self {
        Self::with_config(MaterialConfig::default())
    }

    /// Create a new WiaMaterial client with custom configuration
    pub fn with_config(config: MaterialConfig) -> Self {
        Self {
            config,
            adapter: None,
            materials: Arc::new(RwLock::new(Vec::new())),
            event_handlers: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Get the current configuration
    pub fn config(&self) -> &MaterialConfig {
        &self.config
    }

    /// Set the adapter
    pub async fn set_adapter(&mut self, adapter: Box<dyn MaterialAdapter>) {
        self.adapter = Some(Arc::new(RwLock::new(adapter)));
    }

    /// Add an event handler
    pub async fn add_event_handler(&self, handler: Box<dyn MaterialEventHandler>) {
        let mut handlers = self.event_handlers.write().await;
        handlers.push(handler);
    }

    /// Emit an event to all handlers
    async fn emit_event(&self, event: MaterialEvent) {
        let handlers = self.event_handlers.read().await;
        for handler in handlers.iter() {
            handler.on_event(event.clone()).await;
        }
    }

    /// Create a new material data entry
    pub async fn create_material(&self, data: MaterialData) -> MaterialResult<MaterialData> {
        // Validate the data
        self.validate(&data)?;

        // Store the material
        {
            let mut materials = self.materials.write().await;
            materials.push(data.clone());
        }

        // Emit event
        self.emit_event(MaterialEvent::Created(data.material_id.clone()))
            .await;

        Ok(data)
    }

    /// Get all stored materials
    pub async fn get_materials(&self) -> Vec<MaterialData> {
        let materials = self.materials.read().await;
        materials.clone()
    }

    /// Get material by ID
    pub async fn get_material(&self, id: &str) -> MaterialResult<MaterialData> {
        let materials = self.materials.read().await;
        materials
            .iter()
            .find(|m| m.material_id == id)
            .cloned()
            .ok_or_else(|| MaterialError::NotFound(id.to_string()))
    }

    /// Get materials by type
    pub async fn get_materials_by_type(&self, material_type: MaterialType) -> Vec<MaterialData> {
        let materials = self.materials.read().await;
        materials
            .iter()
            .filter(|m| m.material_type == material_type)
            .cloned()
            .collect()
    }

    /// Validate material data
    pub fn validate(&self, data: &MaterialData) -> MaterialResult<()> {
        // Validate material ID format
        if !MATERIAL_ID_REGEX.is_match(&data.material_id) {
            return Err(MaterialError::InvalidMaterialId(data.material_id.clone()));
        }

        // Validate identity
        if data.identity.name.is_empty() {
            return Err(MaterialError::validation("name", "cannot be empty"));
        }

        if data.identity.formula.is_empty() {
            return Err(MaterialError::validation("formula", "cannot be empty"));
        }

        // Validate meta if present
        if let Some(ref meta) = data.meta {
            if let Some(confidence) = meta.confidence {
                if confidence < 0.0 || confidence > 1.0 {
                    return Err(MaterialError::out_of_range("confidence", confidence, 0.0, 1.0));
                }
            }
        }

        // Validate measurement conditions if present
        if let Some(ref measurement) = data.measurement {
            if let Some(temp) = measurement.temperature_k {
                if temp < 0.0 {
                    return Err(MaterialError::out_of_range(
                        "temperature_k",
                        temp,
                        0.0,
                        f64::MAX,
                    ));
                }
            }
            if let Some(pressure) = measurement.pressure_pa {
                if pressure < 0.0 {
                    return Err(MaterialError::out_of_range(
                        "pressure_pa",
                        pressure,
                        0.0,
                        f64::MAX,
                    ));
                }
            }
        }

        // Emit validation event
        // Note: Can't await in sync function, validation is synchronous

        Ok(())
    }

    /// Parse material data from JSON string
    pub fn parse_json(&self, json: &str) -> MaterialResult<MaterialData> {
        let data: MaterialData = serde_json::from_str(json)?;
        if self.config.strict_validation {
            self.validate(&data)?;
        }
        Ok(data)
    }

    /// Convert material data to JSON string
    pub fn to_json(&self, data: &MaterialData) -> MaterialResult<String> {
        Ok(serde_json::to_string_pretty(data)?)
    }

    /// Create a superconductor material
    pub async fn create_superconductor(
        &self,
        name: &str,
        formula: &str,
        props: SuperconductorProperties,
    ) -> MaterialResult<MaterialData> {
        let domain_specific = serde_json::to_value(&props)?;

        let data = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name(name)
            .formula(formula)
            .domain_specific(domain_specific)
            .build()?;

        self.create_material(data).await
    }

    /// Create a topological insulator material
    pub async fn create_topological_insulator(
        &self,
        name: &str,
        formula: &str,
        props: TopologicalInsulatorProperties,
    ) -> MaterialResult<MaterialData> {
        let domain_specific = serde_json::to_value(&props)?;

        let data = MaterialBuilder::new()
            .material_type(MaterialType::TopologicalInsulator)
            .name(name)
            .formula(formula)
            .domain_specific(domain_specific)
            .build()?;

        self.create_material(data).await
    }

    /// Create a memristor material
    pub async fn create_memristor(
        &self,
        name: &str,
        formula: &str,
        props: MemristorProperties,
    ) -> MaterialResult<MaterialData> {
        let domain_specific = serde_json::to_value(&props)?;

        let data = MaterialBuilder::new()
            .material_type(MaterialType::Memristor)
            .name(name)
            .formula(formula)
            .domain_specific(domain_specific)
            .build()?;

        self.create_material(data).await
    }

    /// Create a metamaterial
    pub async fn create_metamaterial(
        &self,
        name: &str,
        props: MetamaterialProperties,
    ) -> MaterialResult<MaterialData> {
        let domain_specific = serde_json::to_value(&props)?;

        let data = MaterialBuilder::new()
            .material_type(MaterialType::Metamaterial)
            .name(name)
            .formula("composite")
            .domain_specific(domain_specific)
            .build()?;

        self.create_material(data).await
    }

    /// Update material timestamp
    pub async fn update_material(&self, id: &str) -> MaterialResult<MaterialData> {
        let mut materials = self.materials.write().await;

        let material = materials
            .iter_mut()
            .find(|m| m.material_id == id)
            .ok_or_else(|| MaterialError::NotFound(id.to_string()))?;

        material.timestamp.modified = Some(Utc::now());

        let updated = material.clone();

        drop(materials);

        self.emit_event(MaterialEvent::Updated(id.to_string())).await;

        Ok(updated)
    }

    /// Get statistics about stored materials
    pub async fn get_statistics(&self) -> MaterialStatistics {
        let materials = self.materials.read().await;

        let mut stats = MaterialStatistics::default();
        stats.total_count = materials.len();

        for material in materials.iter() {
            match material.material_type {
                MaterialType::Superconductor => stats.superconductor_count += 1,
                MaterialType::Metamaterial => stats.metamaterial_count += 1,
                MaterialType::Memristor => stats.memristor_count += 1,
                MaterialType::TopologicalInsulator => stats.topological_insulator_count += 1,
                MaterialType::HolographicStorage => stats.holographic_storage_count += 1,
                MaterialType::ProgrammableMatter => stats.programmable_matter_count += 1,
                MaterialType::Custom => stats.custom_count += 1,
            }
        }

        stats
    }
}

impl Default for WiaMaterial {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about stored materials
#[derive(Debug, Clone, Default)]
pub struct MaterialStatistics {
    pub total_count: usize,
    pub superconductor_count: usize,
    pub metamaterial_count: usize,
    pub memristor_count: usize,
    pub topological_insulator_count: usize,
    pub holographic_storage_count: usize,
    pub programmable_matter_count: usize,
    pub custom_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_create_material() {
        let client = WiaMaterial::new();

        let data = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name("YBCO")
            .formula("YBa2Cu3O7-x")
            .build()
            .unwrap();

        let result = client.create_material(data).await;
        assert!(result.is_ok());

        let materials = client.get_materials().await;
        assert_eq!(materials.len(), 1);
    }

    #[tokio::test]
    async fn test_get_material_by_type() {
        let client = WiaMaterial::new();

        // Create superconductor
        let sc = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name("YBCO")
            .formula("YBa2Cu3O7-x")
            .build()
            .unwrap();
        client.create_material(sc).await.unwrap();

        // Create topological insulator
        let ti = MaterialBuilder::new()
            .material_type(MaterialType::TopologicalInsulator)
            .name("Bi2Se3")
            .formula("Bi2Se3")
            .build()
            .unwrap();
        client.create_material(ti).await.unwrap();

        let superconductors = client
            .get_materials_by_type(MaterialType::Superconductor)
            .await;
        assert_eq!(superconductors.len(), 1);

        let tis = client
            .get_materials_by_type(MaterialType::TopologicalInsulator)
            .await;
        assert_eq!(tis.len(), 1);
    }

    #[test]
    fn test_material_builder() {
        let result = MaterialBuilder::new()
            .material_type(MaterialType::Memristor)
            .name("TiO2 Memristor")
            .formula("TiO2")
            .confidence(0.95)
            .notes("Test memristor")
            .build();

        assert!(result.is_ok());
        let data = result.unwrap();
        assert_eq!(data.identity.name, "TiO2 Memristor");
        assert!(data.material_id.starts_with("wia-mat-"));
    }

    #[test]
    fn test_validation() {
        let client = WiaMaterial::new();

        // Valid data
        let valid = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name("Test")
            .formula("Test")
            .build()
            .unwrap();
        assert!(client.validate(&valid).is_ok());
    }
}
