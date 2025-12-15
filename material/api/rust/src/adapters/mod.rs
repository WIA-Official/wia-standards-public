//! Adapters module for WIA Material SDK
//!
//! This module contains adapters for different material data sources.

pub mod simulator;

use crate::error::MaterialResult;
use crate::types::MaterialData;
use async_trait::async_trait;

/// Trait for material data adapters
#[async_trait]
pub trait MaterialAdapter: Send + Sync {
    /// Get the adapter name
    fn name(&self) -> &str;

    /// Connect to the data source
    async fn connect(&mut self) -> MaterialResult<()>;

    /// Disconnect from the data source
    async fn disconnect(&mut self) -> MaterialResult<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Fetch material data by ID
    async fn fetch(&self, id: &str) -> MaterialResult<MaterialData>;

    /// Store material data
    async fn store(&self, data: &MaterialData) -> MaterialResult<()>;

    /// List all material IDs
    async fn list(&self) -> MaterialResult<Vec<String>>;

    /// Search materials by criteria
    async fn search(&self, query: &MaterialQuery) -> MaterialResult<Vec<MaterialData>>;
}

/// Query criteria for searching materials
#[derive(Debug, Clone, Default)]
pub struct MaterialQuery {
    /// Filter by material type
    pub material_type: Option<crate::types::MaterialType>,
    /// Filter by name (partial match)
    pub name_contains: Option<String>,
    /// Filter by formula
    pub formula: Option<String>,
    /// Filter by classification tag
    pub classification: Option<String>,
    /// Minimum confidence threshold
    pub min_confidence: Option<f64>,
    /// Limit number of results
    pub limit: Option<usize>,
    /// Offset for pagination
    pub offset: Option<usize>,
}

impl MaterialQuery {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_type(mut self, material_type: crate::types::MaterialType) -> Self {
        self.material_type = Some(material_type);
        self
    }

    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name_contains = Some(name.into());
        self
    }

    pub fn with_formula(mut self, formula: impl Into<String>) -> Self {
        self.formula = Some(formula.into());
        self
    }

    pub fn with_classification(mut self, tag: impl Into<String>) -> Self {
        self.classification = Some(tag.into());
        self
    }

    pub fn with_min_confidence(mut self, confidence: f64) -> Self {
        self.min_confidence = Some(confidence);
        self
    }

    pub fn with_limit(mut self, limit: usize) -> Self {
        self.limit = Some(limit);
        self
    }

    pub fn with_offset(mut self, offset: usize) -> Self {
        self.offset = Some(offset);
        self
    }
}

pub use simulator::SimulatorAdapter;
