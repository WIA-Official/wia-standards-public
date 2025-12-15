//! Data provider traits and types

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::integration::error::IntegrationResult;
use crate::types::{MaterialData, MaterialType};

/// Provider configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProviderConfig {
    /// Provider base URL
    pub base_url: String,

    /// API key (if required)
    pub api_key: Option<String>,

    /// Request timeout in seconds
    #[serde(default = "default_timeout")]
    pub timeout_secs: u64,

    /// Maximum results per query
    #[serde(default = "default_max_results")]
    pub max_results: usize,
}

fn default_timeout() -> u64 {
    30
}

fn default_max_results() -> usize {
    100
}

impl ProviderConfig {
    /// Create a new provider configuration
    pub fn new(base_url: impl Into<String>) -> Self {
        Self {
            base_url: base_url.into(),
            api_key: None,
            timeout_secs: default_timeout(),
            max_results: default_max_results(),
        }
    }

    /// Set API key
    pub fn with_api_key(mut self, api_key: impl Into<String>) -> Self {
        self.api_key = Some(api_key.into());
        self
    }

    /// Set timeout
    pub fn with_timeout(mut self, timeout_secs: u64) -> Self {
        self.timeout_secs = timeout_secs;
        self
    }

    /// Set max results
    pub fn with_max_results(mut self, max_results: usize) -> Self {
        self.max_results = max_results;
        self
    }
}

impl Default for ProviderConfig {
    fn default() -> Self {
        Self {
            base_url: String::new(),
            api_key: None,
            timeout_secs: default_timeout(),
            max_results: default_max_results(),
        }
    }
}

/// Query parameters for searching materials
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ProviderQuery {
    /// Filter by material type
    pub material_type: Option<MaterialType>,

    /// Filter by elements (e.g., ["Fe", "O"])
    pub elements: Option<Vec<String>>,

    /// Filter by formula
    pub formula: Option<String>,

    /// Filter by band gap range (min, max) in eV
    pub band_gap_range: Option<(f64, f64)>,

    /// Filter by name (contains)
    pub name_contains: Option<String>,

    /// Maximum results
    pub limit: Option<usize>,

    /// Pagination offset
    pub offset: Option<usize>,
}

impl ProviderQuery {
    /// Create a new empty query
    pub fn new() -> Self {
        Self::default()
    }

    /// Filter by material type
    pub fn with_type(mut self, material_type: MaterialType) -> Self {
        self.material_type = Some(material_type);
        self
    }

    /// Filter by elements
    pub fn with_elements(mut self, elements: Vec<String>) -> Self {
        self.elements = Some(elements);
        self
    }

    /// Filter by formula
    pub fn with_formula(mut self, formula: impl Into<String>) -> Self {
        self.formula = Some(formula.into());
        self
    }

    /// Filter by band gap range
    pub fn with_band_gap(mut self, min: f64, max: f64) -> Self {
        self.band_gap_range = Some((min, max));
        self
    }

    /// Filter by name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name_contains = Some(name.into());
        self
    }

    /// Set limit
    pub fn with_limit(mut self, limit: usize) -> Self {
        self.limit = Some(limit);
        self
    }

    /// Set offset
    pub fn with_offset(mut self, offset: usize) -> Self {
        self.offset = Some(offset);
        self
    }
}

/// Provider status information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProviderStatus {
    /// Provider name
    pub name: String,

    /// Connection status
    pub connected: bool,

    /// Provider version
    pub version: String,

    /// Total available materials count
    pub available_count: Option<usize>,

    /// Last successful connection time
    pub last_connected: Option<String>,
}

/// Trait for external data providers
#[async_trait]
pub trait DataProvider: Send + Sync {
    /// Get provider name
    fn name(&self) -> &str;

    /// Connect to the provider
    async fn connect(&mut self, config: ProviderConfig) -> IntegrationResult<()>;

    /// Disconnect from the provider
    async fn disconnect(&mut self) -> IntegrationResult<()>;

    /// Check connection status
    fn is_connected(&self) -> bool;

    /// Get provider status
    async fn status(&self) -> IntegrationResult<ProviderStatus>;

    /// Search for materials
    async fn search(&self, query: &ProviderQuery) -> IntegrationResult<Vec<MaterialData>>;

    /// Get material by external ID
    async fn get_by_id(&self, external_id: &str) -> IntegrationResult<MaterialData>;

    /// Get material by WIA material ID
    async fn get_by_wia_id(&self, wia_id: &str) -> IntegrationResult<Option<MaterialData>>;
}
