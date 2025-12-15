//! Integration Manager
//!
//! Central manager for coordinating all integration adapters

use chrono::Utc;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use super::adapter::*;
use super::dashboard::DashboardAdapter;
use super::export::ExportFormatAdapter;
use super::fhir::FhirAdapter;
use super::wearable::{MockHealthConnectAdapter, MockHealthKitAdapter};
use crate::error::{HealthError, Result};
use crate::types::HealthProfile;

/// Integration manager configuration
#[derive(Debug, Clone, Default)]
pub struct IntegrationConfig {
    /// Enable FHIR integration
    pub enable_fhir: bool,
    /// Enable HealthKit integration
    pub enable_healthkit: bool,
    /// Enable Health Connect integration
    pub enable_health_connect: bool,
    /// Enable dashboard integration
    pub enable_dashboard: bool,
    /// Adapter-specific configs
    pub adapter_configs: HashMap<AdapterType, AdapterConfig>,
}

impl IntegrationConfig {
    /// Create with FHIR enabled
    pub fn with_fhir(mut self, url: &str, token: Option<&str>) -> Self {
        self.enable_fhir = true;
        let mut config = AdapterConfig::new().with_url(url);
        if let Some(t) = token {
            config = config.with_auth(t);
        }
        self.adapter_configs.insert(AdapterType::Fhir, config);
        self
    }

    /// Enable HealthKit
    pub fn with_healthkit(mut self) -> Self {
        self.enable_healthkit = true;
        self.adapter_configs
            .insert(AdapterType::HealthKit, AdapterConfig::default());
        self
    }

    /// Enable Health Connect
    pub fn with_health_connect(mut self) -> Self {
        self.enable_health_connect = true;
        self.adapter_configs
            .insert(AdapterType::HealthConnect, AdapterConfig::default());
        self
    }

    /// Enable dashboard
    pub fn with_dashboard(mut self, url: &str) -> Self {
        self.enable_dashboard = true;
        self.adapter_configs
            .insert(AdapterType::Dashboard, AdapterConfig::new().with_url(url));
        self
    }
}

/// Central integration manager
pub struct IntegrationManager {
    config: IntegrationConfig,
    adapters: HashMap<AdapterType, Arc<RwLock<Box<dyn IntegrationAdapter>>>>,
    import_adapters: HashMap<AdapterType, Arc<RwLock<Box<dyn ImportAdapter>>>>,
    export_adapters: HashMap<AdapterType, Arc<RwLock<Box<dyn ExportAdapter>>>>,
    initialized: bool,
}

impl IntegrationManager {
    /// Create new integration manager
    pub fn new(config: IntegrationConfig) -> Self {
        Self {
            config,
            adapters: HashMap::new(),
            import_adapters: HashMap::new(),
            export_adapters: HashMap::new(),
            initialized: false,
        }
    }

    /// Create with default configuration
    pub fn default_config() -> Self {
        Self::new(IntegrationConfig::default())
    }

    /// Initialize all configured adapters
    pub async fn initialize(&mut self) -> Result<()> {
        // Initialize FHIR adapter
        if self.config.enable_fhir {
            let mut adapter = FhirAdapter::new();
            if let Some(config) = self.config.adapter_configs.get(&AdapterType::Fhir) {
                adapter.initialize(config.clone()).await?;
            }
            self.export_adapters.insert(
                AdapterType::Fhir,
                Arc::new(RwLock::new(Box::new(adapter))),
            );
        }

        // Initialize HealthKit adapter
        if self.config.enable_healthkit {
            let mut adapter = MockHealthKitAdapter::new();
            if let Some(config) = self.config.adapter_configs.get(&AdapterType::HealthKit) {
                adapter.initialize(config.clone()).await?;
            } else {
                adapter.initialize(AdapterConfig::default()).await?;
            }
            self.import_adapters.insert(
                AdapterType::HealthKit,
                Arc::new(RwLock::new(Box::new(adapter))),
            );
        }

        // Initialize Health Connect adapter
        if self.config.enable_health_connect {
            let mut adapter = MockHealthConnectAdapter::new();
            if let Some(config) = self.config.adapter_configs.get(&AdapterType::HealthConnect) {
                adapter.initialize(config.clone()).await?;
            } else {
                adapter.initialize(AdapterConfig::default()).await?;
            }
            self.import_adapters.insert(
                AdapterType::HealthConnect,
                Arc::new(RwLock::new(Box::new(adapter))),
            );
        }

        // Initialize Dashboard adapter
        if self.config.enable_dashboard {
            let mut adapter = DashboardAdapter::new();
            if let Some(config) = self.config.adapter_configs.get(&AdapterType::Dashboard) {
                adapter.initialize(config.clone()).await?;
            }
            self.export_adapters.insert(
                AdapterType::Dashboard,
                Arc::new(RwLock::new(Box::new(adapter))),
            );
        }

        // Always initialize export format adapter
        let mut export_adapter = ExportFormatAdapter::new();
        export_adapter.initialize(AdapterConfig::default()).await?;
        self.export_adapters.insert(
            AdapterType::Export,
            Arc::new(RwLock::new(Box::new(export_adapter))),
        );

        self.initialized = true;
        Ok(())
    }

    /// Shutdown all adapters
    pub async fn shutdown(&mut self) -> Result<()> {
        for adapter in self.import_adapters.values() {
            adapter.write().await.shutdown().await?;
        }
        for adapter in self.export_adapters.values() {
            adapter.write().await.shutdown().await?;
        }
        self.initialized = false;
        Ok(())
    }

    /// Get available adapter types
    pub fn available_adapters(&self) -> Vec<AdapterType> {
        let mut types: Vec<AdapterType> = self.import_adapters.keys().cloned().collect();
        types.extend(self.export_adapters.keys().cloned());
        types.sort_by_key(|t| format!("{:?}", t));
        types.dedup();
        types
    }

    /// Import from a specific adapter
    pub async fn import_from(
        &self,
        adapter_type: AdapterType,
        options: ImportOptions,
    ) -> Result<ImportResult> {
        let adapter = self
            .import_adapters
            .get(&adapter_type)
            .ok_or_else(|| HealthError::adapter(format!("Adapter not found: {:?}", adapter_type)))?;

        adapter.read().await.import(options).await
    }

    /// Import from all available adapters
    pub async fn import_all(&self, options: ImportOptions) -> Result<Vec<ImportResult>> {
        let mut results = Vec::new();

        for (adapter_type, adapter) in &self.import_adapters {
            match adapter.read().await.import(options.clone()).await {
                Ok(result) => results.push(result),
                Err(e) => {
                    // Log error but continue with other adapters
                    eprintln!("Import from {:?} failed: {}", adapter_type, e);
                }
            }
        }

        Ok(results)
    }

    /// Export to a specific adapter
    pub async fn export_to(
        &self,
        adapter_type: AdapterType,
        profile: &HealthProfile,
        options: ExportOptions,
    ) -> Result<ExportResult> {
        let adapter = self
            .export_adapters
            .get(&adapter_type)
            .ok_or_else(|| HealthError::adapter(format!("Adapter not found: {:?}", adapter_type)))?;

        adapter.read().await.export(profile, options).await
    }

    /// Export to all available adapters
    pub async fn export_all(
        &self,
        profile: &HealthProfile,
        options: ExportOptions,
    ) -> Result<Vec<ExportResult>> {
        let mut results = Vec::new();

        for (adapter_type, adapter) in &self.export_adapters {
            match adapter.read().await.export(profile, options.clone()).await {
                Ok(result) => results.push(result),
                Err(e) => {
                    // Log error but continue with other adapters
                    eprintln!("Export to {:?} failed: {}", adapter_type, e);
                }
            }
        }

        Ok(results)
    }

    /// Export to FHIR
    pub async fn export_to_fhir(&self, profile: &HealthProfile) -> Result<ExportResult> {
        self.export_to(
            AdapterType::Fhir,
            profile,
            ExportOptions {
                format: ExportFormat::FhirJson,
                ..Default::default()
            },
        )
        .await
    }

    /// Export to JSON
    pub async fn export_to_json(&self, profile: &HealthProfile) -> Result<ExportResult> {
        self.export_to(
            AdapterType::Export,
            profile,
            ExportOptions {
                format: ExportFormat::Json,
                ..Default::default()
            },
        )
        .await
    }

    /// Export to CSV
    pub async fn export_to_csv(&self, profile: &HealthProfile) -> Result<ExportResult> {
        self.export_to(
            AdapterType::Export,
            profile,
            ExportOptions {
                format: ExportFormat::Csv,
                ..Default::default()
            },
        )
        .await
    }

    /// Import from HealthKit
    pub async fn import_from_healthkit(&self, options: ImportOptions) -> Result<ImportResult> {
        self.import_from(AdapterType::HealthKit, options).await
    }

    /// Import from Health Connect
    pub async fn import_from_health_connect(&self, options: ImportOptions) -> Result<ImportResult> {
        self.import_from(AdapterType::HealthConnect, options).await
    }

    /// Check if manager is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get adapter capabilities
    pub async fn get_capabilities(&self, adapter_type: AdapterType) -> Option<AdapterCapabilities> {
        if let Some(adapter) = self.import_adapters.get(&adapter_type) {
            return Some(adapter.read().await.capabilities());
        }
        if let Some(adapter) = self.export_adapters.get(&adapter_type) {
            return Some(adapter.read().await.capabilities());
        }
        None
    }
}

impl Default for IntegrationManager {
    fn default() -> Self {
        Self::default_config()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::*;
    use uuid::Uuid;

    fn create_test_profile() -> HealthProfile {
        HealthProfile {
            id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            subject: Subject {
                id: Uuid::new_v4(),
                anonymized_id: None,
                birth_year: Some(1990),
                biological_sex: Some(BiologicalSex::Female),
                ethnicity: None,
                consent: None,
            },
            biomarkers: None,
            genomics: None,
            epigenetics: None,
            telomeres: None,
            digital_twin: None,
            interventions: None,
            metadata: Metadata {
                created_at: Utc::now(),
                updated_at: Some(Utc::now()),
                source: None,
                version: "1.0.0".to_string(),
            },
        }
    }

    #[tokio::test]
    async fn test_integration_manager_init() {
        let config = IntegrationConfig::default()
            .with_healthkit()
            .with_health_connect();

        let mut manager = IntegrationManager::new(config);
        manager.initialize().await.unwrap();

        assert!(manager.is_initialized());
        assert!(!manager.available_adapters().is_empty());
    }

    #[tokio::test]
    async fn test_import_from_healthkit() {
        let config = IntegrationConfig::default().with_healthkit();
        let mut manager = IntegrationManager::new(config);
        manager.initialize().await.unwrap();

        let result = manager
            .import_from_healthkit(ImportOptions::default())
            .await
            .unwrap();

        assert!(result.records_imported > 0);
    }

    #[tokio::test]
    async fn test_export_to_json() {
        let mut manager = IntegrationManager::default_config();
        manager.initialize().await.unwrap();

        let profile = create_test_profile();
        let result = manager.export_to_json(&profile).await.unwrap();

        assert!(result.success);
    }
}
