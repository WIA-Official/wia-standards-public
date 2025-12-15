//! Integration manager for unified access to providers and exporters

use std::path::Path;

use super::error::{IntegrationError, IntegrationResult};
use super::exporters::{ExportFormat, ExportOptions, ExporterRegistry, ImportOptions};
use super::providers::{
    DataProvider, MaterialsProjectProvider, MockProvider, OptimadeProvider, ProviderConfig,
    ProviderQuery, ProviderRegistry, ProviderStatus,
};
use crate::types::MaterialData;

/// Unified integration manager
///
/// Provides a single interface for:
/// - External data providers (OPTIMADE, Materials Project, etc.)
/// - File format exporters/importers (CIF, POSCAR, XYZ)
pub struct IntegrationManager {
    providers: ProviderRegistry,
    exporters: ExporterRegistry,
}

impl IntegrationManager {
    /// Create a new integration manager with default components
    pub fn new() -> Self {
        let mut providers = ProviderRegistry::new();

        // Register default providers
        providers.register(Box::new(OptimadeProvider::new()));
        providers.register(Box::new(MockProvider::new()));

        Self {
            providers,
            exporters: ExporterRegistry::with_defaults(),
        }
    }

    /// Create with Materials Project API key
    pub fn with_materials_project(api_key: &str) -> Self {
        let mut manager = Self::new();
        manager
            .providers
            .register(Box::new(MaterialsProjectProvider::with_api_key(api_key)));
        manager
    }

    // =========================================================================
    // Provider operations
    // =========================================================================

    /// Get the provider registry
    pub fn providers(&self) -> &ProviderRegistry {
        &self.providers
    }

    /// Get mutable provider registry
    pub fn providers_mut(&mut self) -> &mut ProviderRegistry {
        &mut self.providers
    }

    /// Register a new provider
    pub fn register_provider(&mut self, provider: Box<dyn DataProvider>) {
        self.providers.register(provider);
    }

    /// Connect to a specific provider
    pub async fn connect_provider(
        &mut self,
        name: &str,
        config: ProviderConfig,
    ) -> IntegrationResult<()> {
        let provider = self
            .providers
            .get_mut(name)
            .ok_or_else(|| IntegrationError::ProviderNotFound(name.to_string()))?;

        provider.connect(config).await
    }

    /// Disconnect from a specific provider
    pub async fn disconnect_provider(&mut self, name: &str) -> IntegrationResult<()> {
        let provider = self
            .providers
            .get_mut(name)
            .ok_or_else(|| IntegrationError::ProviderNotFound(name.to_string()))?;

        provider.disconnect().await
    }

    /// Get provider status
    pub async fn provider_status(&self, name: &str) -> IntegrationResult<ProviderStatus> {
        let provider = self
            .providers
            .get(name)
            .ok_or_else(|| IntegrationError::ProviderNotFound(name.to_string()))?;

        provider.status().await
    }

    /// List all registered providers
    pub fn list_providers(&self) -> Vec<&str> {
        self.providers.list()
    }

    /// List connected providers
    pub fn list_connected_providers(&self) -> Vec<&str> {
        self.providers.list_connected()
    }

    /// Search across all connected providers
    pub async fn search(&self, query: &ProviderQuery) -> IntegrationResult<Vec<MaterialData>> {
        let results = self.providers.search_all(query).await?;
        Ok(results.into_iter().map(|(_, m)| m).collect())
    }

    /// Search with provider attribution
    pub async fn search_with_providers(
        &self,
        query: &ProviderQuery,
    ) -> IntegrationResult<Vec<(String, MaterialData)>> {
        self.providers.search_all(query).await
    }

    /// Fetch material from specific provider by external ID
    pub async fn fetch(
        &self,
        provider: &str,
        external_id: &str,
    ) -> IntegrationResult<MaterialData> {
        self.providers.get_by_id(provider, external_id).await
    }

    // =========================================================================
    // Exporter operations
    // =========================================================================

    /// Get the exporter registry
    pub fn exporters(&self) -> &ExporterRegistry {
        &self.exporters
    }

    /// List supported export formats
    pub fn list_formats(&self) -> Vec<ExportFormat> {
        self.exporters.list()
    }

    /// Check if a format is supported
    pub fn supports_format(&self, format: ExportFormat) -> bool {
        self.exporters.supports(format)
    }

    /// Export material to string
    pub fn export(&self, format: ExportFormat, material: &MaterialData) -> IntegrationResult<String> {
        self.exporters
            .export(format, material, &ExportOptions::default())
    }

    /// Export material with options
    pub fn export_with_options(
        &self,
        format: ExportFormat,
        material: &MaterialData,
        options: &ExportOptions,
    ) -> IntegrationResult<String> {
        self.exporters.export(format, material, options)
    }

    /// Export material to file
    pub fn export_to_file(&self, material: &MaterialData, path: &Path) -> IntegrationResult<()> {
        self.exporters
            .export_to_file(material, path, &ExportOptions::default())
    }

    /// Export material to file with options
    pub fn export_to_file_with_options(
        &self,
        material: &MaterialData,
        path: &Path,
        options: &ExportOptions,
    ) -> IntegrationResult<()> {
        self.exporters.export_to_file(material, path, options)
    }

    /// Import material from string
    pub fn import(&self, format: ExportFormat, content: &str) -> IntegrationResult<MaterialData> {
        self.exporters
            .import(format, content, &ImportOptions::default())
    }

    /// Import material with options
    pub fn import_with_options(
        &self,
        format: ExportFormat,
        content: &str,
        options: &ImportOptions,
    ) -> IntegrationResult<MaterialData> {
        self.exporters.import(format, content, options)
    }

    /// Import material from file (auto-detect format)
    pub fn import_file(&self, path: &Path) -> IntegrationResult<MaterialData> {
        self.exporters.import_file(path, &ImportOptions::default())
    }

    /// Import material from file with options
    pub fn import_file_with_options(
        &self,
        path: &Path,
        options: &ImportOptions,
    ) -> IntegrationResult<MaterialData> {
        self.exporters.import_file(path, options)
    }

    /// Convert between file formats
    pub fn convert(&self, input_path: &Path, output_path: &Path) -> IntegrationResult<()> {
        let material = self.import_file(input_path)?;
        self.export_to_file(&material, output_path)
    }

    /// Convert with options
    pub fn convert_with_options(
        &self,
        input_path: &Path,
        output_path: &Path,
        import_options: &ImportOptions,
        export_options: &ExportOptions,
    ) -> IntegrationResult<()> {
        let material = self.import_file_with_options(input_path, import_options)?;
        self.export_to_file_with_options(&material, output_path, export_options)
    }
}

impl Default for IntegrationManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Builder for IntegrationManager
pub struct IntegrationManagerBuilder {
    mp_api_key: Option<String>,
    optimade_providers: Vec<String>,
    include_mock: bool,
}

impl IntegrationManagerBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self {
            mp_api_key: None,
            optimade_providers: Vec::new(),
            include_mock: true,
        }
    }

    /// Add Materials Project with API key
    pub fn with_materials_project(mut self, api_key: &str) -> Self {
        self.mp_api_key = Some(api_key.to_string());
        self
    }

    /// Add OPTIMADE provider URL
    pub fn with_optimade_provider(mut self, url: &str) -> Self {
        self.optimade_providers.push(url.to_string());
        self
    }

    /// Disable mock provider
    pub fn without_mock(mut self) -> Self {
        self.include_mock = false;
        self
    }

    /// Build the manager
    pub fn build(self) -> IntegrationManager {
        let mut providers = ProviderRegistry::new();

        // Add mock provider if enabled
        if self.include_mock {
            providers.register(Box::new(MockProvider::new()));
        }

        // Add default OPTIMADE provider
        providers.register(Box::new(OptimadeProvider::new()));

        // Add Materials Project if API key provided
        if let Some(api_key) = self.mp_api_key {
            providers.register(Box::new(MaterialsProjectProvider::with_api_key(&api_key)));
        }

        // Add custom OPTIMADE providers
        for (_i, url) in self.optimade_providers.into_iter().enumerate() {
            let provider = OptimadeProvider::with_provider(&url);
            // Note: In a real implementation, we'd want to set a unique name on the provider
            // For now, we'll just use the default name
            providers.register(Box::new(provider));
        }

        IntegrationManager {
            providers,
            exporters: ExporterRegistry::with_defaults(),
        }
    }
}

impl Default for IntegrationManagerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manager_new() {
        let manager = IntegrationManager::new();

        assert!(manager.list_providers().contains(&"mock"));
        assert!(manager.list_providers().contains(&"optimade"));
    }

    #[test]
    fn test_manager_with_materials_project() {
        let manager = IntegrationManager::with_materials_project("test_key");

        assert!(manager.list_providers().contains(&"materials_project"));
    }

    #[test]
    fn test_manager_list_formats() {
        let manager = IntegrationManager::new();
        let formats = manager.list_formats();

        assert!(formats.contains(&ExportFormat::Cif));
        assert!(formats.contains(&ExportFormat::Poscar));
        assert!(formats.contains(&ExportFormat::Xyz));
    }

    #[tokio::test]
    async fn test_manager_connect_provider() {
        let mut manager = IntegrationManager::new();

        let config = ProviderConfig::default();
        manager.connect_provider("mock", config).await.unwrap();

        assert!(manager.list_connected_providers().contains(&"mock"));
    }

    #[tokio::test]
    async fn test_manager_search() {
        let mut manager = IntegrationManager::new();

        // Connect mock provider
        manager
            .connect_provider("mock", ProviderConfig::default())
            .await
            .unwrap();

        // Search
        let query = ProviderQuery::default();
        let results = manager.search(&query).await.unwrap();

        assert!(!results.is_empty());
    }

    #[test]
    fn test_builder() {
        let manager = IntegrationManagerBuilder::new()
            .with_materials_project("test_key")
            .without_mock()
            .build();

        assert!(manager.list_providers().contains(&"materials_project"));
        assert!(!manager.list_providers().contains(&"mock"));
    }
}
