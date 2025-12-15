//! Provider registry for managing multiple data providers

use std::collections::HashMap;

use super::traits::{DataProvider, ProviderQuery};
use crate::integration::error::{IntegrationError, IntegrationResult};
use crate::types::MaterialData;

/// Registry for managing multiple data providers
pub struct ProviderRegistry {
    providers: HashMap<String, Box<dyn DataProvider>>,
}

impl ProviderRegistry {
    /// Create a new empty registry
    pub fn new() -> Self {
        Self {
            providers: HashMap::new(),
        }
    }

    /// Register a data provider
    pub fn register(&mut self, provider: Box<dyn DataProvider>) {
        let name = provider.name().to_string();
        self.providers.insert(name, provider);
    }

    /// Unregister a provider by name
    pub fn unregister(&mut self, name: &str) -> Option<Box<dyn DataProvider>> {
        self.providers.remove(name)
    }

    /// Get a provider by name
    pub fn get(&self, name: &str) -> Option<&dyn DataProvider> {
        self.providers.get(name).map(|p| p.as_ref())
    }

    /// Get a mutable reference to a provider
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Box<dyn DataProvider>> {
        self.providers.get_mut(name)
    }

    /// List all registered provider names
    pub fn list(&self) -> Vec<&str> {
        self.providers.keys().map(|k| k.as_str()).collect()
    }

    /// List all connected providers
    pub fn list_connected(&self) -> Vec<&str> {
        self.providers
            .iter()
            .filter(|(_, p)| p.is_connected())
            .map(|(k, _)| k.as_str())
            .collect()
    }

    /// Get provider count
    pub fn count(&self) -> usize {
        self.providers.len()
    }

    /// Check if a provider exists
    pub fn contains(&self, name: &str) -> bool {
        self.providers.contains_key(name)
    }

    /// Search across all connected providers
    ///
    /// Returns results tagged with the provider name
    pub async fn search_all(
        &self,
        query: &ProviderQuery,
    ) -> IntegrationResult<Vec<(String, MaterialData)>> {
        let mut all_results = Vec::new();
        let mut errors = Vec::new();

        for (name, provider) in &self.providers {
            if !provider.is_connected() {
                continue;
            }

            match provider.search(query).await {
                Ok(materials) => {
                    for material in materials {
                        all_results.push((name.clone(), material));
                    }
                }
                Err(e) => {
                    errors.push(format!("{}: {}", name, e));
                }
            }
        }

        // If all providers failed, return an error
        if all_results.is_empty() && !errors.is_empty() {
            return Err(IntegrationError::NetworkError(errors.join("; ")));
        }

        Ok(all_results)
    }

    /// Get a material by external ID from a specific provider
    pub async fn get_by_id(
        &self,
        provider_name: &str,
        external_id: &str,
    ) -> IntegrationResult<MaterialData> {
        let provider = self
            .providers
            .get(provider_name)
            .ok_or_else(|| IntegrationError::ProviderNotFound(provider_name.to_string()))?;

        provider.get_by_id(external_id).await
    }
}

impl Default for ProviderRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integration::providers::MockProvider;

    #[test]
    fn test_registry_register() {
        let mut registry = ProviderRegistry::new();
        registry.register(Box::new(MockProvider::new()));

        assert_eq!(registry.count(), 1);
        assert!(registry.contains("mock"));
    }

    #[test]
    fn test_registry_list() {
        let mut registry = ProviderRegistry::new();
        registry.register(Box::new(MockProvider::new()));

        let list = registry.list();
        assert!(list.contains(&"mock"));
    }

    #[test]
    fn test_registry_unregister() {
        let mut registry = ProviderRegistry::new();
        registry.register(Box::new(MockProvider::new()));

        let removed = registry.unregister("mock");
        assert!(removed.is_some());
        assert_eq!(registry.count(), 0);
    }

    #[tokio::test]
    async fn test_registry_search_all() {
        let mut registry = ProviderRegistry::new();

        let mut mock = MockProvider::new();
        mock.connect(crate::integration::ProviderConfig::default())
            .await
            .unwrap();

        registry.register(Box::new(mock));

        let query = ProviderQuery::default();
        let results = registry.search_all(&query).await.unwrap();

        assert!(!results.is_empty());
    }
}
