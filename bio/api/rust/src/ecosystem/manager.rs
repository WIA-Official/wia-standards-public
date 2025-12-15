//! Ecosystem manager for coordinating adapters

use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use super::base::*;
use super::config::AdapterConfig;

/// Ecosystem manager for managing multiple adapters
#[derive(Debug)]
pub struct EcosystemManager {
    /// Registered adapters
    adapters: HashMap<AdapterType, Arc<RwLock<Box<dyn IEcosystemAdapter>>>>,
    /// Default configurations
    configs: HashMap<AdapterType, AdapterConfig>,
}

impl EcosystemManager {
    /// Create a new ecosystem manager
    pub fn new() -> Self {
        Self {
            adapters: HashMap::new(),
            configs: HashMap::new(),
        }
    }

    /// Register an adapter
    pub fn register(&mut self, adapter: Box<dyn IEcosystemAdapter>) {
        let adapter_type = adapter.adapter_type();
        self.adapters.insert(adapter_type, Arc::new(RwLock::new(adapter)));
    }

    /// Register an adapter with configuration
    pub fn register_with_config(&mut self, adapter: Box<dyn IEcosystemAdapter>, config: AdapterConfig) {
        let adapter_type = adapter.adapter_type();
        self.adapters.insert(adapter_type, Arc::new(RwLock::new(adapter)));
        self.configs.insert(adapter_type, config);
    }

    /// Unregister an adapter
    pub fn unregister(&mut self, adapter_type: AdapterType) -> Option<Arc<RwLock<Box<dyn IEcosystemAdapter>>>> {
        self.configs.remove(&adapter_type);
        self.adapters.remove(&adapter_type)
    }

    /// Get an adapter
    pub fn get(&self, adapter_type: AdapterType) -> Option<Arc<RwLock<Box<dyn IEcosystemAdapter>>>> {
        self.adapters.get(&adapter_type).cloned()
    }

    /// Check if adapter is registered
    pub fn has_adapter(&self, adapter_type: AdapterType) -> bool {
        self.adapters.contains_key(&adapter_type)
    }

    /// Get all registered adapter types
    pub fn registered_types(&self) -> Vec<AdapterType> {
        self.adapters.keys().cloned().collect()
    }

    /// Initialize all adapters
    pub async fn initialize_all(&self, configs: &[AdapterConfig]) -> Result<(), AdapterError> {
        for adapter in self.adapters.values() {
            let mut adapter = adapter.write().await;
            let adapter_type = adapter.adapter_type();

            // Find config for this adapter type
            let config = self.configs.get(&adapter_type)
                .or_else(|| configs.iter().find(|c| !c.base_url.is_empty()))
                .cloned()
                .unwrap_or_default();

            adapter.initialize(&config).await?;
        }
        Ok(())
    }

    /// Initialize a specific adapter
    pub async fn initialize(&self, adapter_type: AdapterType, config: &AdapterConfig) -> Result<(), AdapterError> {
        if let Some(adapter) = self.adapters.get(&adapter_type) {
            let mut adapter = adapter.write().await;
            adapter.initialize(config).await
        } else {
            Err(AdapterError::NotFound(format!("Adapter {:?} not found", adapter_type)))
        }
    }

    /// Health check all adapters
    pub async fn health_check_all(&self) -> HashMap<AdapterType, HealthStatus> {
        let mut results = HashMap::new();

        for (adapter_type, adapter) in &self.adapters {
            let adapter = adapter.read().await;
            let status = adapter.health_check().await
                .unwrap_or_else(|e| HealthStatus::unhealthy(e.to_string()));
            results.insert(*adapter_type, status);
        }

        results
    }

    /// Health check a specific adapter
    pub async fn health_check(&self, adapter_type: AdapterType) -> Result<HealthStatus, AdapterError> {
        if let Some(adapter) = self.adapters.get(&adapter_type) {
            let adapter = adapter.read().await;
            adapter.health_check().await
        } else {
            Err(AdapterError::NotFound(format!("Adapter {:?} not found", adapter_type)))
        }
    }

    /// Shutdown all adapters
    pub async fn shutdown_all(&self) -> Result<(), AdapterError> {
        for adapter in self.adapters.values() {
            let mut adapter = adapter.write().await;
            adapter.shutdown().await?;
        }
        Ok(())
    }

    /// Shutdown a specific adapter
    pub async fn shutdown(&self, adapter_type: AdapterType) -> Result<(), AdapterError> {
        if let Some(adapter) = self.adapters.get(&adapter_type) {
            let mut adapter = adapter.write().await;
            adapter.shutdown().await
        } else {
            Err(AdapterError::NotFound(format!("Adapter {:?} not found", adapter_type)))
        }
    }

    /// Get adapter count
    pub fn adapter_count(&self) -> usize {
        self.adapters.len()
    }
}

impl Default for EcosystemManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::mock::MockAdapter;

    #[tokio::test]
    async fn test_manager_register() {
        let mut manager = EcosystemManager::new();

        assert_eq!(manager.adapter_count(), 0);
        assert!(!manager.has_adapter(AdapterType::Mock));

        manager.register(Box::new(MockAdapter::new()));

        assert_eq!(manager.adapter_count(), 1);
        assert!(manager.has_adapter(AdapterType::Mock));
    }

    #[tokio::test]
    async fn test_manager_initialize() {
        let mut manager = EcosystemManager::new();
        manager.register(Box::new(MockAdapter::new()));

        let config = AdapterConfig::new("https://mock.example.com");
        manager.initialize(AdapterType::Mock, &config).await.unwrap();

        let adapter = manager.get(AdapterType::Mock).unwrap();
        let adapter = adapter.read().await;
        assert!(adapter.is_initialized());
    }

    #[tokio::test]
    async fn test_manager_health_check() {
        let mut manager = EcosystemManager::new();
        manager.register(Box::new(MockAdapter::new()));

        let config = AdapterConfig::new("https://mock.example.com");
        manager.initialize(AdapterType::Mock, &config).await.unwrap();

        let status = manager.health_check(AdapterType::Mock).await.unwrap();
        assert!(status.healthy);
    }

    #[tokio::test]
    async fn test_manager_registered_types() {
        let mut manager = EcosystemManager::new();
        manager.register(Box::new(MockAdapter::new()));

        let types = manager.registered_types();
        assert!(types.contains(&AdapterType::Mock));
    }

    #[tokio::test]
    async fn test_manager_unregister() {
        let mut manager = EcosystemManager::new();
        manager.register(Box::new(MockAdapter::new()));

        assert!(manager.has_adapter(AdapterType::Mock));

        manager.unregister(AdapterType::Mock);

        assert!(!manager.has_adapter(AdapterType::Mock));
    }
}
