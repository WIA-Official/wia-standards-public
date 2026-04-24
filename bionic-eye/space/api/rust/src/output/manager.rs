//! Output manager for coordinating multiple output adapters

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use tokio::sync::RwLock;

use super::adapter::{OutputAdapter, OutputConfig, OutputError, OutputType};
use super::data::{OutputData, OutputResult};

/// Output manager for coordinating adapters
pub struct OutputManager {
    adapters: Arc<RwLock<HashMap<String, Box<dyn OutputAdapter>>>>,
    default_types: HashSet<OutputType>,
}

impl OutputManager {
    /// Create new output manager
    pub fn new() -> Self {
        Self {
            adapters: Arc::new(RwLock::new(HashMap::new())),
            default_types: HashSet::new(),
        }
    }

    /// Register an adapter
    pub async fn register(&self, name: impl Into<String>, adapter: Box<dyn OutputAdapter>) {
        let name = name.into();
        self.adapters.write().await.insert(name, adapter);
    }

    /// Unregister an adapter
    pub async fn unregister(&self, name: &str) -> Option<Box<dyn OutputAdapter>> {
        self.adapters.write().await.remove(name)
    }

    /// Set default output types
    pub fn set_defaults(&mut self, types: Vec<OutputType>) {
        self.default_types = types.into_iter().collect();
    }

    /// Add a default output type
    pub fn add_default(&mut self, output_type: OutputType) {
        self.default_types.insert(output_type);
    }

    /// Output to a specific adapter
    pub async fn output_to(
        &self,
        adapter_name: &str,
        data: &OutputData,
    ) -> Result<OutputResult, OutputError> {
        let adapters = self.adapters.read().await;

        match adapters.get(adapter_name) {
            Some(adapter) => {
                if !adapter.is_available() {
                    return Err(OutputError::NotAvailable(adapter_name.to_string()));
                }
                adapter.output(data).await
            }
            None => Err(OutputError::NotAvailable(format!(
                "Adapter '{}' not found",
                adapter_name
            ))),
        }
    }

    /// Broadcast to all adapters of a specific type
    pub async fn broadcast(
        &self,
        output_type: OutputType,
        data: &OutputData,
    ) -> Vec<Result<OutputResult, OutputError>> {
        let adapters = self.adapters.read().await;
        let mut results = Vec::new();

        for (_name, adapter) in adapters.iter() {
            if adapter.output_type() == output_type && adapter.is_available() {
                let result = adapter.output(data).await;
                results.push(result);
            }
        }

        if results.is_empty() {
            results.push(Err(OutputError::NotAvailable(format!(
                "No adapters available for type {:?}",
                output_type
            ))));
        }

        results
    }

    /// Output to all default adapters
    pub async fn output(&self, data: &OutputData) -> Vec<Result<OutputResult, OutputError>> {
        let adapters = self.adapters.read().await;
        let mut results = Vec::new();

        for (_name, adapter) in adapters.iter() {
            if self.default_types.contains(&adapter.output_type()) && adapter.is_available() {
                let result = adapter.output(data).await;
                results.push(result);
            }
        }

        // If no defaults, output to all available adapters
        if results.is_empty() && self.default_types.is_empty() {
            for (_name, adapter) in adapters.iter() {
                if adapter.is_available() {
                    let result = adapter.output(data).await;
                    results.push(result);
                }
            }
        }

        results
    }

    /// Output to all available adapters
    pub async fn output_all(&self, data: &OutputData) -> Vec<Result<OutputResult, OutputError>> {
        let adapters = self.adapters.read().await;
        let mut results = Vec::new();

        for (_name, adapter) in adapters.iter() {
            if adapter.is_available() {
                let result = adapter.output(data).await;
                results.push(result);
            }
        }

        results
    }

    /// Get adapter names
    pub async fn get_adapter_names(&self) -> Vec<String> {
        self.adapters.read().await.keys().cloned().collect()
    }

    /// Get adapter count
    pub async fn adapter_count(&self) -> usize {
        self.adapters.read().await.len()
    }

    /// Check if an adapter exists
    pub async fn has_adapter(&self, name: &str) -> bool {
        self.adapters.read().await.contains_key(name)
    }

    /// Check if an adapter is available
    pub async fn is_adapter_available(&self, name: &str) -> bool {
        self.adapters
            .read()
            .await
            .get(name)
            .map(|a| a.is_available())
            .unwrap_or(false)
    }

    /// Get available adapters by type
    pub async fn get_available_by_type(&self, output_type: OutputType) -> Vec<String> {
        self.adapters
            .read()
            .await
            .iter()
            .filter(|(_, a)| a.output_type() == output_type && a.is_available())
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// Initialize all adapters
    pub async fn initialize_all(&self, config: &OutputConfig) -> Vec<Result<(), OutputError>> {
        let mut adapters = self.adapters.write().await;
        let mut results = Vec::new();

        for (_, adapter) in adapters.iter_mut() {
            let result = adapter.initialize(config).await;
            results.push(result);
        }

        results
    }

    /// Dispose all adapters
    pub async fn dispose_all(&self) -> Vec<Result<(), OutputError>> {
        let mut adapters = self.adapters.write().await;
        let mut results = Vec::new();

        for (_, adapter) in adapters.iter_mut() {
            let result = adapter.dispose().await;
            results.push(result);
        }

        results
    }
}

impl Default for OutputManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::super::adapter::MockOutputAdapter;
    use super::*;

    #[tokio::test]
    async fn test_manager_creation() {
        let manager = OutputManager::new();
        assert_eq!(manager.adapter_count().await, 0);
    }

    #[tokio::test]
    async fn test_register_adapter() {
        let manager = OutputManager::new();
        let adapter = MockOutputAdapter::visualization("test-vis");

        manager.register("vis", Box::new(adapter)).await;

        assert_eq!(manager.adapter_count().await, 1);
        assert!(manager.has_adapter("vis").await);
        assert!(manager.is_adapter_available("vis").await);
    }

    #[tokio::test]
    async fn test_unregister_adapter() {
        let manager = OutputManager::new();
        let adapter = MockOutputAdapter::visualization("test-vis");

        manager.register("vis", Box::new(adapter)).await;
        manager.unregister("vis").await;

        assert_eq!(manager.adapter_count().await, 0);
        assert!(!manager.has_adapter("vis").await);
    }

    #[tokio::test]
    async fn test_output_to_specific() {
        let manager = OutputManager::new();
        let adapter = MockOutputAdapter::visualization("test-vis");

        manager.register("vis", Box::new(adapter)).await;

        let data = OutputData::new();
        let result = manager.output_to("vis", &data).await;

        assert!(result.is_ok());
        assert!(result.unwrap().is_success());
    }

    #[tokio::test]
    async fn test_output_to_nonexistent() {
        let manager = OutputManager::new();

        let data = OutputData::new();
        let result = manager.output_to("nonexistent", &data).await;

        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_broadcast() {
        let manager = OutputManager::new();

        let adapter1 = MockOutputAdapter::exporter("export1");
        let adapter2 = MockOutputAdapter::exporter("export2");
        let adapter3 = MockOutputAdapter::visualization("vis1");

        manager.register("export1", Box::new(adapter1)).await;
        manager.register("export2", Box::new(adapter2)).await;
        manager.register("vis1", Box::new(adapter3)).await;

        let data = OutputData::new();
        let results = manager.broadcast(OutputType::Export, &data).await;

        // Should output to both exporters
        assert_eq!(results.len(), 2);
        assert!(results.iter().all(|r| r.is_ok()));
    }

    #[tokio::test]
    async fn test_get_available_by_type() {
        let manager = OutputManager::new();

        let adapter1 = MockOutputAdapter::exporter("export1");
        let adapter2 = MockOutputAdapter::exporter("export2");

        manager.register("export1", Box::new(adapter1)).await;
        manager.register("export2", Box::new(adapter2)).await;

        let exporters = manager.get_available_by_type(OutputType::Export).await;
        assert_eq!(exporters.len(), 2);

        let visualizers = manager
            .get_available_by_type(OutputType::Visualization)
            .await;
        assert_eq!(visualizers.len(), 0);
    }

    #[tokio::test]
    async fn test_defaults() {
        let mut manager = OutputManager::new();

        let adapter1 = MockOutputAdapter::exporter("export1");
        let adapter2 = MockOutputAdapter::visualization("vis1");

        manager.register("export1", Box::new(adapter1)).await;
        manager.register("vis1", Box::new(adapter2)).await;

        // Set export as default
        manager.set_defaults(vec![OutputType::Export]);

        let data = OutputData::new();
        let results = manager.output(&data).await;

        // Should only output to exporter
        assert_eq!(results.len(), 1);
    }
}
