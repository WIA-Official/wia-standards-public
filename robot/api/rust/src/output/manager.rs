//! Output manager for coordinating multiple adapters

use crate::error::{RobotError, RobotResult};
use crate::output::adapter::*;
use std::collections::HashMap;
use std::time::Instant;

/// Output manager for coordinating multiple output adapters
///
/// The OutputManager provides a central hub for registering and using
/// multiple output adapters. It supports:
///
/// - Named adapter registration
/// - Type-based adapter lookup
/// - Targeted output to specific adapters
/// - Broadcasting to all adapters of a type
///
/// # Example
///
/// ```rust
/// use wia_robot::output::*;
///
/// let mut manager = OutputManager::new();
///
/// // Register adapters
/// manager.register("json", Box::new(JsonExporter::new("./data")));
///
/// // Get available adapters
/// let adapters = manager.get_available_adapters();
/// ```
pub struct OutputManager {
    /// Registered adapters
    adapters: HashMap<String, Box<dyn OutputAdapter>>,
}

impl Default for OutputManager {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputManager {
    /// Create a new output manager
    pub fn new() -> Self {
        Self {
            adapters: HashMap::new(),
        }
    }

    /// Register an output adapter
    ///
    /// # Arguments
    ///
    /// * `name` - Unique name for the adapter
    /// * `adapter` - The adapter implementation
    pub fn register(&mut self, name: &str, adapter: Box<dyn OutputAdapter>) {
        self.adapters.insert(name.to_string(), adapter);
    }

    /// Unregister an adapter
    ///
    /// # Arguments
    ///
    /// * `name` - Name of the adapter to remove
    ///
    /// # Returns
    ///
    /// The removed adapter, if it existed
    pub fn unregister(&mut self, name: &str) -> Option<Box<dyn OutputAdapter>> {
        self.adapters.remove(name)
    }

    /// Get an adapter by name
    pub fn get(&self, name: &str) -> Option<&dyn OutputAdapter> {
        self.adapters.get(name).map(|a| a.as_ref())
    }

    /// Get a mutable adapter by name
    pub fn get_mut(&mut self, name: &str) -> Option<&mut Box<dyn OutputAdapter>> {
        self.adapters.get_mut(name)
    }

    /// Output data to a specific adapter
    ///
    /// # Arguments
    ///
    /// * `name` - Name of the adapter
    /// * `data` - Data to output
    ///
    /// # Returns
    ///
    /// The output result with timing information
    pub fn output_to(&self, name: &str, data: &OutputData) -> RobotResult<OutputResult> {
        let start = Instant::now();

        let adapter = self.adapters.get(name).ok_or_else(|| {
            RobotError::InvalidParameter(format!("Adapter not found: {}", name))
        })?;

        if !adapter.is_available() {
            return Err(RobotError::InvalidParameter(format!(
                "Adapter '{}' is not available",
                name
            )));
        }

        let mut result = adapter.output(data)?;
        result.duration_ms = start.elapsed().as_millis() as u64;

        Ok(result)
    }

    /// Broadcast data to all adapters of a specific type
    ///
    /// # Arguments
    ///
    /// * `output_type` - Type of adapters to broadcast to
    /// * `data` - Data to output
    ///
    /// # Returns
    ///
    /// Results from all matching adapters
    pub fn broadcast(
        &self,
        output_type: OutputType,
        data: &OutputData,
    ) -> Vec<(String, RobotResult<OutputResult>)> {
        let mut results = Vec::new();

        for (name, adapter) in &self.adapters {
            if adapter.output_type() == output_type && adapter.is_available() {
                let start = Instant::now();
                let result = adapter.output(data).map(|mut r| {
                    r.duration_ms = start.elapsed().as_millis() as u64;
                    r
                });
                results.push((name.clone(), result));
            }
        }

        results
    }

    /// Broadcast data to all available adapters
    ///
    /// # Arguments
    ///
    /// * `data` - Data to output
    ///
    /// # Returns
    ///
    /// Results from all available adapters
    pub fn broadcast_all(&self, data: &OutputData) -> Vec<(String, RobotResult<OutputResult>)> {
        let mut results = Vec::new();

        for (name, adapter) in &self.adapters {
            if adapter.is_available() {
                let start = Instant::now();
                let result = adapter.output(data).map(|mut r| {
                    r.duration_ms = start.elapsed().as_millis() as u64;
                    r
                });
                results.push((name.clone(), result));
            }
        }

        results
    }

    /// Get all available adapter names
    pub fn get_available_adapters(&self) -> Vec<String> {
        self.adapters
            .iter()
            .filter(|(_, adapter)| adapter.is_available())
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// Get adapter names by type
    pub fn get_by_type(&self, output_type: OutputType) -> Vec<String> {
        self.adapters
            .iter()
            .filter(|(_, adapter)| adapter.output_type() == output_type)
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// Get total number of registered adapters
    pub fn count(&self) -> usize {
        self.adapters.len()
    }

    /// Check if an adapter is registered
    pub fn contains(&self, name: &str) -> bool {
        self.adapters.contains_key(name)
    }

    /// Initialize all adapters with configuration
    pub fn initialize_all(&mut self, config: &OutputConfig) -> Vec<(String, RobotResult<()>)> {
        let mut results = Vec::new();

        for (name, adapter) in &mut self.adapters {
            let result = adapter.initialize(config);
            results.push((name.clone(), result));
        }

        results
    }

    /// Dispose all adapters
    pub fn dispose_all(&mut self) -> Vec<(String, RobotResult<()>)> {
        let mut results = Vec::new();

        for (name, adapter) in &mut self.adapters {
            let result = adapter.dispose();
            results.push((name.clone(), result));
        }

        results
    }

    /// Get statistics about registered adapters
    pub fn stats(&self) -> OutputManagerStats {
        let mut stats = OutputManagerStats::default();
        stats.total_adapters = self.adapters.len();

        for adapter in self.adapters.values() {
            if adapter.is_available() {
                stats.available_adapters += 1;
            }
            match adapter.output_type() {
                OutputType::Visualization => stats.visualization_count += 1,
                OutputType::Medical => stats.medical_count += 1,
                OutputType::AiMl => stats.aiml_count += 1,
                OutputType::Dashboard => stats.dashboard_count += 1,
                OutputType::Export => stats.export_count += 1,
                OutputType::Alert => stats.alert_count += 1,
                OutputType::Logger => stats.logger_count += 1,
                OutputType::Custom(_) => stats.custom_count += 1,
            }
        }

        stats
    }
}

/// Statistics about the output manager
#[derive(Debug, Clone, Default)]
pub struct OutputManagerStats {
    /// Total registered adapters
    pub total_adapters: usize,
    /// Available adapters
    pub available_adapters: usize,
    /// Visualization adapters
    pub visualization_count: usize,
    /// Medical adapters
    pub medical_count: usize,
    /// AI/ML adapters
    pub aiml_count: usize,
    /// Dashboard adapters
    pub dashboard_count: usize,
    /// Export adapters
    pub export_count: usize,
    /// Alert adapters
    pub alert_count: usize,
    /// Logger adapters
    pub logger_count: usize,
    /// Custom adapters
    pub custom_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock adapter for testing
    struct MockAdapter {
        name: String,
        output_type: OutputType,
        available: bool,
    }

    // SAFETY: MockAdapter is thread-safe as all fields are immutable after creation
    unsafe impl Sync for MockAdapter {}

    impl MockAdapter {
        fn new(name: &str, output_type: OutputType) -> Self {
            Self {
                name: name.to_string(),
                output_type,
                available: true,
            }
        }
    }

    impl OutputAdapter for MockAdapter {
        fn output_type(&self) -> OutputType {
            self.output_type.clone()
        }

        fn name(&self) -> &str {
            &self.name
        }

        fn initialize(&mut self, _config: &OutputConfig) -> RobotResult<()> {
            Ok(())
        }

        fn output(&self, _data: &OutputData) -> RobotResult<OutputResult> {
            Ok(OutputResult::success("Mock output"))
        }

        fn is_available(&self) -> bool {
            self.available
        }

        fn dispose(&mut self) -> RobotResult<()> {
            Ok(())
        }
    }

    #[test]
    fn test_manager_register_unregister() {
        let mut manager = OutputManager::new();
        assert_eq!(manager.count(), 0);

        manager.register("mock1", Box::new(MockAdapter::new("mock1", OutputType::Export)));
        assert_eq!(manager.count(), 1);
        assert!(manager.contains("mock1"));

        manager.unregister("mock1");
        assert_eq!(manager.count(), 0);
        assert!(!manager.contains("mock1"));
    }

    #[test]
    fn test_manager_output_to() {
        let mut manager = OutputManager::new();
        manager.register("mock", Box::new(MockAdapter::new("mock", OutputType::Export)));

        let data = OutputData::new("device-001", "exoskeleton");
        let result = manager.output_to("mock", &data).unwrap();

        assert!(result.success);
    }

    #[test]
    fn test_manager_output_to_not_found() {
        let manager = OutputManager::new();
        let data = OutputData::new("device-001", "exoskeleton");
        let result = manager.output_to("nonexistent", &data);

        assert!(result.is_err());
    }

    #[test]
    fn test_manager_broadcast() {
        let mut manager = OutputManager::new();
        manager.register("export1", Box::new(MockAdapter::new("export1", OutputType::Export)));
        manager.register("export2", Box::new(MockAdapter::new("export2", OutputType::Export)));
        manager.register("medical", Box::new(MockAdapter::new("medical", OutputType::Medical)));

        let data = OutputData::new("device-001", "exoskeleton");
        let results = manager.broadcast(OutputType::Export, &data);

        assert_eq!(results.len(), 2);
        for (_, result) in results {
            assert!(result.is_ok());
        }
    }

    #[test]
    fn test_manager_get_by_type() {
        let mut manager = OutputManager::new();
        manager.register("export1", Box::new(MockAdapter::new("export1", OutputType::Export)));
        manager.register("export2", Box::new(MockAdapter::new("export2", OutputType::Export)));
        manager.register("medical", Box::new(MockAdapter::new("medical", OutputType::Medical)));

        let exports = manager.get_by_type(OutputType::Export);
        assert_eq!(exports.len(), 2);

        let medicals = manager.get_by_type(OutputType::Medical);
        assert_eq!(medicals.len(), 1);
    }

    #[test]
    fn test_manager_stats() {
        let mut manager = OutputManager::new();
        manager.register("export", Box::new(MockAdapter::new("export", OutputType::Export)));
        manager.register("medical", Box::new(MockAdapter::new("medical", OutputType::Medical)));
        manager.register("aiml", Box::new(MockAdapter::new("aiml", OutputType::AiMl)));

        let stats = manager.stats();
        assert_eq!(stats.total_adapters, 3);
        assert_eq!(stats.available_adapters, 3);
        assert_eq!(stats.export_count, 1);
        assert_eq!(stats.medical_count, 1);
        assert_eq!(stats.aiml_count, 1);
    }
}
