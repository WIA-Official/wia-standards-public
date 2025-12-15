//! Mock adapter for testing

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::RwLock;
use chrono::Utc;

use super::base::*;
use super::config::AdapterConfig;
use crate::types::{Sequence, CrisprExperiment, ProteinStructure, BioPart};

/// Mock adapter for testing
#[derive(Debug)]
pub struct MockAdapter {
    /// Adapter name
    name: String,
    /// Initialized flag
    initialized: bool,
    /// Simulated latency in ms
    latency_ms: u64,
    /// Fail health check
    fail_health: bool,
    /// Stored sequences
    sequences: RwLock<HashMap<String, Sequence>>,
    /// Stored experiments
    experiments: RwLock<HashMap<String, CrisprExperiment>>,
    /// Stored structures
    structures: RwLock<HashMap<String, ProteinStructure>>,
    /// Stored parts
    parts: RwLock<HashMap<String, BioPart>>,
    /// Export counter
    export_count: RwLock<u32>,
}

impl MockAdapter {
    /// Create a new mock adapter
    pub fn new() -> Self {
        Self {
            name: "Mock Adapter".to_string(),
            initialized: false,
            latency_ms: 10,
            fail_health: false,
            sequences: RwLock::new(HashMap::new()),
            experiments: RwLock::new(HashMap::new()),
            structures: RwLock::new(HashMap::new()),
            parts: RwLock::new(HashMap::new()),
            export_count: RwLock::new(0),
        }
    }

    /// Create with custom name
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// Set simulated latency
    pub fn with_latency(mut self, latency_ms: u64) -> Self {
        self.latency_ms = latency_ms;
        self
    }

    /// Set fail health check
    pub fn fail_health(mut self) -> Self {
        self.fail_health = true;
        self
    }

    /// Get export count
    pub fn export_count(&self) -> u32 {
        *self.export_count.read().unwrap()
    }

    /// Add a sequence
    pub fn add_sequence(&self, id: impl Into<String>, sequence: Sequence) {
        self.sequences.write().unwrap().insert(id.into(), sequence);
    }

    /// Simulate latency
    async fn simulate_latency(&self) {
        if self.latency_ms > 0 {
            tokio::time::sleep(tokio::time::Duration::from_millis(self.latency_ms)).await;
        }
    }
}

impl Default for MockAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl IEcosystemAdapter for MockAdapter {
    fn adapter_type(&self) -> AdapterType {
        AdapterType::Mock
    }

    fn name(&self) -> &str {
        &self.name
    }

    async fn initialize(&mut self, _config: &AdapterConfig) -> Result<(), AdapterError> {
        self.simulate_latency().await;
        self.initialized = true;
        Ok(())
    }

    async fn health_check(&self) -> Result<HealthStatus, AdapterError> {
        self.simulate_latency().await;

        if self.fail_health {
            Err(AdapterError::ConnectionFailed("Mock health check failure".to_string()))
        } else if !self.initialized {
            Err(AdapterError::ConfigurationError("Not initialized".to_string()))
        } else {
            Ok(HealthStatus::healthy(self.latency_ms))
        }
    }

    async fn shutdown(&mut self) -> Result<(), AdapterError> {
        self.simulate_latency().await;
        self.initialized = false;
        Ok(())
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }
}

#[async_trait]
impl IDataExporter for MockAdapter {
    async fn export_sequence(&self, sequence: &Sequence) -> Result<ExportResult, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        self.simulate_latency().await;

        let external_id = format!("mock-seq-{}", uuid::Uuid::new_v4().to_string().split('-').next().unwrap());
        self.sequences.write().unwrap().insert(external_id.clone(), sequence.clone());

        *self.export_count.write().unwrap() += 1;

        Ok(ExportResult::success(&external_id)
            .with_url(format!("https://mock.example.com/sequences/{}", external_id)))
    }

    async fn export_experiment(&self, experiment: &CrisprExperiment) -> Result<ExportResult, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        self.simulate_latency().await;

        let external_id = format!("mock-exp-{}", uuid::Uuid::new_v4().to_string().split('-').next().unwrap());
        self.experiments.write().unwrap().insert(external_id.clone(), experiment.clone());

        *self.export_count.write().unwrap() += 1;

        Ok(ExportResult::success(&external_id))
    }

    async fn export_structure(&self, structure: &ProteinStructure) -> Result<ExportResult, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        self.simulate_latency().await;

        let external_id = format!("mock-struct-{}", uuid::Uuid::new_v4().to_string().split('-').next().unwrap());
        self.structures.write().unwrap().insert(external_id.clone(), structure.clone());

        *self.export_count.write().unwrap() += 1;

        Ok(ExportResult::success(&external_id))
    }

    async fn export_part(&self, part: &BioPart) -> Result<ExportResult, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        self.simulate_latency().await;

        let external_id = format!("mock-part-{}", uuid::Uuid::new_v4().to_string().split('-').next().unwrap());
        self.parts.write().unwrap().insert(external_id.clone(), part.clone());

        *self.export_count.write().unwrap() += 1;

        Ok(ExportResult::success(&external_id))
    }
}

#[async_trait]
impl IDataImporter for MockAdapter {
    async fn import_sequence(&self, external_id: &str) -> Result<Sequence, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        self.simulate_latency().await;

        self.sequences.read().unwrap()
            .get(external_id)
            .cloned()
            .ok_or_else(|| AdapterError::NotFound(format!("Sequence {} not found", external_id)))
    }

    async fn search_sequences(&self, query: &SearchQuery) -> Result<Vec<Sequence>, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        self.simulate_latency().await;

        let sequences = self.sequences.read().unwrap();
        let mut results: Vec<Sequence> = sequences.values().cloned().collect();

        // Apply text filter
        if let Some(ref text) = query.text {
            let text_lower = text.to_lowercase();
            results.retain(|seq| {
                seq.sequence_info.as_ref()
                    .and_then(|info| info.name.as_ref())
                    .map(|name| name.to_lowercase().contains(&text_lower))
                    .unwrap_or(false)
            });
        }

        // Apply limit and offset
        let start = query.offset as usize;
        let end = (query.offset + query.limit) as usize;
        if start < results.len() {
            results = results[start..end.min(results.len())].to_vec();
        } else {
            results.clear();
        }

        Ok(results)
    }

    async fn import_structure(&self, external_id: &str) -> Result<ProteinStructure, AdapterError> {
        if !self.initialized {
            return Err(AdapterError::ConfigurationError("Not initialized".to_string()));
        }

        self.simulate_latency().await;

        self.structures.read().unwrap()
            .get(external_id)
            .cloned()
            .ok_or_else(|| AdapterError::NotFound(format!("Structure {} not found", external_id)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::*;
    use crate::types::SequenceType;

    #[tokio::test]
    async fn test_mock_adapter_initialize() {
        let mut adapter = MockAdapter::new();
        let config = AdapterConfig::default();

        assert!(!adapter.is_initialized());
        adapter.initialize(&config).await.unwrap();
        assert!(adapter.is_initialized());
    }

    #[tokio::test]
    async fn test_mock_adapter_health_check() {
        let mut adapter = MockAdapter::new();
        let config = AdapterConfig::default();

        // Should fail before init
        let result = adapter.health_check().await;
        assert!(result.is_err());

        adapter.initialize(&config).await.unwrap();

        // Should pass after init
        let status = adapter.health_check().await.unwrap();
        assert!(status.healthy);
    }

    #[tokio::test]
    async fn test_mock_adapter_fail_health() {
        let mut adapter = MockAdapter::new().fail_health();
        let config = AdapterConfig::default();
        adapter.initialize(&config).await.unwrap();

        let result = adapter.health_check().await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_mock_adapter_export_sequence() {
        let mut adapter = MockAdapter::new();
        let config = AdapterConfig::default();
        adapter.initialize(&config).await.unwrap();

        let sequence = create_sequence("Test Gene", "ATCGATCG", SequenceType::Dna).unwrap();
        let result = adapter.export_sequence(&sequence).await.unwrap();

        assert!(result.success);
        assert!(result.external_id.is_some());
        assert_eq!(adapter.export_count(), 1);
    }

    #[tokio::test]
    async fn test_mock_adapter_import_sequence() {
        let mut adapter = MockAdapter::new();
        let config = AdapterConfig::default();
        adapter.initialize(&config).await.unwrap();

        // Export first
        let sequence = create_sequence("Test Gene", "ATCGATCG", SequenceType::Dna).unwrap();
        let result = adapter.export_sequence(&sequence).await.unwrap();
        let external_id = result.external_id.unwrap();

        // Then import
        let imported = adapter.import_sequence(&external_id).await.unwrap();
        assert_eq!(imported.sequence_id, sequence.sequence_id);
    }

    #[tokio::test]
    async fn test_mock_adapter_search() {
        let mut adapter = MockAdapter::new();
        let config = AdapterConfig::default();
        adapter.initialize(&config).await.unwrap();

        // Add some sequences
        let seq1 = create_sequence("BRCA1 Gene", "ATCGATCG", SequenceType::Dna).unwrap();
        let seq2 = create_sequence("TP53 Gene", "GCTAGCTA", SequenceType::Dna).unwrap();
        adapter.export_sequence(&seq1).await.unwrap();
        adapter.export_sequence(&seq2).await.unwrap();

        // Search
        let query = SearchQuery::new("BRCA1");
        let results = adapter.search_sequences(&query).await.unwrap();

        assert_eq!(results.len(), 1);
    }
}
