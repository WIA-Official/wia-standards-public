//! Provider manager for multi-backend execution

use crate::error::{QuantumError, Result};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use super::provider::{JobHandle, JobRequest, JobResult, ProviderConfig, QuantumProvider};
use crate::types::{BackendInfo, JobStatus};

/// Manager configuration
#[derive(Debug, Clone)]
pub struct ManagerConfig {
    /// Auto-connect on registration
    pub auto_connect: bool,
    /// Enable fallback to other providers
    pub enable_fallback: bool,
    /// Fallback provider order
    pub fallback_order: Vec<String>,
    /// Maximum retry attempts
    pub max_retries: u32,
    /// Retry delay in milliseconds
    pub retry_delay_ms: u64,
}

impl Default for ManagerConfig {
    fn default() -> Self {
        Self {
            auto_connect: true,
            enable_fallback: true,
            fallback_order: vec!["local".to_string()],
            max_retries: 3,
            retry_delay_ms: 1000,
        }
    }
}

/// Provider manager for managing multiple quantum providers
pub struct ProviderManager {
    providers: RwLock<HashMap<String, Box<dyn QuantumProvider>>>,
    default_provider: RwLock<Option<String>>,
    config: ManagerConfig,
}

impl ProviderManager {
    /// Create a new provider manager
    pub fn new() -> Self {
        Self {
            providers: RwLock::new(HashMap::new()),
            default_provider: RwLock::new(None),
            config: ManagerConfig::default(),
        }
    }

    /// Create with custom configuration
    pub fn with_config(config: ManagerConfig) -> Self {
        Self {
            providers: RwLock::new(HashMap::new()),
            default_provider: RwLock::new(None),
            config,
        }
    }

    /// Register a provider
    pub async fn register(&self, provider: Box<dyn QuantumProvider>) -> Result<()> {
        let id = provider.id().to_string();
        let mut providers = self.providers.write().await;

        if providers.contains_key(&id) {
            return Err(QuantumError::InvalidConfig(format!(
                "Provider already registered: {}",
                id
            )));
        }

        providers.insert(id.clone(), provider);

        // Set as default if first provider
        let mut default = self.default_provider.write().await;
        if default.is_none() {
            *default = Some(id);
        }

        Ok(())
    }

    /// Unregister a provider
    pub async fn unregister(&self, id: &str) -> Result<()> {
        let mut providers = self.providers.write().await;

        if providers.remove(id).is_none() {
            return Err(QuantumError::InvalidConfig(format!(
                "Provider not found: {}",
                id
            )));
        }

        // Clear default if it was this provider
        let mut default = self.default_provider.write().await;
        if default.as_deref() == Some(id) {
            *default = providers.keys().next().cloned();
        }

        Ok(())
    }

    /// Get a provider by ID
    pub async fn get(&self, id: &str) -> Option<String> {
        let providers = self.providers.read().await;
        if providers.contains_key(id) {
            Some(id.to_string())
        } else {
            None
        }
    }

    /// List all registered providers
    pub async fn list(&self) -> Vec<String> {
        let providers = self.providers.read().await;
        providers.keys().cloned().collect()
    }

    /// Set default provider
    pub async fn set_default(&self, id: &str) -> Result<()> {
        let providers = self.providers.read().await;
        if !providers.contains_key(id) {
            return Err(QuantumError::InvalidConfig(format!(
                "Provider not found: {}",
                id
            )));
        }

        let mut default = self.default_provider.write().await;
        *default = Some(id.to_string());
        Ok(())
    }

    /// Get default provider ID
    pub async fn default_provider(&self) -> Option<String> {
        self.default_provider.read().await.clone()
    }

    /// Connect all providers
    pub async fn connect_all(&self, configs: HashMap<String, ProviderConfig>) -> Result<()> {
        let mut providers = self.providers.write().await;

        for (id, provider) in providers.iter_mut() {
            let config = configs.get(id).cloned().unwrap_or_default();
            provider.connect(config).await?;
        }

        Ok(())
    }

    /// Disconnect all providers
    pub async fn disconnect_all(&self) -> Result<()> {
        let mut providers = self.providers.write().await;

        for provider in providers.values_mut() {
            provider.disconnect().await?;
        }

        Ok(())
    }

    /// Get all available backends across all providers
    pub async fn get_all_backends(&self) -> Result<Vec<(String, BackendInfo)>> {
        let providers = self.providers.read().await;
        let mut all_backends = Vec::new();

        for (id, provider) in providers.iter() {
            if provider.is_connected() {
                let backends = provider.get_backends().await?;
                for backend in backends {
                    all_backends.push((id.clone(), backend));
                }
            }
        }

        Ok(all_backends)
    }

    /// Submit job to default provider
    pub async fn submit(&self, job: JobRequest) -> Result<JobHandle> {
        let default = self.default_provider.read().await;
        let provider_id = default.as_ref().ok_or_else(|| {
            QuantumError::InvalidConfig("No default provider set".to_string())
        })?;

        self.submit_to(provider_id, job).await
    }

    /// Submit job to specific provider
    pub async fn submit_to(&self, provider_id: &str, job: JobRequest) -> Result<JobHandle> {
        let providers = self.providers.read().await;
        let provider = providers.get(provider_id).ok_or_else(|| {
            QuantumError::InvalidConfig(format!("Provider not found: {}", provider_id))
        })?;

        if !provider.is_connected() {
            return Err(QuantumError::ConnectionError(format!(
                "Provider not connected: {}",
                provider_id
            )));
        }

        provider.submit_job(job).await
    }

    /// Submit jobs to multiple providers
    pub async fn submit_multi(&self, jobs: Vec<(String, JobRequest)>) -> Result<Vec<JobHandle>> {
        let mut handles = Vec::new();

        for (provider_id, job) in jobs {
            let handle = self.submit_to(&provider_id, job).await?;
            handles.push(handle);
        }

        Ok(handles)
    }

    /// Get job status
    pub async fn get_job_status(&self, provider_id: &str, job_id: &str) -> Result<JobStatus> {
        let providers = self.providers.read().await;
        let provider = providers.get(provider_id).ok_or_else(|| {
            QuantumError::InvalidConfig(format!("Provider not found: {}", provider_id))
        })?;

        provider.get_job_status(job_id).await
    }

    /// Get job result
    pub async fn get_job_result(&self, provider_id: &str, job_id: &str) -> Result<JobResult> {
        let providers = self.providers.read().await;
        let provider = providers.get(provider_id).ok_or_else(|| {
            QuantumError::InvalidConfig(format!("Provider not found: {}", provider_id))
        })?;

        provider.get_job_result(job_id).await
    }

    /// Cancel a job
    pub async fn cancel_job(&self, provider_id: &str, job_id: &str) -> Result<()> {
        let providers = self.providers.read().await;
        let provider = providers.get(provider_id).ok_or_else(|| {
            QuantumError::InvalidConfig(format!("Provider not found: {}", provider_id))
        })?;

        provider.cancel_job(job_id).await
    }
}

impl Default for ProviderManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Job monitor for tracking job status
pub struct JobMonitor {
    manager: Arc<ProviderManager>,
    tracked_jobs: RwLock<HashMap<String, JobHandle>>,
}

impl JobMonitor {
    /// Create a new job monitor
    pub fn new(manager: Arc<ProviderManager>) -> Self {
        Self {
            manager,
            tracked_jobs: RwLock::new(HashMap::new()),
        }
    }

    /// Track a job
    pub async fn track(&self, handle: JobHandle) {
        let mut jobs = self.tracked_jobs.write().await;
        jobs.insert(handle.job_id.clone(), handle);
    }

    /// Get status of tracked job
    pub async fn status(&self, job_id: &str) -> Result<JobStatus> {
        let jobs = self.tracked_jobs.read().await;
        let handle = jobs.get(job_id).ok_or_else(|| {
            QuantumError::JobError(format!("Job not tracked: {}", job_id))
        })?;

        self.manager
            .get_job_status(&handle.provider_id, job_id)
            .await
    }

    /// Wait for job completion
    pub async fn wait(&self, job_id: &str) -> Result<JobResult> {
        let jobs = self.tracked_jobs.read().await;
        let handle = jobs.get(job_id).ok_or_else(|| {
            QuantumError::JobError(format!("Job not tracked: {}", job_id))
        })?;
        let provider_id = handle.provider_id.clone();
        drop(jobs);

        // Poll until complete
        loop {
            let status = self.manager.get_job_status(&provider_id, job_id).await?;

            match status {
                JobStatus::Completed => {
                    return self.manager.get_job_result(&provider_id, job_id).await;
                }
                JobStatus::Failed => {
                    return Err(QuantumError::ExecutionError("Job failed".to_string()));
                }
                JobStatus::Cancelled => {
                    return Err(QuantumError::ExecutionError("Job cancelled".to_string()));
                }
                _ => {
                    tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
                }
            }
        }
    }

    /// Wait for all tracked jobs
    pub async fn wait_all(&self) -> Result<Vec<JobResult>> {
        let jobs = self.tracked_jobs.read().await;
        let job_ids: Vec<String> = jobs.keys().cloned().collect();
        drop(jobs);

        let mut results = Vec::new();
        for job_id in job_ids {
            let result = self.wait(&job_id).await?;
            results.push(result);
        }

        Ok(results)
    }

    /// Get all tracked jobs
    pub async fn tracked(&self) -> Vec<String> {
        let jobs = self.tracked_jobs.read().await;
        jobs.keys().cloned().collect()
    }
}
