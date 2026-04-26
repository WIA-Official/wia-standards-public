//! Client for WIA 3D Printing Construction API
//!
//! 弘益人間 - Technology that builds homes for all humanity

use crate::error::{Error, Result};
use crate::types::*;
use async_trait::async_trait;
use reqwest;
use serde_json;
use uuid::Uuid;

/// API client for 3D printing construction operations
#[derive(Debug, Clone)]
pub struct Client {
    base_url: String,
    api_key: String,
    http_client: reqwest::Client,
}

impl Client {
    /// Create a new client instance
    pub fn new(base_url: impl Into<String>, api_key: impl Into<String>) -> Result<Self> {
        let base_url = base_url.into();
        let api_key = api_key.into();

        if api_key.is_empty() {
            return Err(Error::InvalidInput("API key cannot be empty".into()));
        }

        Ok(Self {
            base_url,
            api_key,
            http_client: reqwest::Client::new(),
        })
    }

    /// Submit a new print job
    pub async fn submit_print_job(&self, design: &BuildingDesign) -> Result<PrintJob> {
        let url = format!("{}/api/v1/print-jobs", self.base_url);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(design)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let job: PrintJob = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(job)
    }

    /// Get print job status
    pub async fn get_print_job(&self, job_id: Uuid) -> Result<PrintJob> {
        let url = format!("{}/api/v1/print-jobs/{}", self.base_url, job_id);

        let response = self.http_client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let job: PrintJob = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(job)
    }

    /// List all print jobs
    pub async fn list_print_jobs(&self) -> Result<Vec<PrintJob>> {
        let url = format!("{}/api/v1/print-jobs", self.base_url);

        let response = self.http_client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let jobs: Vec<PrintJob> = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(jobs)
    }

    /// Pause a print job
    pub async fn pause_print_job(&self, job_id: Uuid) -> Result<PrintJob> {
        let url = format!("{}/api/v1/print-jobs/{}/pause", self.base_url, job_id);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let job: PrintJob = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(job)
    }

    /// Resume a paused print job
    pub async fn resume_print_job(&self, job_id: Uuid) -> Result<PrintJob> {
        let url = format!("{}/api/v1/print-jobs/{}/resume", self.base_url, job_id);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let job: PrintJob = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(job)
    }

    /// Get printer configuration
    pub async fn get_printer_config(&self, printer_id: &str) -> Result<PrinterConfig> {
        let url = format!("{}/api/v1/printers/{}", self.base_url, printer_id);

        let response = self.http_client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let config: PrinterConfig = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(config)
    }

    /// Submit quality check results
    pub async fn submit_quality_check(&self, job_id: Uuid, check: &QualityCheck) -> Result<QualityCheck> {
        let url = format!("{}/api/v1/print-jobs/{}/quality-checks", self.base_url, job_id);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(check)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let check: QualityCheck = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(check)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let client = Client::new("https://api.wia.global", "test-key");
        assert!(client.is_ok());
    }

    #[test]
    fn test_client_empty_api_key() {
        let client = Client::new("https://api.wia.global", "");
        assert!(client.is_err());
    }
}
