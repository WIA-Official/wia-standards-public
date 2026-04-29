//! HTTP client and API implementations for the 6g-communication SDK
//!
//! This module contains the main client and API endpoint implementations.

use crate::error::{Error, Result};
use crate::types::*;
use crate::validators;
use reqwest::{header, Client, StatusCode};
use std::time::Duration;

/// Main 6g-communication SDK client
///
/// # Example
///
/// ```rust,no_run
/// use 6g_communication_sdk::{SixGCommunicationClient, Config};
///
/// #[tokio::main]
/// async fn main() {
///     let client = SixGCommunicationClient::new(Config {
///         api_key: "your-api-key".to_string(),
///         ..Default::default()
///     });
///
///     let data = client.get("id").await;
/// }
/// ```
pub struct SixGCommunicationClient {
    http: Client,
    config: Config,
    base_url: String,
}

impl SixGCommunicationClient {
    /// Create a new 6g-communication client
    pub fn new(config: Config) -> Self {
        let base_url = config
            .base_url
            .clone()
            .unwrap_or_else(|| "https://api.wiastandards.com/6g-communication".to_string());

        let http = Client::builder()
            .timeout(Duration::from_secs(config.timeout_secs))
            .build()
            .expect("Failed to create HTTP client");

        Self {
            http,
            config,
            base_url,
        }
    }

    /// Validate data against 6g-communication standards
    pub async fn validate(&self, data: &serde_json::Value) -> Result<ValidationResult> {
        validators::validate_data(data)
    }

    /// Get data by ID
    pub async fn get(&self, id: &str) -> Result<SixGCommunicationData> {
        let url = format!("{}/data/{}", self.base_url, id);

        let response = self
            .http
            .get(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .send()
            .await?;

        if !response.status().is_success() {
            return Err(self.handle_error_response(response).await);
        }

        let result: SixGCommunicationResponse = response.json().await?;
        result.data.ok_or_else(|| Error::NotFound(id.to_string()))
    }

    /// Create new data
    pub async fn create(&self, request: CreateSixGCommunicationRequest) -> Result<SixGCommunicationData> {
        let url = format!("{}/data", self.base_url);

        let response = self
            .http
            .post(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .header(header::CONTENT_TYPE, "application/json")
            .json(&request)
            .send()
            .await?;

        if !response.status().is_success() {
            return Err(self.handle_error_response(response).await);
        }

        let result: SixGCommunicationResponse = response.json().await?;
        result.data.ok_or_else(|| Error::Unknown("No data in response".to_string()))
    }

    /// List data with pagination
    pub async fn list(&self, params: Option<ListParams>) -> Result<SixGCommunicationList> {
        let url = format!("{}/data", self.base_url);
        let params = params.unwrap_or_default();

        let response = self
            .http
            .get(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .query(&params)
            .send()
            .await?;

        if !response.status().is_success() {
            return Err(self.handle_error_response(response).await);
        }

        Ok(response.json().await?)
    }

    /// Delete data by ID
    pub async fn delete(&self, id: &str) -> Result<()> {
        let url = format!("{}/data/{}", self.base_url, id);

        let response = self
            .http
            .delete(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .send()
            .await?;

        if !response.status().is_success() {
            return Err(self.handle_error_response(response).await);
        }

        Ok(())
    }

    async fn handle_error_response(&self, response: reqwest::Response) -> Error {
        let status = response.status();

        match status {
            StatusCode::UNAUTHORIZED => Error::Authentication("Invalid API key".to_string()),
            StatusCode::NOT_FOUND => Error::NotFound("Resource not found".to_string()),
            StatusCode::TOO_MANY_REQUESTS => Error::RateLimited { retry_after: None },
            _ => {
                if let Ok(body) = response.text().await {
                    Error::Unknown(format!("API error: {}", body))
                } else {
                    Error::Unknown(format!("HTTP {}", status))
                }
            }
        }
    }
}
