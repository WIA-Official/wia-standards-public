//! HTTP client and API implementations for the WIA-CHILD-010-predator-detection SDK
//!
//! This module contains the main client and API endpoint implementations.

use crate::error::{Error, Result};
use crate::types::*;
use crate::validators;
use reqwest::{header, Client, StatusCode};
use std::time::Duration;

/// Main WIA-CHILD-010-predator-detection SDK client
///
/// # Example
///
/// ```rust,no_run
/// use wia_child_010_predator_detection_sdk::{Child010PredatorDetectionClient, Config};
///
/// #[tokio::main]
/// async fn main() {
///     let client = Child010PredatorDetectionClient::new(Config {
///         api_key: "your-api-key".to_string(),
///         ..Default::default()
///     });
///
///     let data = client.get("id").await;
/// }
/// ```
pub struct Child010PredatorDetectionClient {
    http: Client,
    config: Config,
    base_url: String,
}

impl Child010PredatorDetectionClient {
    /// Create a new WIA-CHILD-010-predator-detection client
    pub fn new(config: Config) -> Self {
        let base_url = config
            .base_url
            .clone()
            .unwrap_or_else(|| "https://api.wiastandards.com/WIA-CHILD-010-predator-detection".to_string());

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

    /// Validate data against WIA-CHILD-010-predator-detection standards
    pub async fn validate(&self, data: &serde_json::Value) -> Result<ValidationResult> {
        validators::validate_data(data)
    }

    /// Get data by ID
    pub async fn get(&self, id: &str) -> Result<Child010PredatorDetectionData> {
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

        let result: Child010PredatorDetectionResponse = response.json().await?;
        result.data.ok_or_else(|| Error::NotFound(id.to_string()))
    }

    /// Create new data
    pub async fn create(&self, request: CreateChild010PredatorDetectionRequest) -> Result<Child010PredatorDetectionData> {
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

        let result: Child010PredatorDetectionResponse = response.json().await?;
        result.data.ok_or_else(|| Error::Unknown("No data in response".to_string()))
    }

    /// List data with pagination
    pub async fn list(&self, params: Option<ListParams>) -> Result<Child010PredatorDetectionList> {
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
