//! HTTP client and API implementations for the WIA-AGING SDK
//!
//! This module contains the main client and API endpoint implementations.

use crate::error::{ApiError, Error, Result};
use crate::types::*;
use reqwest::{header, Client, StatusCode};
use std::time::Duration;

/// Main WIA-AGING SDK client
///
/// # Example
///
/// ```rust,no_run
/// use wia_aging_sdk::{WiaAgingClient, Config, Environment};
///
/// #[tokio::main]
/// async fn main() {
///     let client = WiaAgingClient::new(Config {
///         api_key: "your-api-key".to_string(),
///         environment: Environment::Production,
///         ..Default::default()
///     });
///
///     let profiles = client.profiles().list(None).await;
/// }
/// ```
pub struct WiaAgingClient {
    http: Client,
    config: Config,
    base_url: String,
}

impl WiaAgingClient {
    /// Create a new WIA-AGING client
    pub fn new(config: Config) -> Self {
        let base_url = config
            .base_url
            .clone()
            .unwrap_or_else(|| config.environment.base_url().to_string());

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

    /// Get the profiles API
    pub fn profiles(&self) -> ProfilesApi<'_> {
        ProfilesApi { client: self }
    }

    /// Get the assessments API
    pub fn assessments(&self) -> AssessmentsApi<'_> {
        AssessmentsApi { client: self }
    }

    /// Get the biomarkers API
    pub fn biomarkers(&self) -> BiomarkersApi<'_> {
        BiomarkersApi { client: self }
    }

    /// Get the interventions API
    pub fn interventions(&self) -> InterventionsApi<'_> {
        InterventionsApi { client: self }
    }

    /// Make an authenticated GET request
    async fn get<T: serde::de::DeserializeOwned>(&self, path: &str) -> Result<T> {
        let url = format!("{}{}", self.base_url, path);

        let response = self
            .http
            .get(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .header(header::ACCEPT, "application/json")
            .send()
            .await?;

        self.handle_response(response).await
    }

    /// Make an authenticated POST request
    async fn post<T, B>(&self, path: &str, body: &B) -> Result<T>
    where
        T: serde::de::DeserializeOwned,
        B: serde::Serialize,
    {
        let url = format!("{}{}", self.base_url, path);

        let response = self
            .http
            .post(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .header(header::CONTENT_TYPE, "application/json")
            .header(header::ACCEPT, "application/json")
            .json(body)
            .send()
            .await?;

        self.handle_response(response).await
    }

    /// Make an authenticated PUT request
    async fn put<T, B>(&self, path: &str, body: &B) -> Result<T>
    where
        T: serde::de::DeserializeOwned,
        B: serde::Serialize,
    {
        let url = format!("{}{}", self.base_url, path);

        let response = self
            .http
            .put(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .header(header::CONTENT_TYPE, "application/json")
            .header(header::ACCEPT, "application/json")
            .json(body)
            .send()
            .await?;

        self.handle_response(response).await
    }

    /// Make an authenticated DELETE request
    async fn delete<T: serde::de::DeserializeOwned>(&self, path: &str) -> Result<T> {
        let url = format!("{}{}", self.base_url, path);

        let response = self
            .http
            .delete(&url)
            .header(header::AUTHORIZATION, format!("Bearer {}", self.config.api_key))
            .header(header::ACCEPT, "application/json")
            .send()
            .await?;

        self.handle_response(response).await
    }

    /// Handle API response
    async fn handle_response<T: serde::de::DeserializeOwned>(
        &self,
        response: reqwest::Response,
    ) -> Result<T> {
        let status = response.status();

        match status {
            StatusCode::OK | StatusCode::CREATED => {
                let body = response.json::<T>().await?;
                Ok(body)
            }
            StatusCode::UNAUTHORIZED => {
                Err(Error::Authentication("Invalid or expired token".to_string()))
            }
            StatusCode::FORBIDDEN => {
                Err(Error::Authentication("Insufficient permissions".to_string()))
            }
            StatusCode::NOT_FOUND => {
                Err(Error::NotFound("Resource not found".to_string()))
            }
            StatusCode::TOO_MANY_REQUESTS => {
                let retry_after = response
                    .headers()
                    .get("Retry-After")
                    .and_then(|v| v.to_str().ok())
                    .and_then(|s| s.parse().ok());
                Err(Error::RateLimited { retry_after })
            }
            _ => {
                // Try to parse error response
                match response.json::<serde_json::Value>().await {
                    Ok(json) => {
                        if let Some(error) = json.get("error") {
                            let api_error: ApiError = serde_json::from_value(error.clone())?;
                            Err(Error::Api(api_error))
                        } else {
                            Err(Error::Unknown(format!("HTTP {}: {:?}", status, json)))
                        }
                    }
                    Err(_) => Err(Error::Unknown(format!("HTTP {}", status))),
                }
            }
        }
    }
}

// ============================================================================
// Profiles API
// ============================================================================

/// Profiles API
pub struct ProfilesApi<'a> {
    client: &'a WiaAgingClient,
}

impl<'a> ProfilesApi<'a> {
    /// List all profiles
    pub async fn list(
        &self,
        params: Option<PaginationParams>,
    ) -> Result<PaginatedResponse<ProfileSummary>> {
        let mut path = "/profiles".to_string();

        if let Some(p) = params {
            let mut query_parts = Vec::new();
            if let Some(limit) = p.limit {
                query_parts.push(format!("limit={}", limit));
            }
            if let Some(offset) = p.offset {
                query_parts.push(format!("offset={}", offset));
            }
            if !query_parts.is_empty() {
                path = format!("{}?{}", path, query_parts.join("&"));
            }
        }

        self.client.get(&path).await
    }

    /// Get a specific profile
    pub async fn get(&self, profile_id: &str) -> Result<AgingProfile> {
        let path = format!("/profiles/{}", profile_id);
        self.client.get(&path).await
    }

    /// Create a new profile
    pub async fn create(&self, request: CreateProfileRequest) -> Result<ProfileSummary> {
        self.client.post("/profiles", &request).await
    }

    /// Update a profile
    pub async fn update(
        &self,
        profile_id: &str,
        request: CreateProfileRequest,
    ) -> Result<ProfileSummary> {
        let path = format!("/profiles/{}", profile_id);
        self.client.put(&path, &request).await
    }

    /// Delete a profile
    pub async fn delete(&self, profile_id: &str) -> Result<serde_json::Value> {
        let path = format!("/profiles/{}", profile_id);
        self.client.delete(&path).await
    }
}

// ============================================================================
// Assessments API
// ============================================================================

/// Assessments API
pub struct AssessmentsApi<'a> {
    client: &'a WiaAgingClient,
}

impl<'a> AssessmentsApi<'a> {
    /// Create a new assessment
    pub async fn create(
        &self,
        profile_id: &str,
        request: CreateAssessmentRequest,
    ) -> Result<AssessmentResult> {
        let path = format!("/profiles/{}/assessments", profile_id);
        self.client.post(&path, &request).await
    }

    /// List assessments for a profile
    pub async fn list(
        &self,
        profile_id: &str,
        params: Option<PaginationParams>,
    ) -> Result<PaginatedResponse<AssessmentResult>> {
        let mut path = format!("/profiles/{}/assessments", profile_id);

        if let Some(p) = params {
            let mut query_parts = Vec::new();
            if let Some(limit) = p.limit {
                query_parts.push(format!("limit={}", limit));
            }
            if let Some(offset) = p.offset {
                query_parts.push(format!("offset={}", offset));
            }
            if !query_parts.is_empty() {
                path = format!("{}?{}", path, query_parts.join("&"));
            }
        }

        self.client.get(&path).await
    }

    /// Get a specific assessment
    pub async fn get(&self, profile_id: &str, assessment_id: &str) -> Result<AssessmentResult> {
        let path = format!("/profiles/{}/assessments/{}", profile_id, assessment_id);
        self.client.get(&path).await
    }
}

// ============================================================================
// Biomarkers API
// ============================================================================

/// Batch biomarker submission request
#[derive(Debug, Clone, serde::Serialize)]
struct BatchBiomarkerRequest {
    profile_id: String,
    biomarkers: Vec<Biomarker>,
    #[serde(skip_serializing_if = "Option::is_none")]
    source: Option<String>,
}

/// Batch submission response
#[derive(Debug, Clone, serde::Deserialize)]
pub struct BatchSubmitResponse {
    /// Whether submission was successful
    pub success: bool,
    /// Number of biomarkers submitted
    pub count: usize,
}

/// Biomarker catalog item
#[derive(Debug, Clone, serde::Deserialize)]
pub struct BiomarkerCatalogItem {
    /// Biomarker code
    pub code: String,
    /// Human-readable name
    pub name: String,
    /// Unit of measurement
    pub unit: String,
    /// Category
    pub category: String,
    /// Reference range
    pub reference_range: ReferenceRange,
}

/// Biomarkers API
pub struct BiomarkersApi<'a> {
    client: &'a WiaAgingClient,
}

impl<'a> BiomarkersApi<'a> {
    /// Submit biomarkers for a profile
    pub async fn submit(
        &self,
        profile_id: &str,
        biomarkers: Vec<Biomarker>,
        source: Option<&str>,
    ) -> Result<BatchSubmitResponse> {
        let request = BatchBiomarkerRequest {
            profile_id: profile_id.to_string(),
            biomarkers,
            source: source.map(|s| s.to_string()),
        };
        self.client.post("/biomarkers/batch", &request).await
    }

    /// Get the biomarker catalog
    pub async fn catalog(&self) -> Result<Vec<BiomarkerCatalogItem>> {
        self.client.get("/biomarkers/catalog").await
    }

    /// List biomarkers for a profile
    pub async fn list(
        &self,
        profile_id: &str,
        params: Option<PaginationParams>,
    ) -> Result<PaginatedResponse<Biomarker>> {
        let mut path = format!("/profiles/{}/biomarkers", profile_id);

        if let Some(p) = params {
            let mut query_parts = Vec::new();
            if let Some(limit) = p.limit {
                query_parts.push(format!("limit={}", limit));
            }
            if let Some(offset) = p.offset {
                query_parts.push(format!("offset={}", offset));
            }
            if !query_parts.is_empty() {
                path = format!("{}?{}", path, query_parts.join("&"));
            }
        }

        self.client.get(&path).await
    }
}

// ============================================================================
// Interventions API
// ============================================================================

/// Create intervention request
#[derive(Debug, Clone, serde::Serialize)]
pub struct CreateInterventionRequest {
    /// Type of intervention
    #[serde(rename = "type")]
    pub intervention_type: InterventionType,
    /// Name of intervention
    pub name: String,
    /// Dosage
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dosage: Option<String>,
    /// Frequency
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<String>,
    /// Start date
    pub start_date: String,
    /// Notes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub notes: Option<String>,
}

/// Interventions API
pub struct InterventionsApi<'a> {
    client: &'a WiaAgingClient,
}

impl<'a> InterventionsApi<'a> {
    /// Create a new intervention
    pub async fn create(
        &self,
        profile_id: &str,
        request: CreateInterventionRequest,
    ) -> Result<Intervention> {
        let path = format!("/profiles/{}/interventions", profile_id);
        self.client.post(&path, &request).await
    }

    /// List interventions for a profile
    pub async fn list(&self, profile_id: &str) -> Result<Vec<Intervention>> {
        let path = format!("/profiles/{}/interventions", profile_id);
        self.client.get(&path).await
    }

    /// Update an intervention
    pub async fn update(
        &self,
        profile_id: &str,
        intervention_id: &str,
        request: CreateInterventionRequest,
    ) -> Result<Intervention> {
        let path = format!("/profiles/{}/interventions/{}", profile_id, intervention_id);
        self.client.put(&path, &request).await
    }

    /// Delete an intervention
    pub async fn delete(
        &self,
        profile_id: &str,
        intervention_id: &str,
    ) -> Result<serde_json::Value> {
        let path = format!("/profiles/{}/interventions/{}", profile_id, intervention_id);
        self.client.delete(&path).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let client = WiaAgingClient::new(Config {
            api_key: "test-key".to_string(),
            environment: Environment::Sandbox,
            ..Default::default()
        });

        assert_eq!(client.base_url, Environment::Sandbox.base_url());
    }

    #[test]
    fn test_environment_urls() {
        assert_eq!(
            Environment::Production.base_url(),
            "https://api.aging.wia.org/v1"
        );
        assert_eq!(
            Environment::Sandbox.base_url(),
            "https://sandbox.aging.wia.org/v1"
        );
    }
}
