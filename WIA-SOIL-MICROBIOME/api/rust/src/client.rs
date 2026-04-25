//! HTTP client and API implementations for the WIA-SOIL-MICROBIOME SDK
//!
//! This module contains the main client and API endpoint implementations.
//!
//! 弘益人間 (Benefit All Humanity)

use crate::error::{ApiError, Error, Result};
use crate::types::*;
use reqwest::{header, Client, StatusCode};
use std::time::Duration;

/// Main WIA-SOIL-MICROBIOME SDK client
///
/// # Example
///
/// ```rust,no_run
/// use wia_soil_microbiome_sdk::{WiaSoilMicrobiomeClient, Config, Environment};
///
/// #[tokio::main]
/// async fn main() {
///     let client = WiaSoilMicrobiomeClient::new(Config {
///         api_key: "your-api-key".to_string(),
///         environment: Environment::Production,
///         ..Default::default()
///     });
///
///     let samples = client.samples().list(None).await;
/// }
/// ```
pub struct WiaSoilMicrobiomeClient {
    http: Client,
    config: Config,
    base_url: String,
}

impl WiaSoilMicrobiomeClient {
    /// Create a new WIA-SOIL-MICROBIOME client
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

    /// Get the samples API
    pub fn samples(&self) -> SamplesApi<'_> {
        SamplesApi { client: self }
    }

    /// Get the microbiome API
    pub fn microbiome(&self) -> MicrobiomeApi<'_> {
        MicrobiomeApi { client: self }
    }

    /// Get the health index API
    pub fn health_index(&self) -> HealthIndexApi<'_> {
        HealthIndexApi { client: self }
    }

    /// Get the carbon API
    pub fn carbon(&self) -> CarbonApi<'_> {
        CarbonApi { client: self }
    }

    /// Get the interventions API
    pub fn interventions(&self) -> InterventionsApi<'_> {
        InterventionsApi { client: self }
    }

    /// Get the reports API
    pub fn reports(&self) -> ReportsApi<'_> {
        ReportsApi { client: self }
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
// Samples API
// ============================================================================

/// Samples API
pub struct SamplesApi<'a> {
    client: &'a WiaSoilMicrobiomeClient,
}

impl<'a> SamplesApi<'a> {
    /// List all samples
    pub async fn list(
        &self,
        params: Option<PaginationParams>,
    ) -> Result<PaginatedResponse<SampleSummary>> {
        let mut path = "/samples".to_string();

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

    /// Get a specific sample
    pub async fn get(&self, sample_id: &str) -> Result<SoilSample> {
        let path = format!("/samples/{}", sample_id);
        self.client.get(&path).await
    }

    /// Submit a new sample
    pub async fn submit(&self, request: SubmitSampleRequest) -> Result<SampleSummary> {
        self.client.post("/samples", &request).await
    }

    /// Update a sample
    pub async fn update(&self, sample_id: &str, sample: SoilSample) -> Result<SampleSummary> {
        let path = format!("/samples/{}", sample_id);
        self.client.put(&path, &sample).await
    }

    /// Delete a sample
    pub async fn delete(&self, sample_id: &str) -> Result<serde_json::Value> {
        let path = format!("/samples/{}", sample_id);
        self.client.delete(&path).await
    }

    /// Get sample analysis status
    pub async fn get_status(
        &self,
        sample_id: &str,
    ) -> Result<serde_json::Value> {
        let path = format!("/samples/{}/status", sample_id);
        self.client.get(&path).await
    }
}

// ============================================================================
// Microbiome API
// ============================================================================

/// Microbiome API
pub struct MicrobiomeApi<'a> {
    client: &'a WiaSoilMicrobiomeClient,
}

impl<'a> MicrobiomeApi<'a> {
    /// Get microbiome profile for a sample
    pub async fn get_profile(&self, sample_id: &str) -> Result<MicrobiomeProfile> {
        let path = format!("/samples/{}/microbiome", sample_id);
        self.client.get(&path).await
    }

    /// Request new analysis
    pub async fn request_analysis(&self, request: AnalysisRequest) -> Result<serde_json::Value> {
        self.client.post("/microbiome/analyze", &request).await
    }

    /// Get diversity metrics for a sample
    pub async fn get_diversity(&self, sample_id: &str) -> Result<DiversityIndex> {
        let path = format!("/samples/{}/microbiome/diversity", sample_id);
        self.client.get(&path).await
    }

    /// Compare multiple samples
    pub async fn compare(&self, sample_ids: Vec<String>) -> Result<serde_json::Value> {
        #[derive(serde::Serialize)]
        struct CompareRequest {
            sample_ids: Vec<String>,
        }
        self.client
            .post("/microbiome/compare", &CompareRequest { sample_ids })
            .await
    }

    /// Get functional group analysis
    pub async fn get_functional_groups(&self, sample_id: &str) -> Result<serde_json::Value> {
        let path = format!("/samples/{}/microbiome/functional-groups", sample_id);
        self.client.get(&path).await
    }
}

// ============================================================================
// Health Index API
// ============================================================================

/// Health Index API
pub struct HealthIndexApi<'a> {
    client: &'a WiaSoilMicrobiomeClient,
}

impl<'a> HealthIndexApi<'a> {
    /// Calculate soil health index
    pub async fn calculate(&self, request: CalculateHealthIndexRequest) -> Result<SoilHealthIndex> {
        self.client.post("/health-index/calculate", &request).await
    }

    /// Get health index for a sample
    pub async fn get(&self, sample_id: &str) -> Result<SoilHealthIndex> {
        let path = format!("/samples/{}/health-index", sample_id);
        self.client.get(&path).await
    }

    /// Get health trends over time
    pub async fn get_trends(
        &self,
        sample_ids: Vec<String>,
        start_date: Option<String>,
        end_date: Option<String>,
    ) -> Result<serde_json::Value> {
        #[derive(serde::Serialize)]
        struct TrendsRequest {
            sample_ids: Vec<String>,
        }

        let mut path = "/health-index/trends".to_string();
        let mut query_parts = Vec::new();
        if let Some(start) = start_date {
            query_parts.push(format!("startDate={}", start));
        }
        if let Some(end) = end_date {
            query_parts.push(format!("endDate={}", end));
        }
        if !query_parts.is_empty() {
            path = format!("{}?{}", path, query_parts.join("&"));
        }

        self.client
            .post(&path, &TrendsRequest { sample_ids })
            .await
    }
}

// ============================================================================
// Carbon API
// ============================================================================

/// Carbon API
pub struct CarbonApi<'a> {
    client: &'a WiaSoilMicrobiomeClient,
}

impl<'a> CarbonApi<'a> {
    /// Submit carbon sequestration data
    pub async fn submit_sequestration(
        &self,
        sample_id: &str,
        data: CarbonSequestration,
    ) -> Result<CarbonSequestration> {
        let path = format!("/samples/{}/carbon", sample_id);
        self.client.post(&path, &data).await
    }

    /// Get carbon sequestration data
    pub async fn get_sequestration(&self, sample_id: &str) -> Result<CarbonSequestration> {
        let path = format!("/samples/{}/carbon", sample_id);
        self.client.get(&path).await
    }

    /// Calculate carbon credits
    pub async fn calculate_credits(
        &self,
        sample_id: &str,
        start_date: String,
        end_date: String,
        area: f64,
    ) -> Result<serde_json::Value> {
        #[derive(serde::Serialize)]
        struct CreditsRequest {
            start_date: String,
            end_date: String,
            area: f64,
        }

        let path = format!("/samples/{}/carbon/credits", sample_id);
        self.client
            .post(
                &path,
                &CreditsRequest {
                    start_date,
                    end_date,
                    area,
                },
            )
            .await
    }

    /// Get carbon trends across multiple samples
    pub async fn get_trends(
        &self,
        sample_ids: Vec<String>,
        start_date: Option<String>,
        end_date: Option<String>,
    ) -> Result<serde_json::Value> {
        #[derive(serde::Serialize)]
        struct TrendsRequest {
            sample_ids: Vec<String>,
            start_date: Option<String>,
            end_date: Option<String>,
        }

        self.client
            .post(
                "/carbon/trends",
                &TrendsRequest {
                    sample_ids,
                    start_date,
                    end_date,
                },
            )
            .await
    }
}

// ============================================================================
// Interventions API
// ============================================================================

/// Interventions API
pub struct InterventionsApi<'a> {
    client: &'a WiaSoilMicrobiomeClient,
}

impl<'a> InterventionsApi<'a> {
    /// Create a new intervention
    pub async fn create(&self, sample_id: &str, intervention: Intervention) -> Result<Intervention> {
        let path = format!("/samples/{}/interventions", sample_id);
        self.client.post(&path, &intervention).await
    }

    /// List interventions for a sample
    pub async fn list(&self, sample_id: &str) -> Result<Vec<Intervention>> {
        let path = format!("/samples/{}/interventions", sample_id);
        self.client.get(&path).await
    }

    /// Update an intervention
    pub async fn update(
        &self,
        sample_id: &str,
        intervention_id: &str,
        intervention: Intervention,
    ) -> Result<Intervention> {
        let path = format!("/samples/{}/interventions/{}", sample_id, intervention_id);
        self.client.put(&path, &intervention).await
    }

    /// Delete an intervention
    pub async fn delete(&self, sample_id: &str, intervention_id: &str) -> Result<serde_json::Value> {
        let path = format!("/samples/{}/interventions/{}", sample_id, intervention_id);
        self.client.delete(&path).await
    }

    /// Get intervention effectiveness
    pub async fn get_effectiveness(
        &self,
        sample_id: &str,
        intervention_id: &str,
    ) -> Result<serde_json::Value> {
        let path = format!(
            "/samples/{}/interventions/{}/effectiveness",
            sample_id, intervention_id
        );
        self.client.get(&path).await
    }
}

// ============================================================================
// Reports API
// ============================================================================

/// Reports API
pub struct ReportsApi<'a> {
    client: &'a WiaSoilMicrobiomeClient,
}

impl<'a> ReportsApi<'a> {
    /// Generate complete soil microbiome report
    pub async fn generate(&self, sample_id: &str) -> Result<SoilMicrobiomeReport> {
        #[derive(serde::Serialize)]
        struct GenerateRequest {
            sample_id: String,
        }
        self.client
            .post(
                "/reports/generate",
                &GenerateRequest {
                    sample_id: sample_id.to_string(),
                },
            )
            .await
    }

    /// Get existing report
    pub async fn get(&self, report_id: &str) -> Result<SoilMicrobiomeReport> {
        let path = format!("/reports/{}", report_id);
        self.client.get(&path).await
    }

    /// List reports
    pub async fn list(&self, params: Option<PaginationParams>) -> Result<PaginatedResponse<serde_json::Value>> {
        let mut path = "/reports".to_string();

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let config = Config {
            api_key: "test-key".to_string(),
            ..Default::default()
        };
        let client = WiaSoilMicrobiomeClient::new(config);
        assert_eq!(client.base_url, Environment::Production.base_url());
    }

    #[test]
    fn test_environment_urls() {
        assert_eq!(
            Environment::Production.base_url(),
            "https://api.soil-microbiome.wia.org/v1"
        );
        assert_eq!(
            Environment::Sandbox.base_url(),
            "https://sandbox.soil-microbiome.wia.org/v1"
        );
    }
}
