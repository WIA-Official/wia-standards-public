//! WIA-HYDROPONICS Client implementation
//!
//! 弘益人間 - Growing food sustainably for humanity

use crate::error::{HydroponicsError, Result};
use crate::types::*;
use crate::validators;
use reqwest::{Client, Method};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// WIA-HYDROPONICS API Client
#[derive(Debug, Clone)]
pub struct HydroponicsClient {
    base_url: String,
    api_key: String,
    client: Client,
}

impl HydroponicsClient {
    /// Create a new HydroponicsClient instance
    pub fn new(base_url: impl Into<String>, api_key: impl Into<String>) -> Result<Self> {
        let base_url = base_url.into();
        let api_key = api_key.into();

        validators::validate_url(&base_url)?;
        validators::validate_api_key(&api_key)?;

        Ok(Self {
            base_url,
            api_key,
            client: Client::new(),
        })
    }

    /// List all hydroponic systems
    pub async fn list_systems(&self) -> Result<Vec<HydroponicSystem>> {
        let response: ApiResponse<Vec<HydroponicSystem>> = self
            .request(Method::GET, "/systems", None::<&()>)
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("No data returned".to_string()))
    }

    /// Get system by ID
    pub async fn get_system(&self, system_id: Uuid) -> Result<HydroponicSystem> {
        let response: ApiResponse<HydroponicSystem> = self
            .request(Method::GET, &format!("/systems/{}", system_id), None::<&()>)
            .await?;

        response.data.ok_or(HydroponicsError::SystemNotFound(system_id.to_string()))
    }

    /// Get plants in a system
    pub async fn get_plants(&self, system_id: Uuid) -> Result<Vec<Plant>> {
        let response: ApiResponse<Vec<Plant>> = self
            .request(Method::GET, &format!("/systems/{}/plants", system_id), None::<&()>)
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("No plants found".to_string()))
    }

    /// Add plant to system
    pub async fn add_plant(&self, plant: &Plant) -> Result<Plant> {
        let response: ApiResponse<Plant> = self
            .request(Method::POST, "/plants", Some(plant))
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("Failed to add plant".to_string()))
    }

    /// Get environment data
    pub async fn get_environment_data(&self, system_id: Uuid, hours: u32) -> Result<Vec<EnvironmentData>> {
        let response: ApiResponse<Vec<EnvironmentData>> = self
            .request(
                Method::GET,
                &format!("/systems/{}/environment?hours={}", system_id, hours),
                None::<&()>,
            )
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("No environment data found".to_string()))
    }

    /// Get nutrient data
    pub async fn get_nutrient_data(&self, system_id: Uuid, hours: u32) -> Result<Vec<NutrientData>> {
        let response: ApiResponse<Vec<NutrientData>> = self
            .request(
                Method::GET,
                &format!("/systems/{}/nutrients?hours={}", system_id, hours),
                None::<&()>,
            )
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("No nutrient data found".to_string()))
    }

    /// Adjust pH level
    pub async fn adjust_ph(&self, system_id: Uuid, target_ph: f64) -> Result<()> {
        validators::validate_ph(target_ph)?;

        #[derive(Serialize)]
        struct PhRequest {
            target_ph: f64,
        }

        let request = PhRequest { target_ph };

        let _response: ApiResponse<()> = self
            .request(Method::POST, &format!("/systems/{}/adjust-ph", system_id), Some(&request))
            .await?;

        Ok(())
    }

    /// Adjust EC level
    pub async fn adjust_ec(&self, system_id: Uuid, target_ec: f64) -> Result<()> {
        validators::validate_ec(target_ec)?;

        #[derive(Serialize)]
        struct EcRequest {
            target_ec: f64,
        }

        let request = EcRequest { target_ec };

        let _response: ApiResponse<()> = self
            .request(Method::POST, &format!("/systems/{}/adjust-ec", system_id), Some(&request))
            .await?;

        Ok(())
    }

    /// Get nutrient formulas
    pub async fn get_nutrient_formulas(&self) -> Result<Vec<NutrientFormula>> {
        let response: ApiResponse<Vec<NutrientFormula>> = self
            .request(Method::GET, "/formulas", None::<&()>)
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("No formulas found".to_string()))
    }

    /// Record harvest
    pub async fn record_harvest(&self, harvest: &HarvestRecord) -> Result<HarvestRecord> {
        let response: ApiResponse<HarvestRecord> = self
            .request(Method::POST, "/harvests", Some(harvest))
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("Failed to record harvest".to_string()))
    }

    /// Get system alerts
    pub async fn get_alerts(&self, system_id: Uuid) -> Result<Vec<SystemAlert>> {
        let response: ApiResponse<Vec<SystemAlert>> = self
            .request(Method::GET, &format!("/systems/{}/alerts", system_id), None::<&()>)
            .await?;

        response.data.ok_or(HydroponicsError::ApiError("No alerts found".to_string()))
    }

    /// Resolve alert
    pub async fn resolve_alert(&self, alert_id: Uuid) -> Result<()> {
        let _response: ApiResponse<()> = self
            .request(Method::POST, &format!("/alerts/{}/resolve", alert_id), None::<&()>)
            .await?;

        Ok(())
    }

    /// Internal request helper
    async fn request<T, B>(&self, method: Method, path: &str, body: Option<&B>) -> Result<T>
    where
        T: for<'de> Deserialize<'de>,
        B: Serialize,
    {
        let url = format!("{}{}", self.base_url, path);
        let mut request = self.client.request(method, &url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .header("Content-Type", "application/json");

        if let Some(body) = body {
            request = request.json(body);
        }

        let response = request.send().await?;

        if !response.status().is_success() {
            return Err(HydroponicsError::HttpError(response.status()));
        }

        let data = response.json::<T>().await?;
        Ok(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let result = HydroponicsClient::new("https://api.wia-hydroponics.org", "test-key");
        assert!(result.is_ok());
    }
}
