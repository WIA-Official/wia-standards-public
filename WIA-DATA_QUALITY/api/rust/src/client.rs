//! Data quality client implementation

use crate::{error::{Error, Result}, types::*};
use reqwest::Client;

#[derive(Debug, Clone)]
pub struct DataQualityClient {
    api_key: String,
    base_url: String,
    client: Client,
}

impl DataQualityClient {
    pub fn new(api_key: String) -> Self {
        Self::with_base_url(api_key, "https://api.wia.org/data-quality".to_string())
    }

    pub fn with_base_url(api_key: String, base_url: String) -> Self {
        Self {
            api_key,
            base_url,
            client: Client::new(),
        }
    }

    pub async fn assess_quality(&self, dataset_id: &str) -> Result<DataQualityReport> {
        let url = format!("{}/assess/{}", self.base_url, dataset_id);
        let response = self.client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        response.json::<DataQualityReport>().await
            .map_err(|e| Error::ParseError(e.to_string()))
    }

    pub async fn get_report(&self, report_id: &str) -> Result<DataQualityReport> {
        let url = format!("{}/reports/{}", self.base_url, report_id);
        let response = self.client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        response.json::<DataQualityReport>().await
            .map_err(|e| Error::ParseError(e.to_string()))
    }
}
