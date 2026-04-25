//! Client for marine biology data API

use crate::error::{Result, MarineBiologyError};
use crate::types::*;
use reqwest::Client;

pub struct MarineBiologyClient {
    base_url: String,
    client: Client,
}

impl MarineBiologyClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn register_species(&self, species: MarineSpecies) -> Result<MarineSpecies> {
        let url = format!("{}/species", self.base_url);
        let response = self.client
            .post(&url)
            .json(&species)
            .send()
            .await
            .map_err(|e| MarineBiologyError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| MarineBiologyError::ParseError(e.to_string()))
    }

    pub async fn record_observation(&self, obs: BiologicalObservation) -> Result<()> {
        let url = format!("{}/observations", self.base_url);
        self.client
            .post(&url)
            .json(&obs)
            .send()
            .await
            .map_err(|e| MarineBiologyError::NetworkError(e.to_string()))?;
        Ok(())
    }
}
