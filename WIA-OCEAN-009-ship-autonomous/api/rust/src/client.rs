//! Client for autonomous ship API

use crate::error::{Result, AutonomousShipError};
use crate::types::*;
use reqwest::Client;

pub struct AutonomousShipClient {
    base_url: String,
    client: Client,
}

impl AutonomousShipClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: AutonomousShip) -> Result<AutonomousShip> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| AutonomousShipError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| AutonomousShipError::ParseError(e.to_string()))
    }
}
