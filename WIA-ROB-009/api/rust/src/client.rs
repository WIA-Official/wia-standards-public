//! Client for robotics 009 API

use crate::error::{Result, RoboticsError};
use crate::types::*;
use reqwest::Client;

pub struct RoboticsClient {
    base_url: String,
    client: Client,
}

impl RoboticsClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: Robotics) -> Result<Robotics> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| RoboticsError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| RoboticsError::ParseError(e.to_string()))
    }
}
