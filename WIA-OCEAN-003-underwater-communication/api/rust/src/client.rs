//! Client for underwater communication API

use crate::error::{Result, UnderwaterCommunicationError};
use crate::types::*;
use reqwest::Client;

pub struct UnderwaterCommunicationClient {
    base_url: String,
    client: Client,
}

impl UnderwaterCommunicationClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn send_message(&self, message: AcousticMessage) -> Result<()> {
        let url = format!("{}/messages", self.base_url);
        self.client
            .post(&url)
            .json(&message)
            .send()
            .await
            .map_err(|e| UnderwaterCommunicationError::NetworkError(e.to_string()))?;
        Ok(())
    }
}
