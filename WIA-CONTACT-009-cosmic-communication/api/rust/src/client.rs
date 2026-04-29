//! Cosmic communication client implementation

use crate::{
    error::{Error, Result},
    types::*,
    validators,
};
use reqwest::Client;
use std::time::Duration;

/// Cosmic Communication API Client
#[derive(Debug, Clone)]
pub struct CosmicClient {
    api_key: String,
    base_url: String,
    client: Client,
}

impl CosmicClient {
    /// Create a new cosmic client
    pub fn new(api_key: String) -> Self {
        Self::with_base_url(api_key, "https://api.wia.org/contact-009".to_string())
    }

    /// Create client with custom base URL
    pub fn with_base_url(api_key: String, base_url: String) -> Self {
        let client = Client::builder()
            .timeout(Duration::from_secs(30))
            .build()
            .expect("Failed to create HTTP client");

        Self {
            api_key,
            base_url,
            client,
        }
    }

    /// Receive cosmic signal
    pub async fn receive_signal(&self) -> Result<CosmicSignal> {
        let url = format!("{}/signals/receive", self.base_url);

        let response = self.client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .header("User-Agent", format!("wia-contact-009-rust/{}", crate::VERSION))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        response
            .json::<CosmicSignal>()
            .await
            .map_err(|e| Error::ParseError(e.to_string()))
    }

    /// Send universal message
    pub async fn send_message(&self, message: &UniversalMessage) -> Result<String> {
        validators::validate_message(message)?;

        let url = format!("{}/messages/send", self.base_url);

        let response = self.client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .header("User-Agent", format!("wia-contact-009-rust/{}", crate::VERSION))
            .json(message)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let result: serde_json::Value = response
            .json()
            .await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(result["message_id"].as_str().unwrap_or("").to_string())
    }

    /// Analyze signal
    pub async fn analyze_signal(&self, signal_id: &str) -> Result<AnalysisResult> {
        let url = format!("{}/signals/{}/analyze", self.base_url, signal_id);

        let response = self.client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .header("User-Agent", format!("wia-contact-009-rust/{}", crate::VERSION))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        response
            .json::<AnalysisResult>()
            .await
            .map_err(|e| Error::ParseError(e.to_string()))
    }

    /// List recent signals
    pub async fn list_signals(&self, limit: u32) -> Result<Vec<CosmicSignal>> {
        let url = format!("{}/signals?limit={}", self.base_url, limit);

        let response = self.client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .header("User-Agent", format!("wia-contact-009-rust/{}", crate::VERSION))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        response
            .json::<Vec<CosmicSignal>>()
            .await
            .map_err(|e| Error::ParseError(e.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let client = CosmicClient::new("test-key".to_string());
        assert_eq!(client.api_key, "test-key");
    }

    #[test]
    fn test_custom_base_url() {
        let client = CosmicClient::with_base_url(
            "test-key".to_string(),
            "https://custom.api".to_string()
        );
        assert_eq!(client.base_url, "https://custom.api");
    }
}
