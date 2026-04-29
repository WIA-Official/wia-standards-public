//! Client for intergenerational API

use crate::error::{Result, IntergenerationalError};
use crate::types::*;
use reqwest::Client;

pub struct IntergenerationalClient {
    base_url: String,
    client: Client,
}

impl IntergenerationalClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: Intergenerational) -> Result<Intergenerational> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| IntergenerationalError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| IntergenerationalError::ParseError(e.to_string()))
    }
}
