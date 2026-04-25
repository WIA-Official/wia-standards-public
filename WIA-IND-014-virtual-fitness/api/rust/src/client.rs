//! Client implementation for WIA-IND-014-virtual-fitness
use crate::error::{Error, Result};
use crate::types::*;
use crate::validators;
use reqwest::{Client as HttpClient, Method};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct Client {
    base_url: String,
    api_key: String,
    client: HttpClient,
}

impl Client {
    pub fn new(base_url: impl Into<String>, api_key: impl Into<String>) -> Result<Self> {
        let base_url = base_url.into();
        let api_key = api_key.into();
        validators::validate_url(&base_url)?;
        validators::validate_api_key(&api_key)?;
        Ok(Self { base_url, api_key, client: HttpClient::new() })
    }

    pub async fn get_resource(&self, id: Uuid) -> Result<Resource> {
        let response: ApiResponse<Resource> = self
            .request(Method::GET, &format!("/resources/{}", id), None::<&()>)
            .await?;
        response.data.ok_or(Error::ApiError("Resource not found".to_string()))
    }

    pub async fn list_resources(&self) -> Result<Vec<Resource>> {
        let response: ApiResponse<Vec<Resource>> = self
            .request(Method::GET, "/resources", None::<&()>)
            .await?;
        response.data.ok_or(Error::ApiError("No data returned".to_string()))
    }

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
            return Err(Error::HttpError(response.status()));
        }
        let data = response.json::<T>().await?;
        Ok(data)
    }
}
