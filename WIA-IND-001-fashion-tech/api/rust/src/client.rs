//! WIA-IND-001 Fashion Technology Client

use crate::error::{FashionTechError, Result};
use crate::types::*;
use crate::validators;
use reqwest::{Client, Method};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct FashionTechClient {
    base_url: String,
    api_key: String,
    client: Client,
}

impl FashionTechClient {
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

    pub async fn get_product(&self, product_id: Uuid) -> Result<Product> {
        let response: ApiResponse<Product> = self
            .request(Method::GET, &format!("/products/{}", product_id), None::<&()>)
            .await?;

        response.data.ok_or(FashionTechError::ApiError("Product not found".to_string()))
    }

    pub async fn get_size_recommendation(&self, user_id: Uuid, product_id: Uuid) -> Result<SizeRecommendation> {
        let response: ApiResponse<SizeRecommendation> = self
            .request(
                Method::GET,
                &format!("/recommendations/size?user_id={}&product_id={}", user_id, product_id),
                None::<&()>,
            )
            .await?;

        response.data.ok_or(FashionTechError::ApiError("No recommendation found".to_string()))
    }

    pub async fn create_virtual_tryon(&self, user_id: Uuid, product_id: Uuid) -> Result<VirtualTryOn> {
        #[derive(Serialize)]
        struct TryOnRequest {
            user_id: Uuid,
            product_id: Uuid,
        }

        let request = TryOnRequest { user_id, product_id };

        let response: ApiResponse<VirtualTryOn> = self
            .request(Method::POST, "/virtual-tryon", Some(&request))
            .await?;

        response.data.ok_or(FashionTechError::ApiError("Failed to create try-on".to_string()))
    }

    pub async fn get_style_profile(&self, user_id: Uuid) -> Result<StyleProfile> {
        let response: ApiResponse<StyleProfile> = self
            .request(Method::GET, &format!("/profiles/{}", user_id), None::<&()>)
            .await?;

        response.data.ok_or(FashionTechError::ApiError("Profile not found".to_string()))
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
            return Err(FashionTechError::HttpError(response.status()));
        }

        let data = response.json::<T>().await?;
        Ok(data)
    }
}
