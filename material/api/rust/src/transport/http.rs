//! HTTP Transport implementation
//!
//! REST API client for the WIA Material Protocol.

use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::RwLock;

use super::{Transport, TransportConfig};
use crate::error::{MaterialError, MaterialResult};
use crate::protocol::{
    CreatePayload, CreateResponsePayload, DeletePayload, DeleteResponsePayload, ErrorCode,
    GetPayload, GetResponsePayload, MessageBuilder, QueryPayload, QueryResponsePayload,
    UpdatePayload, UpdateResponsePayload,
};
use crate::types::MaterialData;

/// HTTP transport for REST API communication
///
/// This transport implements the WIA Material Protocol over HTTP/HTTPS.
///
/// ## Example
///
/// ```rust,ignore
/// use wia_material::transport::{HttpTransport, TransportConfig};
///
/// let config = TransportConfig::new("https://api.example.com")
///     .with_api_key("your-api-key");
/// let mut transport = HttpTransport::new(config);
/// transport.connect().await?;
/// ```
pub struct HttpTransport {
    config: TransportConfig,
    connected: AtomicBool,
    materials: Arc<RwLock<Vec<MaterialData>>>,
}

impl HttpTransport {
    /// Create a new HTTP transport
    pub fn new(config: TransportConfig) -> Self {
        Self {
            config,
            connected: AtomicBool::new(false),
            materials: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Get the configuration
    pub fn config(&self) -> &TransportConfig {
        &self.config
    }

    /// Build request headers
    fn build_headers(&self) -> Vec<(&str, String)> {
        let mut headers = vec![
            ("Content-Type", "application/json".to_string()),
            ("Accept", "application/json".to_string()),
            ("User-Agent", self.config.user_agent.clone()),
        ];

        if let Some(ref api_key) = self.config.api_key {
            headers.push(("X-WIA-API-Key", api_key.clone()));
        }

        if let Some(auth) = self.config.auth_header() {
            headers.push(("Authorization", auth));
        }

        headers
    }

    /// Simulate HTTP request (for development/testing)
    /// In production, this would use reqwest or similar HTTP client
    async fn simulate_request<T: serde::Serialize>(
        &self,
        method: &str,
        path: &str,
        body: Option<&T>,
    ) -> MaterialResult<serde_json::Value> {
        if !self.connected.load(Ordering::SeqCst) {
            return Err(MaterialError::Transport(
                "Not connected. Call connect() first.".to_string(),
            ));
        }

        // Log the request (in development)
        let _url = format!("{}{}", self.config.materials_url(), path);
        let _body_json = body.map(|b| serde_json::to_string(b).ok()).flatten();

        // Simulate network latency
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;

        // Return simulated response based on method
        match method {
            "GET" => {
                if path.is_empty() || path.starts_with('?') {
                    // List materials (query string is for filtering, not individual lookup)
                    let materials = self.materials.read().await;
                    Ok(serde_json::json!({
                        "data": *materials,
                        "meta": {
                            "total_count": materials.len(),
                            "returned_count": materials.len(),
                            "offset": 0,
                            "has_more": false
                        }
                    }))
                } else {
                    // Get single material
                    let material_id = path.trim_start_matches('/');
                    let materials = self.materials.read().await;
                    if let Some(mat) = materials.iter().find(|m| m.material_id == material_id) {
                        Ok(serde_json::json!({ "data": mat }))
                    } else {
                        Err(MaterialError::NotFound(material_id.to_string()))
                    }
                }
            }
            "POST" => {
                // Create material
                Ok(serde_json::json!({ "data": body }))
            }
            "PUT" | "PATCH" => {
                // Update material
                Ok(serde_json::json!({ "data": body }))
            }
            "DELETE" => {
                let material_id = path.trim_start_matches('/');
                Ok(serde_json::json!({
                    "material_id": material_id,
                    "success": true
                }))
            }
            _ => Err(MaterialError::Transport(format!(
                "Unknown method: {}",
                method
            ))),
        }
    }
}

#[async_trait]
impl Transport for HttpTransport {
    async fn connect(&mut self) -> MaterialResult<()> {
        // In production, this would validate the connection
        // For now, just mark as connected
        self.connected.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn disconnect(&mut self) -> MaterialResult<()> {
        self.connected.store(false, Ordering::SeqCst);
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    async fn query(&self, payload: QueryPayload) -> MaterialResult<QueryResponsePayload> {
        let message = MessageBuilder::query(payload.clone());
        let _json = message.to_json()?;

        // Build query parameters
        let mut query_params = Vec::new();
        if let Some(ref mt) = payload.material_type {
            query_params.push(format!("type={}", mt));
        }
        if let Some(ref pagination) = payload.pagination {
            query_params.push(format!("offset={}", pagination.offset));
            query_params.push(format!("limit={}", pagination.limit));
        }

        let path = if query_params.is_empty() {
            String::new()
        } else {
            format!("?{}", query_params.join("&"))
        };

        let response = self
            .simulate_request::<()>("GET", &path, None)
            .await?;

        // Parse response
        let data: Vec<MaterialData> = serde_json::from_value(
            response
                .get("data")
                .cloned()
                .unwrap_or(serde_json::Value::Array(vec![])),
        )
        .unwrap_or_default();

        let total_count = data.len();

        Ok(QueryResponsePayload {
            data: data.clone(),
            meta: crate::protocol::QueryMeta {
                total_count,
                returned_count: data.len(),
                offset: payload.pagination.as_ref().map(|p| p.offset).unwrap_or(0),
                has_more: false,
            },
        })
    }

    async fn get(&self, payload: GetPayload) -> MaterialResult<GetResponsePayload> {
        let path = format!("/{}", payload.material_id);
        let response = self
            .simulate_request::<()>("GET", &path, None)
            .await?;

        let data: MaterialData = serde_json::from_value(
            response
                .get("data")
                .cloned()
                .ok_or_else(|| MaterialError::Protocol("Missing data field".to_string()))?,
        )?;

        Ok(GetResponsePayload { data })
    }

    async fn create(&self, payload: CreatePayload) -> MaterialResult<CreateResponsePayload> {
        // Store the material
        {
            let mut materials = self.materials.write().await;
            materials.push(payload.material.clone());
        }

        let _response = self
            .simulate_request("POST", "", Some(&payload.material))
            .await?;

        Ok(CreateResponsePayload {
            data: payload.material,
        })
    }

    async fn update(&self, payload: UpdatePayload) -> MaterialResult<UpdateResponsePayload> {
        // Update in storage
        {
            let mut materials = self.materials.write().await;
            if let Some(pos) = materials
                .iter()
                .position(|m| m.material_id == payload.material_id)
            {
                materials[pos] = payload.material.clone();
            } else {
                return Err(MaterialError::NotFound(payload.material_id));
            }
        }

        let path = format!("/{}", payload.material.material_id);
        let _response = self
            .simulate_request("PUT", &path, Some(&payload.material))
            .await?;

        Ok(UpdateResponsePayload {
            data: payload.material,
        })
    }

    async fn delete(&self, payload: DeletePayload) -> MaterialResult<DeleteResponsePayload> {
        // Remove from storage
        {
            let mut materials = self.materials.write().await;
            if let Some(pos) = materials
                .iter()
                .position(|m| m.material_id == payload.material_id)
            {
                materials.remove(pos);
            }
        }

        let path = format!("/{}", payload.material_id);
        let _response = self
            .simulate_request::<()>("DELETE", &path, None)
            .await?;

        Ok(DeleteResponsePayload {
            material_id: payload.material_id,
            success: true,
        })
    }
}

/// HTTP response wrapper
#[derive(Debug, Clone)]
pub struct HttpResponse {
    /// HTTP status code
    pub status: u16,

    /// Response headers
    pub headers: std::collections::HashMap<String, String>,

    /// Response body
    pub body: String,
}

impl HttpResponse {
    /// Check if response is successful (2xx)
    pub fn is_success(&self) -> bool {
        (200..300).contains(&self.status)
    }

    /// Parse body as JSON
    pub fn json<T: serde::de::DeserializeOwned>(&self) -> MaterialResult<T> {
        serde_json::from_str(&self.body).map_err(|e| MaterialError::Parse(e.to_string()))
    }

    /// Get error code from response
    pub fn error_code(&self) -> Option<ErrorCode> {
        match self.status {
            400 => Some(ErrorCode::InvalidRequest),
            401 => Some(ErrorCode::AuthRequired),
            403 => Some(ErrorCode::Forbidden),
            404 => Some(ErrorCode::NotFound),
            409 => Some(ErrorCode::DuplicateId),
            422 => Some(ErrorCode::ValidationError),
            429 => Some(ErrorCode::RateLimited),
            500 => Some(ErrorCode::InternalError),
            503 => Some(ErrorCode::ServiceUnavailable),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_http_transport_connect() {
        let config = TransportConfig::new("https://api.example.com");
        let mut transport = HttpTransport::new(config);

        assert!(!transport.is_connected());
        transport.connect().await.unwrap();
        assert!(transport.is_connected());
        transport.disconnect().await.unwrap();
        assert!(!transport.is_connected());
    }

    #[tokio::test]
    async fn test_http_transport_query() {
        let config = TransportConfig::new("https://api.example.com");
        let mut transport = HttpTransport::new(config);
        transport.connect().await.unwrap();

        let payload = QueryPayload::default();
        let response = transport.query(payload).await.unwrap();

        assert_eq!(response.data.len(), 0);
        assert_eq!(response.meta.total_count, 0);
    }

    #[tokio::test]
    async fn test_http_transport_crud() {
        use crate::core::MaterialBuilder;
        use crate::MaterialType;

        let config = TransportConfig::new("https://api.example.com");
        let mut transport = HttpTransport::new(config);
        transport.connect().await.unwrap();

        // Create
        let material = MaterialBuilder::new()
            .material_type(MaterialType::Superconductor)
            .name("Test SC")
            .formula("YBa2Cu3O7")
            .build()
            .unwrap();

        let create_response = transport
            .create(CreatePayload {
                material: material.clone(),
            })
            .await
            .unwrap();

        assert_eq!(create_response.data.identity.name, "Test SC");

        // Get
        let get_response = transport
            .get(GetPayload {
                material_id: material.material_id.clone(),
                fields: None,
            })
            .await
            .unwrap();

        assert_eq!(get_response.data.identity.name, "Test SC");

        // Delete
        let delete_response = transport
            .delete(DeletePayload {
                material_id: material.material_id.clone(),
            })
            .await
            .unwrap();

        assert!(delete_response.success);
    }

    #[test]
    fn test_http_response() {
        let response = HttpResponse {
            status: 200,
            headers: std::collections::HashMap::new(),
            body: r#"{"key": "value"}"#.to_string(),
        };

        assert!(response.is_success());
        assert!(response.error_code().is_none());

        let error_response = HttpResponse {
            status: 404,
            headers: std::collections::HashMap::new(),
            body: "Not found".to_string(),
        };

        assert!(!error_response.is_success());
        assert_eq!(error_response.error_code(), Some(ErrorCode::NotFound));
    }
}
