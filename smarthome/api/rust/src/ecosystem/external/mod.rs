//! # External Platform Integration
//!
//! Integration with third-party smart home platforms.

pub mod alexa;
pub mod google;
pub mod homekit;

pub use alexa::*;
pub use google::*;
pub use homekit::*;

use super::*;
use crate::error::Result;
use serde::{Deserialize, Serialize};

/// External platform type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExternalPlatform {
    Alexa,
    GoogleHome,
    HomeKit,
    SmartThings,
}

/// Platform connection status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlatformStatus {
    pub platform: ExternalPlatform,
    pub connected: bool,
    pub last_sync_ms: u64,
    pub device_count: u32,
    pub error: Option<String>,
}

/// Platform bridge for managing external connections
#[derive(Debug)]
pub struct PlatformBridge {
    alexa: Option<AlexaAdapter>,
    google: Option<GoogleHomeAdapter>,
    homekit: Option<HomeKitAdapter>,
}

impl PlatformBridge {
    pub fn new() -> Self {
        Self {
            alexa: None,
            google: None,
            homekit: None,
        }
    }

    pub fn with_alexa(mut self, adapter: AlexaAdapter) -> Self {
        self.alexa = Some(adapter);
        self
    }

    pub fn with_google(mut self, adapter: GoogleHomeAdapter) -> Self {
        self.google = Some(adapter);
        self
    }

    pub fn with_homekit(mut self, adapter: HomeKitAdapter) -> Self {
        self.homekit = Some(adapter);
        self
    }

    /// Process incoming request from any platform
    pub async fn process_request(&self, request: ExternalRequest) -> Result<ExternalResponse> {
        match request.platform {
            ExternalPlatform::Alexa => {
                if let Some(ref alexa) = self.alexa {
                    alexa.process_request(request).await
                } else {
                    Ok(ExternalResponse::error("Alexa not configured"))
                }
            }
            ExternalPlatform::GoogleHome => {
                if let Some(ref google) = self.google {
                    google.process_request(request).await
                } else {
                    Ok(ExternalResponse::error("Google Home not configured"))
                }
            }
            ExternalPlatform::HomeKit => {
                if let Some(ref homekit) = self.homekit {
                    homekit.process_request(request).await
                } else {
                    Ok(ExternalResponse::error("HomeKit not configured"))
                }
            }
            _ => Ok(ExternalResponse::error("Platform not supported")),
        }
    }

    /// Get status of all platforms
    pub fn get_status(&self) -> Vec<PlatformStatus> {
        let mut statuses = Vec::new();

        if let Some(ref alexa) = self.alexa {
            statuses.push(alexa.get_status());
        }

        if let Some(ref google) = self.google {
            statuses.push(google.get_status());
        }

        if let Some(ref homekit) = self.homekit {
            statuses.push(homekit.get_status());
        }

        statuses
    }
}

impl Default for PlatformBridge {
    fn default() -> Self {
        Self::new()
    }
}

/// Request from external platform
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExternalRequest {
    pub platform: ExternalPlatform,
    pub request_id: String,
    pub payload: serde_json::Value,
}

/// Response to external platform
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExternalResponse {
    pub success: bool,
    pub payload: serde_json::Value,
    pub error_message: Option<String>,
}

impl ExternalResponse {
    pub fn success(payload: serde_json::Value) -> Self {
        Self {
            success: true,
            payload,
            error_message: None,
        }
    }

    pub fn error(message: &str) -> Self {
        Self {
            success: false,
            payload: serde_json::json!({}),
            error_message: Some(message.to_string()),
        }
    }
}

/// Trait for external platform adapters
pub trait ExternalPlatformAdapter {
    /// Process incoming request
    fn process_request(
        &self,
        request: ExternalRequest,
    ) -> impl std::future::Future<Output = Result<ExternalResponse>> + Send;

    /// Get platform status
    fn get_status(&self) -> PlatformStatus;

    /// Sync devices with platform
    fn sync_devices(&mut self) -> impl std::future::Future<Output = Result<()>> + Send;
}
