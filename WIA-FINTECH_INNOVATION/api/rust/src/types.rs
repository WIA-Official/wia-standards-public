//! Type definitions for the WIA-FINTECH_INNOVATION Standard
//!
//! This module contains all the data structures used in the WIA-FINTECH_INNOVATION API.

use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Configuration for the FintechInnovation client
#[derive(Debug, Clone)]
pub struct Config {
    /// API key for authentication
    pub api_key: String,
    /// Base URL (optional, uses default if not provided)
    pub base_url: Option<String>,
    /// Request timeout in seconds
    pub timeout_secs: u64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            api_key: String::new(),
            base_url: None,
            timeout_secs: 30,
        }
    }
}

/// Main data structure for WIA-FINTECH_INNOVATION
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FintechInnovationData {
    /// Unique identifier
    pub id: Uuid,
    /// Data content
    pub content: serde_json::Value,
    /// Creation timestamp (Unix milliseconds)
    pub created_at: i64,
    /// Update timestamp (Unix milliseconds)
    pub updated_at: i64,
    /// Metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

/// Request to create new data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CreateFintechInnovationRequest {
    /// Data content
    pub content: serde_json::Value,
    /// Optional metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

/// Response from API operations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FintechInnovationResponse {
    /// Success status
    pub success: bool,
    /// Response data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<FintechInnovationData>,
    /// Error message (if any)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Request ID for tracking
    pub request_id: String,
}

/// Validation result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationResult {
    /// Whether the data is valid
    pub valid: bool,
    /// Validation errors (if any)
    pub errors: Vec<ValidationError>,
    /// Warnings
    pub warnings: Vec<String>,
}

/// Individual validation error
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationError {
    /// Field that failed validation
    pub field: String,
    /// Error message
    pub message: String,
    /// Error code
    pub code: String,
}

/// List query parameters
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ListParams {
    /// Page number (0-indexed)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub page: Option<u32>,
    /// Items per page
    #[serde(skip_serializing_if = "Option::is_none")]
    pub limit: Option<u32>,
    /// Sort by field
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sort_by: Option<String>,
    /// Sort order (asc/desc)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sort_order: Option<String>,
}

/// Paginated list response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FintechInnovationList {
    /// List of items
    pub items: Vec<FintechInnovationData>,
    /// Total count
    pub total: u32,
    /// Current page
    pub page: u32,
    /// Items per page
    pub limit: u32,
}
