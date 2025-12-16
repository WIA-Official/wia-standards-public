//! Transport Layer
//!
//! HTTP, WebSocket, and gRPC transport implementations.

use serde::{Deserialize, Serialize};
use std::time::Duration;

// ============================================================================
// Transport Configuration
// ============================================================================

/// Transport type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransportType {
    Https,
    WebSocket,
    Grpc,
}

/// TLS version
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TlsVersion {
    #[serde(rename = "1.2")]
    Tls12,
    #[serde(rename = "1.3")]
    Tls13,
}

/// TLS cipher suite
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CipherSuite {
    #[serde(rename = "TLS_AES_256_GCM_SHA384")]
    TlsAes256GcmSha384,
    #[serde(rename = "TLS_CHACHA20_POLY1305_SHA256")]
    TlsChacha20Poly1305Sha256,
    #[serde(rename = "TLS_AES_128_GCM_SHA256")]
    TlsAes128GcmSha256,
}

/// TLS configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TlsConfig {
    /// Minimum TLS version
    pub min_version: TlsVersion,
    /// Maximum TLS version
    pub max_version: TlsVersion,
    /// Allowed cipher suites
    pub cipher_suites: Vec<CipherSuite>,
    /// Verify hostname
    pub verify_hostname: bool,
    /// Verify certificate expiry
    pub verify_expiry: bool,
    /// Check certificate revocation
    pub check_revocation: bool,
}

impl Default for TlsConfig {
    fn default() -> Self {
        Self {
            min_version: TlsVersion::Tls13,
            max_version: TlsVersion::Tls13,
            cipher_suites: vec![
                CipherSuite::TlsAes256GcmSha384,
                CipherSuite::TlsChacha20Poly1305Sha256,
                CipherSuite::TlsAes128GcmSha256,
            ],
            verify_hostname: true,
            verify_expiry: true,
            check_revocation: true,
        }
    }
}

/// mTLS authentication mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MtlsMode {
    Strict,
    Optional,
    Off,
}

/// mTLS configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MtlsConfig {
    /// mTLS mode
    pub mode: MtlsMode,
    /// Whether client certificate is required
    pub client_auth_required: bool,
    /// CA certificate paths
    pub ca_certificates: Vec<String>,
    /// Server certificate path
    pub server_cert_path: Option<String>,
    /// Server key path
    pub server_key_path: Option<String>,
    /// Client certificate path
    pub client_cert_path: Option<String>,
    /// Client key path
    pub client_key_path: Option<String>,
    /// Allowed organizations
    pub allowed_organizations: Vec<String>,
}

impl Default for MtlsConfig {
    fn default() -> Self {
        Self {
            mode: MtlsMode::Strict,
            client_auth_required: true,
            ca_certificates: vec![],
            server_cert_path: None,
            server_key_path: None,
            client_cert_path: None,
            client_key_path: None,
            allowed_organizations: vec![],
        }
    }
}

/// Post-Quantum Cryptography hybrid mode configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PqcHybridConfig {
    /// Whether PQC hybrid mode is enabled
    pub enabled: bool,
    /// Key Encapsulation Mechanism algorithm
    pub kem_algorithm: String,
    /// Classical fallback algorithm
    pub classical_fallback: String,
    /// Signature algorithm
    pub signature_algorithm: String,
}

impl Default for PqcHybridConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            kem_algorithm: "Kyber768".to_string(),
            classical_fallback: "X25519".to_string(),
            signature_algorithm: "Dilithium3".to_string(),
        }
    }
}

/// Transport configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransportConfig {
    /// Transport type
    pub transport_type: TransportType,
    /// Base URL
    pub base_url: String,
    /// Connection timeout
    pub connect_timeout_ms: u64,
    /// Request timeout
    pub request_timeout_ms: u64,
    /// TLS configuration
    pub tls: TlsConfig,
    /// mTLS configuration
    pub mtls: MtlsConfig,
    /// PQC hybrid configuration
    pub pqc: PqcHybridConfig,
    /// Maximum retries
    pub max_retries: u32,
    /// Retry backoff configuration
    pub retry_backoff: RetryBackoff,
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            transport_type: TransportType::Https,
            base_url: "https://localhost:8443".to_string(),
            connect_timeout_ms: 5000,
            request_timeout_ms: 30000,
            tls: TlsConfig::default(),
            mtls: MtlsConfig::default(),
            pqc: PqcHybridConfig::default(),
            max_retries: 3,
            retry_backoff: RetryBackoff::default(),
        }
    }
}

impl TransportConfig {
    /// Create HTTPS transport config
    pub fn https(base_url: impl Into<String>) -> Self {
        Self {
            transport_type: TransportType::Https,
            base_url: base_url.into(),
            ..Default::default()
        }
    }

    /// Create WebSocket transport config
    pub fn websocket(base_url: impl Into<String>) -> Self {
        Self {
            transport_type: TransportType::WebSocket,
            base_url: base_url.into(),
            ..Default::default()
        }
    }

    /// Create gRPC transport config
    pub fn grpc(base_url: impl Into<String>) -> Self {
        Self {
            transport_type: TransportType::Grpc,
            base_url: base_url.into(),
            ..Default::default()
        }
    }

    /// Get connect timeout as Duration
    pub fn connect_timeout(&self) -> Duration {
        Duration::from_millis(self.connect_timeout_ms)
    }

    /// Get request timeout as Duration
    pub fn request_timeout(&self) -> Duration {
        Duration::from_millis(self.request_timeout_ms)
    }
}

// ============================================================================
// Retry Configuration
// ============================================================================

/// Retry backoff type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BackoffType {
    Constant,
    Linear,
    Exponential,
}

/// Retry backoff configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RetryBackoff {
    /// Backoff type
    pub backoff_type: BackoffType,
    /// Initial delay in milliseconds
    pub initial_delay_ms: u64,
    /// Maximum delay in milliseconds
    pub max_delay_ms: u64,
    /// Multiplier for exponential backoff
    pub multiplier: f64,
}

impl Default for RetryBackoff {
    fn default() -> Self {
        Self {
            backoff_type: BackoffType::Exponential,
            initial_delay_ms: 1000,
            max_delay_ms: 30000,
            multiplier: 2.0,
        }
    }
}

impl RetryBackoff {
    /// Calculate delay for given retry attempt
    pub fn delay_for_attempt(&self, attempt: u32) -> Duration {
        let delay_ms = match self.backoff_type {
            BackoffType::Constant => self.initial_delay_ms,
            BackoffType::Linear => self.initial_delay_ms * (attempt as u64 + 1),
            BackoffType::Exponential => {
                (self.initial_delay_ms as f64 * self.multiplier.powi(attempt as i32)) as u64
            }
        };

        Duration::from_millis(delay_ms.min(self.max_delay_ms))
    }
}

// ============================================================================
// Rate Limiting
// ============================================================================

/// Rate limit configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateLimitConfig {
    /// Maximum requests per window
    pub max_requests: u32,
    /// Window size in seconds
    pub window_seconds: u32,
    /// Whether to apply per-client limiting
    pub per_client: bool,
}

impl Default for RateLimitConfig {
    fn default() -> Self {
        Self {
            max_requests: 1000,
            window_seconds: 60,
            per_client: true,
        }
    }
}

/// Rate limit status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateLimitStatus {
    /// Total limit
    pub limit: u32,
    /// Remaining requests
    pub remaining: u32,
    /// Reset timestamp (Unix epoch)
    pub reset_at: u64,
    /// Window size in seconds
    pub window_seconds: u32,
}

impl RateLimitStatus {
    /// Check if rate limit is exceeded
    pub fn is_exceeded(&self) -> bool {
        self.remaining == 0
    }

    /// Create from HTTP headers
    pub fn from_headers(
        limit: u32,
        remaining: u32,
        reset: u64,
        window: u32,
    ) -> Self {
        Self {
            limit,
            remaining,
            reset_at: reset,
            window_seconds: window,
        }
    }
}

// ============================================================================
// Connection Pool
// ============================================================================

/// Connection pool configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionPoolConfig {
    /// Maximum connections
    pub max_connections: u32,
    /// Minimum connections
    pub min_connections: u32,
    /// Connection idle timeout in seconds
    pub idle_timeout_seconds: u64,
    /// Maximum connection lifetime in seconds
    pub max_lifetime_seconds: u64,
}

impl Default for ConnectionPoolConfig {
    fn default() -> Self {
        Self {
            max_connections: 100,
            min_connections: 10,
            idle_timeout_seconds: 300,
            max_lifetime_seconds: 3600,
        }
    }
}

// ============================================================================
// HTTP Headers
// ============================================================================

/// Standard headers for WIA Security Protocol
pub mod headers {
    /// Protocol version header
    pub const X_WIA_PROTOCOL_VERSION: &str = "X-WIA-Protocol-Version";
    /// Client ID header
    pub const X_WIA_CLIENT_ID: &str = "X-WIA-Client-ID";
    /// Request ID header
    pub const X_WIA_REQUEST_ID: &str = "X-WIA-Request-ID";
    /// Rate limit headers
    pub const X_RATELIMIT_LIMIT: &str = "X-RateLimit-Limit";
    pub const X_RATELIMIT_REMAINING: &str = "X-RateLimit-Remaining";
    pub const X_RATELIMIT_RESET: &str = "X-RateLimit-Reset";
    pub const X_RATELIMIT_WINDOW: &str = "X-RateLimit-Window";
    /// Content type
    pub const CONTENT_TYPE_WIA: &str = "application/wia-security+json;version=1.0";
    pub const CONTENT_TYPE_STIX: &str = "application/stix+json;version=2.1";
    pub const CONTENT_TYPE_TAXII: &str = "application/taxii+json;version=2.1";
}

// ============================================================================
// Endpoints
// ============================================================================

/// Standard API endpoints
pub mod endpoints {
    /// Base API path
    pub const API_V1: &str = "/api/v1";
    /// Health check
    pub const HEALTH: &str = "/health";
    /// Events endpoint
    pub const EVENTS: &str = "/events";
    /// Alerts endpoint
    pub const ALERTS: &str = "/alerts";
    /// Threats endpoint
    pub const THREATS: &str = "/threats";
    /// Policies endpoint
    pub const POLICIES: &str = "/policies";
    /// Access decisions endpoint
    pub const ACCESS_DECISIONS: &str = "/access/decisions";
    /// TAXII discovery
    pub const TAXII_DISCOVERY: &str = "/taxii2/";
    /// TAXII API root
    pub const TAXII_API_ROOT: &str = "/taxii2/api/v1/";
    /// WebSocket stream
    pub const WS_STREAM: &str = "/stream";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_retry_backoff_exponential() {
        let backoff = RetryBackoff::default();

        assert_eq!(backoff.delay_for_attempt(0), Duration::from_millis(1000));
        assert_eq!(backoff.delay_for_attempt(1), Duration::from_millis(2000));
        assert_eq!(backoff.delay_for_attempt(2), Duration::from_millis(4000));
        assert_eq!(backoff.delay_for_attempt(3), Duration::from_millis(8000));
    }

    #[test]
    fn test_retry_backoff_max() {
        let backoff = RetryBackoff {
            max_delay_ms: 5000,
            ..Default::default()
        };

        assert_eq!(backoff.delay_for_attempt(10), Duration::from_millis(5000));
    }

    #[test]
    fn test_tls_config_default() {
        let config = TlsConfig::default();
        assert_eq!(config.min_version, TlsVersion::Tls13);
        assert!(config.verify_hostname);
    }
}
