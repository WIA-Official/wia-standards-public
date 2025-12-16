//! WIA Voice-Sign API
//!
//! A comprehensive Rust API for voice-to-sign language translation.
//!
//! # Overview
//!
//! This library provides a complete pipeline for converting spoken language
//! to sign language representation, including:
//!
//! - Speech recognition (ASR)
//! - Text-to-sign translation
//! - Sign notation generation (HamNoSys, SiGML)
//! - 3D pose animation generation
//! - Avatar rendering
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use wia_voice_sign::*;
//! use wia_voice_sign::adapters::create_simulator_pipeline;
//!
//! #[tokio::main]
//! async fn main() {
//!     // Create a pipeline with simulators
//!     let pipeline = create_simulator_pipeline(SignLanguageCode::Asl);
//!
//!     // Create a translation request
//!     let request = TranslationRequest {
//!         request_id: "test-001".to_string(),
//!         audio: None,
//!         text: Some(TextInput {
//!             text: "Hello, how are you?".to_string(),
//!             language: "en".to_string(),
//!         }),
//!         target_language: SignLanguageCode::Asl,
//!         output: OutputPreferences {
//!             include_transcript: true,
//!             include_gloss: true,
//!             include_notation: true,
//!             include_pose: true,
//!             include_render: false,
//!             render_settings: None,
//!         },
//!         options: None,
//!     };
//!
//!     // Process the request
//!     let response = pipeline.process(request).await;
//!     println!("Status: {:?}", response.status);
//! }
//! ```
//!
//! # Modules
//!
//! - [`types`] - Core type definitions (AudioInput, SignGloss, etc.)
//! - [`error`] - Error types and handling
//! - [`core`] - Main processing pipeline and traits
//! - [`adapters`] - Adapter implementations (simulators, etc.)
//! - [`safety`] - Safety, quality, and privacy modules
//! - [`deployment`] - Deployment configuration and health checks
//! - [`monitoring`] - Metrics, tracing, and alerting
//! - [`analytics`] - Analytics collection and export
//! - [`integration`] - WIA ecosystem integration (Exoskeleton, Bionic Eye)
//!
//! # Supported Sign Languages
//!
//! - ASL (American Sign Language)
//! - BSL (British Sign Language)
//! - KSL (Korean Sign Language / 한국수화)
//! - And more...
//!
//! # Features
//!
//! - `websocket` - WebSocket streaming support
//! - `wasm` - WebAssembly bindings
//! - `python` - Python bindings via PyO3
//! - `full` - All features enabled

pub mod types;
pub mod error;
pub mod core;
pub mod adapters;
pub mod safety;
pub mod deployment;
pub mod monitoring;
pub mod analytics;
pub mod integration;

// Re-export commonly used types
pub use types::*;
pub use error::{Result, VoiceSignError};
pub use core::{
    AsrEngine,
    TranslationEngine,
    PoseGenerator,
    Renderer,
    TranslationPipeline,
    RequestValidator,
    AudioProcessor,
    VoiceActivityDetector,
    GlossDatabase,
    GlossDatabaseManager,
};
pub use safety::{
    SafetyChecker,
    QualityChecker,
    AccessibilityChecker,
    EmergencyDetector,
    ContentFilter,
    PrivacyGuard,
};
pub use deployment::{
    DeploymentConfig,
    HealthCheckManager,
    HealthStatus,
    ScalingConfig,
    ScalingCalculator,
};
pub use monitoring::{
    MetricsRegistry,
    VoiceSignMetrics,
    TracingConfig,
    AlertManager,
    AlertSeverity,
};
pub use analytics::{
    AnalyticsCollector,
    MetricsAggregator,
    AnalyticsExporter,
    AggregationWindow,
};
pub use integration::{
    WiaBridge,
    WiaSystem,
    SignGestureCommand,
    BionicEyeDisplayRequest,
    WiaCloudEvent,
    AuthorizationEngine,
};

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Library name
pub const NAME: &str = env!("CARGO_PKG_NAME");

/// Create a new translation pipeline with default simulators
///
/// This is a convenience function for quickly getting started.
///
/// # Arguments
///
/// * `target_language` - The target sign language
///
/// # Example
///
/// ```rust,no_run
/// use wia_voice_sign::{create_pipeline, SignLanguageCode};
///
/// let pipeline = create_pipeline(SignLanguageCode::Asl);
/// ```
pub fn create_pipeline(target_language: SignLanguageCode) -> TranslationPipeline {
    adapters::create_simulator_pipeline(target_language)
}

/// Validate a translation request
///
/// # Arguments
///
/// * `request` - The translation request to validate
///
/// # Returns
///
/// `Ok(())` if the request is valid, or an error describing the validation failure.
///
/// # Example
///
/// ```rust,no_run
/// use wia_voice_sign::{validate_request, TranslationRequest};
///
/// let request: TranslationRequest = // ...
/// # todo!();
/// if let Err(e) = validate_request(&request) {
///     eprintln!("Invalid request: {}", e);
/// }
/// ```
pub fn validate_request(request: &TranslationRequest) -> Result<()> {
    RequestValidator::validate(request)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn test_create_pipeline() {
        let _pipeline = create_pipeline(SignLanguageCode::Asl);
    }

    #[tokio::test]
    async fn test_basic_translation() {
        let pipeline = create_pipeline(SignLanguageCode::Asl);

        let request = TranslationRequest {
            request_id: "test".to_string(),
            audio: None,
            text: Some(TextInput {
                text: "Hello".to_string(),
                language: "en".to_string(),
            }),
            target_language: SignLanguageCode::Asl,
            output: OutputPreferences {
                include_transcript: false,
                include_gloss: true,
                include_notation: false,
                include_pose: false,
                include_render: false,
                render_settings: None,
            },
            options: None,
        };

        let response = pipeline.process(request).await;
        assert_eq!(response.status, ResponseStatus::Success);
        assert!(response.gloss.is_some());
    }
}
