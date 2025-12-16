//! Brain-Computer Interface (BCI) Integration
//!
//! Provides integration with BCI systems for intent detection
//! and safety-critical commands.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::error::Result;

/// BCI signal pattern types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BciPattern {
    /// Motor imagery - both hands
    MotorImageryBothHands,
    /// Motor imagery - right hand
    MotorImageryRight,
    /// Motor imagery - left hand
    MotorImageryLeft,
    /// P300 emergency symbol
    P300Emergency,
    /// SSVEP 8Hz
    Ssvep8Hz,
    /// SSVEP 10Hz
    Ssvep10Hz,
    /// SSVEP 12Hz
    Ssvep12Hz,
}

/// BCI intent types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BciIntent {
    /// Stop vehicle immediately
    StopVehicle,
    /// Call for help
    CallHelp,
    /// Open door
    OpenDoor,
    /// Confirm destination
    ConfirmDestination,
    /// Cancel trip
    CancelTrip,
    /// Temperature up
    TemperatureUp,
    /// Temperature down
    TemperatureDown,
    /// No intent detected
    None,
}

/// BCI command configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BciCommandConfig {
    /// Intent to trigger
    pub intent: BciIntent,
    /// Pattern to detect
    pub pattern: BciPattern,
    /// Confidence threshold (0-1)
    pub confidence_threshold: f32,
    /// Whether confirmation is required
    pub confirmation_required: bool,
    /// Confirmation duration in seconds
    pub confirmation_duration: f32,
    /// Cooldown in seconds
    pub cooldown_seconds: f32,
}

/// Default BCI command configurations
pub fn default_commands() -> Vec<BciCommandConfig> {
    vec![
        BciCommandConfig {
            intent: BciIntent::StopVehicle,
            pattern: BciPattern::MotorImageryBothHands,
            confidence_threshold: 0.85,
            confirmation_required: false,
            confirmation_duration: 0.0,
            cooldown_seconds: 0.0,
        },
        BciCommandConfig {
            intent: BciIntent::CallHelp,
            pattern: BciPattern::P300Emergency,
            confidence_threshold: 0.80,
            confirmation_required: true,
            confirmation_duration: 1.0,
            cooldown_seconds: 5.0,
        },
        BciCommandConfig {
            intent: BciIntent::ConfirmDestination,
            pattern: BciPattern::MotorImageryRight,
            confidence_threshold: 0.75,
            confirmation_required: true,
            confirmation_duration: 1.0,
            cooldown_seconds: 2.0,
        },
        BciCommandConfig {
            intent: BciIntent::CancelTrip,
            pattern: BciPattern::MotorImageryLeft,
            confidence_threshold: 0.80,
            confirmation_required: true,
            confirmation_duration: 1.0,
            cooldown_seconds: 3.0,
        },
        BciCommandConfig {
            intent: BciIntent::TemperatureUp,
            pattern: BciPattern::Ssvep8Hz,
            confidence_threshold: 0.70,
            confirmation_required: false,
            confirmation_duration: 0.0,
            cooldown_seconds: 10.0,
        },
        BciCommandConfig {
            intent: BciIntent::TemperatureDown,
            pattern: BciPattern::Ssvep10Hz,
            confidence_threshold: 0.70,
            confirmation_required: false,
            confirmation_duration: 0.0,
            cooldown_seconds: 10.0,
        },
    ]
}

/// BCI signal quality
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalQuality {
    /// Excellent signal
    Excellent,
    /// Good signal
    Good,
    /// Fair signal
    Fair,
    /// Poor signal
    Poor,
    /// No signal
    NoSignal,
}

impl SignalQuality {
    /// Get numeric quality value (0-100)
    pub fn value(&self) -> u8 {
        match self {
            SignalQuality::Excellent => 100,
            SignalQuality::Good => 75,
            SignalQuality::Fair => 50,
            SignalQuality::Poor => 25,
            SignalQuality::NoSignal => 0,
        }
    }

    /// From numeric value
    pub fn from_value(value: u8) -> Self {
        match value {
            0..=20 => SignalQuality::NoSignal,
            21..=40 => SignalQuality::Poor,
            41..=60 => SignalQuality::Fair,
            61..=80 => SignalQuality::Good,
            _ => SignalQuality::Excellent,
        }
    }
}

/// BCI detection result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BciDetection {
    /// Detected pattern
    pub pattern: BciPattern,
    /// Confidence score (0-1)
    pub confidence: f32,
    /// Signal quality
    pub signal_quality: SignalQuality,
    /// Timestamp
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

/// Intent confirmation state
#[derive(Debug, Clone)]
pub struct IntentConfirmation {
    /// Intent being confirmed
    pub intent: BciIntent,
    /// Start time
    pub start_time: chrono::DateTime<chrono::Utc>,
    /// Required duration
    pub required_duration: f32,
    /// Accumulated confidence samples
    pub confidence_samples: Vec<f32>,
}

impl IntentConfirmation {
    /// Create new confirmation
    pub fn new(intent: BciIntent, required_duration: f32) -> Self {
        Self {
            intent,
            start_time: chrono::Utc::now(),
            required_duration,
            confidence_samples: Vec::new(),
        }
    }

    /// Add confidence sample
    pub fn add_sample(&mut self, confidence: f32) {
        self.confidence_samples.push(confidence);
    }

    /// Get elapsed time in seconds
    pub fn elapsed_seconds(&self) -> f32 {
        (chrono::Utc::now() - self.start_time).num_milliseconds() as f32 / 1000.0
    }

    /// Check if confirmation is complete
    pub fn is_complete(&self) -> bool {
        self.elapsed_seconds() >= self.required_duration
    }

    /// Get average confidence
    pub fn average_confidence(&self) -> f32 {
        if self.confidence_samples.is_empty() {
            0.0
        } else {
            self.confidence_samples.iter().sum::<f32>() / self.confidence_samples.len() as f32
        }
    }
}

/// BCI safety configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BciSafetyConfig {
    /// Maximum speed when BCI controlled (m/s)
    pub max_speed: f32,
    /// Auto-stop timeout when no signal (seconds)
    pub auto_stop_timeout: f32,
    /// Enable obstacle override
    pub obstacle_override: bool,
    /// Minimum signal quality required
    pub min_signal_quality: SignalQuality,
}

impl Default for BciSafetyConfig {
    fn default() -> Self {
        Self {
            max_speed: 0.3,
            auto_stop_timeout: 2.0,
            obstacle_override: true,
            min_signal_quality: SignalQuality::Fair,
        }
    }
}

/// BCI status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BciStatus {
    /// Connection status
    pub connected: bool,
    /// Signal quality
    pub signal_quality: SignalQuality,
    /// Battery level (if applicable)
    pub battery_level: Option<u8>,
    /// Last detection
    pub last_detection: Option<BciDetection>,
    /// Currently confirming intent
    pub confirming_intent: Option<BciIntent>,
}

impl Default for BciStatus {
    fn default() -> Self {
        Self {
            connected: false,
            signal_quality: SignalQuality::NoSignal,
            battery_level: None,
            last_detection: None,
            confirming_intent: None,
        }
    }
}

/// BCI fallback action
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FallbackAction {
    /// Switch to eye gaze
    SwitchToEyeGaze,
    /// Switch to voice
    SwitchToVoice,
    /// Alert passenger
    AlertPassenger,
    /// Auto pullover
    AutoPullover,
}

/// BCI integration handler
#[async_trait]
pub trait BciIntegration: Send + Sync {
    /// Connect to BCI device
    async fn connect(&mut self) -> Result<()>;

    /// Disconnect from BCI device
    async fn disconnect(&mut self) -> Result<()>;

    /// Get current status
    async fn status(&self) -> Result<BciStatus>;

    /// Get latest detection
    async fn latest_detection(&self) -> Result<Option<BciDetection>>;

    /// Get confirmed intent (if any)
    async fn confirmed_intent(&self) -> Result<Option<BciIntent>>;

    /// Check if connected
    fn is_connected(&self) -> bool;
}

/// Mock BCI integration for testing
pub struct MockBciIntegration {
    connected: bool,
    status: std::sync::Arc<tokio::sync::RwLock<BciStatus>>,
    next_intent: std::sync::Arc<tokio::sync::RwLock<Option<BciIntent>>>,
}

impl MockBciIntegration {
    /// Create new mock integration
    pub fn new() -> Self {
        Self {
            connected: false,
            status: std::sync::Arc::new(tokio::sync::RwLock::new(BciStatus::default())),
            next_intent: std::sync::Arc::new(tokio::sync::RwLock::new(None)),
        }
    }

    /// Simulate intent detection
    pub async fn simulate_intent(&self, intent: BciIntent) {
        *self.next_intent.write().await = Some(intent);
    }

    /// Simulate signal quality
    pub async fn simulate_signal_quality(&self, quality: SignalQuality) {
        self.status.write().await.signal_quality = quality;
    }
}

impl Default for MockBciIntegration {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl BciIntegration for MockBciIntegration {
    async fn connect(&mut self) -> Result<()> {
        self.connected = true;
        let mut status = self.status.write().await;
        status.connected = true;
        status.signal_quality = SignalQuality::Good;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<()> {
        self.connected = false;
        let mut status = self.status.write().await;
        status.connected = false;
        status.signal_quality = SignalQuality::NoSignal;
        Ok(())
    }

    async fn status(&self) -> Result<BciStatus> {
        Ok(self.status.read().await.clone())
    }

    async fn latest_detection(&self) -> Result<Option<BciDetection>> {
        Ok(self.status.read().await.last_detection.clone())
    }

    async fn confirmed_intent(&self) -> Result<Option<BciIntent>> {
        Ok(self.next_intent.write().await.take())
    }

    fn is_connected(&self) -> bool {
        self.connected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signal_quality() {
        assert_eq!(SignalQuality::Excellent.value(), 100);
        assert_eq!(SignalQuality::from_value(85), SignalQuality::Excellent);
        assert_eq!(SignalQuality::from_value(30), SignalQuality::Poor);
    }

    #[test]
    fn test_intent_confirmation() {
        let mut confirmation = IntentConfirmation::new(BciIntent::ConfirmDestination, 1.0);
        confirmation.add_sample(0.8);
        confirmation.add_sample(0.85);
        confirmation.add_sample(0.75);

        assert!((confirmation.average_confidence() - 0.8).abs() < 0.01);
    }

    #[tokio::test]
    async fn test_mock_integration() {
        let mut integration = MockBciIntegration::new();
        integration.connect().await.unwrap();

        assert!(integration.is_connected());

        let status = integration.status().await.unwrap();
        assert!(status.connected);
        assert_eq!(status.signal_quality, SignalQuality::Good);

        integration.simulate_intent(BciIntent::StopVehicle).await;
        let intent = integration.confirmed_intent().await.unwrap();
        assert_eq!(intent, Some(BciIntent::StopVehicle));
    }
}
