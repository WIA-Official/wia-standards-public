//! Core types for cosmic communication

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

/// Cosmic signal types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SignalType {
    /// Radio frequency signals
    Radio,
    /// Optical/laser signals
    Optical,
    /// Neutrino-based signals
    Neutrino,
    /// Gravitational wave patterns
    Gravitational,
    /// Unknown signal type
    Unknown,
}

/// Cosmic coordinate system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CosmicCoordinate {
    /// Right ascension (hours)
    pub right_ascension: f64,
    /// Declination (degrees)
    pub declination: f64,
    /// Distance (light years)
    pub distance: f64,
    /// Galactic longitude
    pub galactic_longitude: Option<f64>,
    /// Galactic latitude
    pub galactic_latitude: Option<f64>,
}

/// Cosmic signal structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CosmicSignal {
    /// Unique signal identifier
    pub signal_id: String,
    /// Signal type
    pub signal_type: SignalType,
    /// Source coordinates
    pub source: CosmicCoordinate,
    /// Frequency (Hz)
    pub frequency: f64,
    /// Signal strength (dB)
    pub strength: f64,
    /// Detection timestamp
    pub detected_at: DateTime<Utc>,
    /// Raw signal data (base64)
    pub raw_data: String,
    /// Decoded message (if available)
    pub decoded_message: Option<String>,
    /// Confidence score (0-1)
    pub confidence: f64,
    /// Metadata
    pub metadata: SignalMetadata,
}

/// Signal metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalMetadata {
    /// Observatory/detector name
    pub observatory: String,
    /// Detection equipment
    pub equipment: String,
    /// Analysis algorithm version
    pub algorithm_version: String,
    /// Peer review status
    pub peer_reviewed: bool,
    /// Tags
    pub tags: Vec<String>,
}

/// Universal message format
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UniversalMessage {
    /// Message ID
    pub message_id: String,
    /// Message type
    pub message_type: MessageType,
    /// Encoded content
    pub content: String,
    /// Encoding format
    pub encoding: EncodingFormat,
    /// Language/protocol version
    pub protocol_version: String,
    /// Creation timestamp
    pub created_at: DateTime<Utc>,
}

/// Message types
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum MessageType {
    /// Initial greeting
    Greeting,
    /// Mathematical concepts
    Mathematics,
    /// Scientific data
    Science,
    /// Cultural information
    Culture,
    /// Question
    Query,
    /// Response
    Response,
}

/// Encoding formats
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum EncodingFormat {
    /// Binary encoding
    Binary,
    /// Prime number sequences
    Prime,
    /// Arecibo-style pictogram
    Pictogram,
    /// Mathematical constants
    Mathematical,
    /// DNA-based encoding
    Biological,
}

/// Signal analysis result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisResult {
    /// Analysis ID
    pub analysis_id: String,
    /// Signal ID
    pub signal_id: String,
    /// Is artificial origin probable?
    pub artificial_probability: f64,
    /// Pattern detected
    pub patterns: Vec<String>,
    /// Recommendations
    pub recommendations: Vec<String>,
    /// Analysis timestamp
    pub analyzed_at: DateTime<Utc>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_signal_type_serialization() {
        let signal_type = SignalType::Radio;
        let json = serde_json::to_string(&signal_type).unwrap();
        assert_eq!(json, "\"RADIO\"");
    }

    #[test]
    fn test_coordinate_creation() {
        let coord = CosmicCoordinate {
            right_ascension: 12.5,
            declination: 45.0,
            distance: 100.0,
            galactic_longitude: Some(90.0),
            galactic_latitude: Some(-30.0),
        };
        assert_eq!(coord.right_ascension, 12.5);
    }
}
