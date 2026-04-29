//! Validation functions for cosmic communication

use crate::{
    error::{Error, Result},
    types::*,
};

/// Validate cosmic signal
pub fn validate_signal(signal: &CosmicSignal) -> Result<()> {
    if signal.signal_id.is_empty() {
        return Err(Error::ValidationError("Signal ID cannot be empty".to_string()));
    }

    if signal.frequency <= 0.0 {
        return Err(Error::ValidationError("Frequency must be positive".to_string()));
    }

    if signal.strength < -200.0 || signal.strength > 200.0 {
        return Err(Error::ValidationError("Signal strength out of valid range".to_string()));
    }

    if signal.confidence < 0.0 || signal.confidence > 1.0 {
        return Err(Error::ValidationError("Confidence must be between 0 and 1".to_string()));
    }

    validate_coordinate(&signal.source)?;

    Ok(())
}

/// Validate cosmic coordinate
pub fn validate_coordinate(coord: &CosmicCoordinate) -> Result<()> {
    if coord.right_ascension < 0.0 || coord.right_ascension > 24.0 {
        return Err(Error::ValidationError("Right ascension must be 0-24 hours".to_string()));
    }

    if coord.declination < -90.0 || coord.declination > 90.0 {
        return Err(Error::ValidationError("Declination must be -90 to +90 degrees".to_string()));
    }

    if coord.distance <= 0.0 {
        return Err(Error::ValidationError("Distance must be positive".to_string()));
    }

    if let Some(lon) = coord.galactic_longitude {
        if lon < 0.0 || lon > 360.0 {
            return Err(Error::ValidationError("Galactic longitude must be 0-360 degrees".to_string()));
        }
    }

    if let Some(lat) = coord.galactic_latitude {
        if lat < -90.0 || lat > 90.0 {
            return Err(Error::ValidationError("Galactic latitude must be -90 to +90 degrees".to_string()));
        }
    }

    Ok(())
}

/// Validate universal message
pub fn validate_message(message: &UniversalMessage) -> Result<()> {
    if message.message_id.is_empty() {
        return Err(Error::ValidationError("Message ID cannot be empty".to_string()));
    }

    if message.content.is_empty() {
        return Err(Error::ValidationError("Message content cannot be empty".to_string()));
    }

    if message.protocol_version.is_empty() {
        return Err(Error::ValidationError("Protocol version cannot be empty".to_string()));
    }

    Ok(())
}

/// Validate analysis result
pub fn validate_analysis(analysis: &AnalysisResult) -> Result<()> {
    if analysis.analysis_id.is_empty() {
        return Err(Error::ValidationError("Analysis ID cannot be empty".to_string()));
    }

    if analysis.signal_id.is_empty() {
        return Err(Error::ValidationError("Signal ID cannot be empty".to_string()));
    }

    if analysis.artificial_probability < 0.0 || analysis.artificial_probability > 1.0 {
        return Err(Error::ValidationError("Probability must be between 0 and 1".to_string()));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;

    #[test]
    fn test_valid_coordinate() {
        let coord = CosmicCoordinate {
            right_ascension: 12.5,
            declination: 45.0,
            distance: 100.0,
            galactic_longitude: Some(180.0),
            galactic_latitude: Some(-30.0),
        };
        assert!(validate_coordinate(&coord).is_ok());
    }

    #[test]
    fn test_invalid_right_ascension() {
        let coord = CosmicCoordinate {
            right_ascension: 25.0, // Invalid
            declination: 45.0,
            distance: 100.0,
            galactic_longitude: None,
            galactic_latitude: None,
        };
        assert!(validate_coordinate(&coord).is_err());
    }

    #[test]
    fn test_valid_message() {
        let message = UniversalMessage {
            message_id: "msg-001".to_string(),
            message_type: MessageType::Greeting,
            content: "Hello Universe".to_string(),
            encoding: EncodingFormat::Binary,
            protocol_version: "1.0".to_string(),
            created_at: Utc::now(),
        };
        assert!(validate_message(&message).is_ok());
    }
}
