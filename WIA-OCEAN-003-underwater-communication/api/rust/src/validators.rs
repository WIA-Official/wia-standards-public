//! Validation functions for underwater communication

use crate::error::{Result, UnderwaterCommunicationError};
use crate::types::*;

pub fn validate_message(message: &AcousticMessage) -> Result<()> {
    if message.frequency_khz <= 0.0 {
        return Err(UnderwaterCommunicationError::ValidationError(
            "Frequency must be positive".to_string()
        ));
    }
    Ok(())
}

pub fn validate_config(config: &TransmissionConfig) -> Result<()> {
    if config.max_range_meters <= 0.0 {
        return Err(UnderwaterCommunicationError::ValidationError(
            "Max range must be positive".to_string()
        ));
    }
    Ok(())
}
