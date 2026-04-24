//! Validation functions for battery-passport

use crate::error::{Result, BatteryPassportError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(BatteryPassportError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
