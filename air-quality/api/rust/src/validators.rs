//! Validation functions for air-quality

use crate::error::{Result, AirQualityError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AirQualityError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
