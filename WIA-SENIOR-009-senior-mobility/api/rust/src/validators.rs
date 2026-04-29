//! Validation functions for senior mobility

use crate::error::{Result, SeniorMobilityError};
use crate::types::*;

pub fn validate_item(item: &SeniorMobility) -> Result<()> {
    if item.name.is_empty() {
        return Err(SeniorMobilityError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
