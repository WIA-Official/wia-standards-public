//! Validation functions for battery-management-system

use crate::error::{Result, BatteryManagementSystemError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(BatteryManagementSystemError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
