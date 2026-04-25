//! Validation functions for autonomous-weapon-ethics

use crate::error::{Result, AutonomousWeaponEthicsError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AutonomousWeaponEthicsError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
