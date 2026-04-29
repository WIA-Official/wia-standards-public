//! Validation functions for autonomous ship

use crate::error::{Result, AutonomousShipError};
use crate::types::*;

pub fn validate_item(item: &AutonomousShip) -> Result<()> {
    if item.name.is_empty() {
        return Err(AutonomousShipError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
