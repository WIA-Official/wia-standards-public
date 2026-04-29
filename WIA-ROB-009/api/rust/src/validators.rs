//! Validation functions for robotics 009

use crate::error::{Result, RoboticsError};
use crate::types::*;

pub fn validate_item(item: &Robotics) -> Result<()> {
    if item.name.is_empty() {
        return Err(RoboticsError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
