//! Validation functions for port automation

use crate::error::{Result, PortAutomationError};
use crate::types::*;

pub fn validate_item(item: &PortAutomation) -> Result<()> {
    if item.name.is_empty() {
        return Err(PortAutomationError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
