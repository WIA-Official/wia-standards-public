//! Validation functions for plastic enzyme

use crate::error::{Result, PlasticEnzymeError};
use crate::types::*;

pub fn validate_item(item: &PlasticEnzyme) -> Result<()> {
    if item.name.is_empty() {
        return Err(PlasticEnzymeError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
