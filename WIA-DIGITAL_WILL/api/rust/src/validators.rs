//! Data validation utilities for the WIA-DIGITAL_WILL Standard
//!
//! This module provides validation functions for WIA-DIGITAL_WILL data.

use crate::error::Result;
use crate::types::{ValidationError, ValidationResult};

/// Validate data against WIA-DIGITAL_WILL standards
///
/// # Example
///
/// ```rust
/// use wia_digital_will_sdk::validators::validate_data;
/// use serde_json::json;
///
/// let data = json!({"field": "value"});
/// let result = validate_data(&data)?;
/// assert!(result.valid);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn validate_data(data: &serde_json::Value) -> Result<ValidationResult> {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();

    // Basic validation: ensure it's an object
    if !data.is_object() {
        errors.push(ValidationError {
            field: "root".to_string(),
            message: "Data must be a JSON object".to_string(),
            code: "INVALID_TYPE".to_string(),
        });
    }

    // Add more validation rules here based on WIA-DIGITAL_WILL requirements

    Ok(ValidationResult {
        valid: errors.is_empty(),
        errors,
        warnings,
    })
}

/// Validate field presence
pub fn validate_required_field(
    data: &serde_json::Value,
    field: &str,
) -> Option<ValidationError> {
    if data.get(field).is_none() {
        Some(ValidationError {
            field: field.to_string(),
            message: format!("Required field '{}' is missing", field),
            code: "REQUIRED_FIELD".to_string(),
        })
    } else {
        None
    }
}

/// Validate string length
pub fn validate_string_length(
    value: &str,
    min: usize,
    max: usize,
    field: &str,
) -> Option<ValidationError> {
    let len = value.len();
    if len < min || len > max {
        Some(ValidationError {
            field: field.to_string(),
            message: format!(
                "Field '{}' must be between {} and {} characters (got {})",
                field, min, max, len
            ),
            code: "INVALID_LENGTH".to_string(),
        })
    } else {
        None
    }
}
