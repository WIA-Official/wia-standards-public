//! Validation functions for ai-embodiment

use crate::error::{Result, AiEmbodimentError};
use crate::types::*;

pub fn validate_resource(resource: &Resource) -> Result<()> {
    if resource.name.is_empty() {
        return Err(AiEmbodimentError::ValidationError(
            "Name cannot be empty".to_string()
        ));
    }
    Ok(())
}
