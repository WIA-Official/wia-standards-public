use crate::{error::{Error, Result}, types::*};

pub fn validate_data(data: &StandardData) -> Result<()> {
    if data.id.is_empty() {
        return Err(Error::ValidationError("ID cannot be empty".to_string()));
    }
    Ok(())
}
