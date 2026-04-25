//! Validation functions

use crate::error::{Error, Result};
use crate::types::*;

pub fn validate_record(record: &Record) -> Result<()> {
    if record.data.is_empty() {
        return Err(Error::InvalidInput("Data cannot be empty".into()));
    }
    Ok(())
}
