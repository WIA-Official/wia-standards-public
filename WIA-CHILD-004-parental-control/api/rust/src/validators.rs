//! Validation functions

use crate::error::{Error, Result};
use crate::types::*;

pub fn validate_parental_control(control: &ParentalControl) -> Result<()> {
    if control.screen_time_limit_minutes == 0 {
        return Err(Error::InvalidInput("Screen time limit must be positive".into()));
    }
    Ok(())
}
