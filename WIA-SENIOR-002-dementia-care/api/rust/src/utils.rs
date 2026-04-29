//! Utility functions for dementia care

use crate::types::*;
use uuid::Uuid;

pub fn generate_id() -> Uuid {
    Uuid::new_v4()
}

pub fn format_info(item: &DementiaCare) -> String {
    format!("{} ({:?})", item.name, item.status)
}
