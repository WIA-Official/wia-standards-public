//! Utility functions for time management 024

use crate::types::*;
use uuid::Uuid;

pub fn generate_id() -> Uuid {
    Uuid::new_v4()
}

pub fn format_info(item: &TimeManagement) -> String {
    format!("{} ({:?})", item.name, item.status)
}
