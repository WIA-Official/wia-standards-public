//! Utility functions for plastic enzyme

use crate::types::*;
use uuid::Uuid;

pub fn generate_id() -> Uuid {
    Uuid::new_v4()
}

pub fn format_info(item: &PlasticEnzyme) -> String {
    format!("{} ({:?})", item.name, item.status)
}
