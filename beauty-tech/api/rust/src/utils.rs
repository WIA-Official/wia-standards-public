//! Utility functions for beauty-tech

use crate::types::*;
use uuid::Uuid;

pub fn generate_id() -> Uuid {
    Uuid::new_v4()
}

pub fn format_info(resource: &Resource) -> String {
    format!("{} ({:?})", resource.name, resource.status)
}
