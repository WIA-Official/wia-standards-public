//! Utility functions

use crate::types::*;

pub fn format_record(record: &Record) -> String {
    format!("Record {}: {}", record.id, record.data)
}
