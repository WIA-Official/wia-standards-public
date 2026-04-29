use crate::types::*;

pub fn format_timestamp(ts: &chrono::DateTime<chrono::Utc>) -> String {
    ts.format("%Y-%m-%d %H:%M:%S UTC").to_string()
}
