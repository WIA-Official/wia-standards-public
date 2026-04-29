//! Utility functions for the WIA-CHILD-006-child-data-privacy SDK

use std::time::{SystemTime, UNIX_EPOCH};

/// Get current timestamp in Unix milliseconds
pub fn current_timestamp() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards")
        .as_millis() as i64
}

/// Format timestamp to ISO 8601 string
pub fn format_timestamp(timestamp: i64) -> String {
    let secs = timestamp / 1000;
    let nsecs = ((timestamp % 1000) * 1_000_000) as u32;

    if let Some(datetime) = chrono::DateTime::from_timestamp(secs, nsecs) {
        datetime.format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string()
    } else {
        String::from("Invalid timestamp")
    }
}

/// Parse ISO 8601 string to timestamp
pub fn parse_timestamp(s: &str) -> Result<i64, chrono::ParseError> {
    use chrono::DateTime;
    let dt = DateTime::parse_from_rfc3339(s)?;
    Ok(dt.timestamp_millis())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_current_timestamp() {
        let ts = current_timestamp();
        assert!(ts > 0);
    }

    #[test]
    fn test_format_timestamp() {
        let ts = 1700000000000i64; // Nov 14, 2023
        let formatted = format_timestamp(ts);
        assert!(formatted.contains("2023"));
    }

    #[test]
    fn test_parse_timestamp() {
        let s = "2023-11-14T22:13:20.000Z";
        let ts = parse_timestamp(s).unwrap();
        assert_eq!(ts, 1700000000000);
    }
}
