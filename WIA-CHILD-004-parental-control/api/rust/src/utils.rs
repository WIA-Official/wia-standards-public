//! Utility functions

use crate::types::*;

pub fn calculate_screen_time_remaining(used_minutes: u32, limit_minutes: u32) -> u32 {
    limit_minutes.saturating_sub(used_minutes)
}

pub fn is_content_age_appropriate(filter: &ContentFilter, age: u8) -> bool {
    filter.age_appropriate && !filter.blocked
}
