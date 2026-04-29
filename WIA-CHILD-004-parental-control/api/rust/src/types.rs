//! Types for Parental Control

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParentalControl {
    pub id: Uuid,
    pub child_id: Uuid,
    pub restrictions: Vec<Restriction>,
    pub screen_time_limit_minutes: u32,
    pub content_filters: Vec<ContentFilter>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Restriction {
    pub restriction_type: RestrictionType,
    pub enabled: bool,
    pub schedule: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum RestrictionType {
    #[serde(rename = "app_usage")]
    AppUsage,
    #[serde(rename = "web_browsing")]
    WebBrowsing,
    #[serde(rename = "social_media")]
    SocialMedia,
    #[serde(rename = "purchases")]
    Purchases,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContentFilter {
    pub category: String,
    pub blocked: bool,
    pub age_appropriate: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActivityLog {
    pub id: Uuid,
    pub child_id: Uuid,
    pub activity_type: String,
    pub timestamp: DateTime<Utc>,
    pub duration_minutes: u32,
    pub blocked: bool,
}
