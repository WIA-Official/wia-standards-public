//! Type definitions for time management 012

use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeManagement {
    pub id: Uuid,
    pub name: String,
    pub created_at: DateTime<Utc>,
    pub status: Status,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Status {
    Active,
    Inactive,
    Pending,
}
