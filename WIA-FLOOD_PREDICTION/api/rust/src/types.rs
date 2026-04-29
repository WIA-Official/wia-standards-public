use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StandardData {
    pub id: String,
    pub created_at: DateTime<Utc>,
    pub metadata: std::collections::HashMap<String, String>,
}
