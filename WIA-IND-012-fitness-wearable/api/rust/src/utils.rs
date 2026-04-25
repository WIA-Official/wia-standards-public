//! Utility functions
use crate::types::*;
use uuid::Uuid;
use chrono::Utc;

pub fn generate_id() -> Uuid {
    Uuid::new_v4()
}

pub fn current_timestamp() -> chrono::DateTime<Utc> {
    Utc::now()
}

pub fn filter_by_status(resources: &[Resource], status: &Status) -> Vec<Resource> {
    resources.iter().filter(|r| &r.status == status).cloned().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_generate_id() {
        let id1 = generate_id();
        let id2 = generate_id();
        assert_ne!(id1, id2);
    }
}
