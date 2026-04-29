//! Type definitions for Art Authentication Standard

use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Artwork {
    pub id: Uuid,
    pub title: String,
    pub artist: String,
    pub year_created: i32,
    pub medium: String,
    pub dimensions: String,
    pub certificate_hash: String,
    pub provenance: Vec<ProvenanceRecord>,
    pub verified: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProvenanceRecord {
    pub id: Uuid,
    pub previous_owner: String,
    pub current_owner: String,
    pub transfer_date: DateTime<Utc>,
    pub verification_method: String,
    pub certificate_url: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Certificate {
    pub id: Uuid,
    pub artwork_id: Uuid,
    pub issued_date: DateTime<Utc>,
    pub issuer: String,
    pub certificate_type: CertificateType,
    pub blockchain_hash: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum CertificateType {
    #[serde(rename = "authenticity")]
    Authenticity,
    #[serde(rename = "provenance")]
    Provenance,
    #[serde(rename = "ownership")]
    Ownership,
}
