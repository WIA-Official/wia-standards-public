//! Quantum Key Distribution types

use crate::types::WIA_QUANTUM_VERSION;
use crate::error::Result;
use serde::{Deserialize, Serialize};
use uuid::Uuid;
use chrono::{DateTime, Utc};

/// QKD Protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum QkdProtocol {
    /// Original BB84 protocol
    BB84,
    /// E91 (Ekert) protocol using entanglement
    E91,
    /// BBM92 protocol
    BBM92,
    /// SARG04 protocol
    SARG04,
    /// Coherent One-Way protocol
    COW,
    /// Differential Phase Shift protocol
    DPS,
}

impl QkdProtocol {
    /// Get protocol description
    pub fn description(&self) -> &'static str {
        match self {
            QkdProtocol::BB84 => "Bennett-Brassard 1984 - Prepare and measure",
            QkdProtocol::E91 => "Ekert 1991 - Entanglement-based",
            QkdProtocol::BBM92 => "Bennett-Brassard-Mermin 1992 - Entanglement-based",
            QkdProtocol::SARG04 => "Scarani et al. 2004 - PNS attack resistant",
            QkdProtocol::COW => "Coherent One-Way - Industrial deployment",
            QkdProtocol::DPS => "Differential Phase Shift - High rate",
        }
    }
}

/// QKD Session status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QkdSessionStatus {
    Initializing,
    Active,
    Paused,
    Completed,
    Failed,
}

/// Channel type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChannelType {
    Fiber,
    FreeSpace,
    Satellite,
}

/// QKD Participant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QkdParticipant {
    pub node_id: String,
    pub address: String,
}

/// QKD Channel info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QkdChannel {
    #[serde(rename = "type")]
    pub channel_type: ChannelType,
    pub length_km: f64,
    pub loss_db: f64,
}

/// QKD Statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QkdStatistics {
    /// Raw key generation rate (bits per second)
    pub raw_key_rate_bps: f64,
    /// Quantum Bit Error Rate
    pub qber: f64,
    /// Secure key rate after error correction
    pub secure_key_rate_bps: f64,
    /// Total secure key bits generated
    pub total_key_bits: u64,
}

impl Default for QkdStatistics {
    fn default() -> Self {
        Self {
            raw_key_rate_bps: 0.0,
            qber: 0.0,
            secure_key_rate_bps: 0.0,
            total_key_bits: 0,
        }
    }
}

/// Participants structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Participants {
    pub alice: QkdParticipant,
    pub bob: QkdParticipant,
}

/// QKD Session
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QkdSession {
    pub wia_quantum_version: String,
    #[serde(rename = "type")]
    pub data_type: String,
    pub crypto_type: String,
    pub session_id: Uuid,
    pub protocol: QkdProtocol,
    pub participants: Participants,
    pub channel: QkdChannel,
    pub statistics: QkdStatistics,
    pub status: QkdSessionStatus,
    pub started_at: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ended_at: Option<DateTime<Utc>>,
}

impl QkdSession {
    /// Create a new QKD session
    pub fn new(
        protocol: QkdProtocol,
        alice: QkdParticipant,
        bob: QkdParticipant,
        channel: QkdChannel,
    ) -> Self {
        Self {
            wia_quantum_version: WIA_QUANTUM_VERSION.to_string(),
            data_type: "crypto".to_string(),
            crypto_type: "qkd_session".to_string(),
            session_id: Uuid::new_v4(),
            protocol,
            participants: Participants { alice, bob },
            channel,
            statistics: QkdStatistics::default(),
            status: QkdSessionStatus::Initializing,
            started_at: Utc::now(),
            ended_at: None,
        }
    }

    /// Update session status
    pub fn set_status(&mut self, status: QkdSessionStatus) {
        self.status = status;
        if matches!(status, QkdSessionStatus::Completed | QkdSessionStatus::Failed) {
            self.ended_at = Some(Utc::now());
        }
    }

    /// Update statistics
    pub fn update_statistics(&mut self, stats: QkdStatistics) {
        self.statistics = stats;
    }

    /// Calculate estimated secure key rate based on channel loss
    pub fn estimate_secure_key_rate(&self) -> f64 {
        // Simplified model: rate decreases with channel loss
        let base_rate = 10000.0; // 10 kbps base
        let loss_factor = 10.0_f64.powf(-self.channel.loss_db / 10.0);
        let qber_factor = (1.0 - 4.0 * self.statistics.qber).max(0.0);
        base_rate * loss_factor * qber_factor
    }

    /// Serialize to JSON
    pub fn to_json(&self) -> Result<String> {
        serde_json::to_string_pretty(self).map_err(Into::into)
    }

    /// Deserialize from JSON
    pub fn from_json(json: &str) -> Result<Self> {
        serde_json::from_str(json).map_err(Into::into)
    }
}
