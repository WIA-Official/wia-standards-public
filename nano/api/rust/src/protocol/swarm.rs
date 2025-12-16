//! Swarm communication and coordination

use crate::error::{NanoError, NanoResult};
use crate::types::Position3D;
use super::{ProtocolMessage, MessageType, SwarmPayload, SwarmAction, ProtocolHeader, MessagePayload};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Swarm coordinator for managing group communication
pub struct SwarmCoordinator {
    swarm_id: String,
    node_id: String,
    role: SwarmRole,
    members: Arc<RwLock<HashMap<String, SwarmMemberState>>>,
    leader_id: Arc<RwLock<Option<String>>>,
    config: SwarmConfig,
    state: Arc<RwLock<SwarmState>>,
}

impl SwarmCoordinator {
    pub fn new(swarm_id: impl Into<String>, node_id: impl Into<String>, config: SwarmConfig) -> Self {
        Self {
            swarm_id: swarm_id.into(),
            node_id: node_id.into(),
            role: SwarmRole::Member,
            members: Arc::new(RwLock::new(HashMap::new())),
            leader_id: Arc::new(RwLock::new(None)),
            config,
            state: Arc::new(RwLock::new(SwarmState::Forming)),
        }
    }

    pub fn swarm_id(&self) -> &str {
        &self.swarm_id
    }

    pub fn node_id(&self) -> &str {
        &self.node_id
    }

    pub fn role(&self) -> SwarmRole {
        self.role
    }

    /// Join the swarm
    pub async fn join(&mut self) -> NanoResult<ProtocolMessage> {
        let mut members = self.members.write().await;
        members.insert(
            self.node_id.clone(),
            SwarmMemberState {
                id: self.node_id.clone(),
                role: SwarmRole::Member,
                position: None,
                status: MemberStatus::Active,
                last_heartbeat: chrono::Utc::now().timestamp() as u64,
                capabilities: Vec::new(),
            },
        );

        let msg = self.create_swarm_message(SwarmAction::Join, serde_json::json!({
            "node_id": self.node_id,
            "capabilities": []
        }));

        Ok(msg)
    }

    /// Leave the swarm
    pub async fn leave(&mut self) -> NanoResult<ProtocolMessage> {
        let mut members = self.members.write().await;
        members.remove(&self.node_id);

        let msg = self.create_swarm_message(SwarmAction::Leave, serde_json::json!({
            "node_id": self.node_id
        }));

        Ok(msg)
    }

    /// Initiate leader election
    pub async fn start_election(&self) -> NanoResult<ProtocolMessage> {
        let members = self.members.read().await;

        // Simple bully algorithm: highest ID wins
        let candidates: Vec<_> = members
            .values()
            .filter(|m| m.status == MemberStatus::Active)
            .map(|m| m.id.clone())
            .collect();

        let msg = self.create_swarm_message(SwarmAction::LeaderElection, serde_json::json!({
            "initiator": self.node_id,
            "candidates": candidates,
            "election_id": uuid::Uuid::new_v4().to_string()
        }));

        Ok(msg)
    }

    /// Handle election result
    pub async fn set_leader(&mut self, leader_id: String) {
        let mut leader = self.leader_id.write().await;
        *leader = Some(leader_id.clone());

        if leader_id == self.node_id {
            self.role = SwarmRole::Leader;
        } else {
            self.role = SwarmRole::Member;
        }
    }

    /// Get current leader
    pub async fn get_leader(&self) -> Option<String> {
        self.leader_id.read().await.clone()
    }

    /// Assign task to swarm members
    pub async fn assign_task(&self, task: SwarmTask) -> NanoResult<ProtocolMessage> {
        if self.role != SwarmRole::Leader {
            return Err(NanoError::InvalidParameter("Only leader can assign tasks".into()));
        }

        let msg = self.create_swarm_message(SwarmAction::TaskAssignment, serde_json::json!({
            "task_id": task.id,
            "task_type": format!("{:?}", task.task_type),
            "assignees": task.assignees,
            "parameters": task.parameters
        }));

        Ok(msg)
    }

    /// Synchronize swarm state
    pub async fn sync_state(&self) -> NanoResult<ProtocolMessage> {
        let members = self.members.read().await;
        let state = self.state.read().await;

        let msg = self.create_swarm_message(SwarmAction::StateSync, serde_json::json!({
            "state": format!("{:?}", *state),
            "member_count": members.len(),
            "timestamp": chrono::Utc::now().timestamp()
        }));

        Ok(msg)
    }

    /// Update formation
    pub async fn update_formation(&self, formation: Formation) -> NanoResult<ProtocolMessage> {
        let msg = self.create_swarm_message(SwarmAction::Formation, serde_json::json!({
            "formation_type": format!("{:?}", formation.formation_type),
            "center": formation.center.map(|p| serde_json::json!({"x": p.x, "y": p.y, "z": p.z})),
            "spacing_nm": formation.spacing_nm,
            "positions": formation.positions.iter().map(|(id, pos)| {
                serde_json::json!({"id": id, "position": {"x": pos.x, "y": pos.y, "z": pos.z}})
            }).collect::<Vec<_>>()
        }));

        Ok(msg)
    }

    /// Reach consensus on a value
    pub async fn propose_consensus(&self, key: String, value: serde_json::Value) -> NanoResult<ProtocolMessage> {
        let msg = self.create_swarm_message(SwarmAction::Consensus, serde_json::json!({
            "proposal_id": uuid::Uuid::new_v4().to_string(),
            "key": key,
            "value": value,
            "proposer": self.node_id
        }));

        Ok(msg)
    }

    /// Handle incoming swarm message
    pub async fn handle_message(&mut self, msg: &ProtocolMessage) -> NanoResult<Option<ProtocolMessage>> {
        if let MessagePayload::Swarm(payload) = &msg.payload {
            if payload.swarm_id != self.swarm_id {
                return Ok(None); // Not for our swarm
            }

            match payload.swarm_action {
                SwarmAction::Join => {
                    self.handle_join(&payload.data).await?;
                }
                SwarmAction::Leave => {
                    self.handle_leave(&payload.data).await?;
                }
                SwarmAction::LeaderElection => {
                    return self.handle_election(&payload.data).await;
                }
                SwarmAction::StateSync => {
                    self.handle_state_sync(&payload.data).await?;
                }
                _ => {}
            }
        }

        Ok(None)
    }

    async fn handle_join(&mut self, data: &serde_json::Value) -> NanoResult<()> {
        if let Some(node_id) = data.get("node_id").and_then(|v| v.as_str()) {
            let mut members = self.members.write().await;
            members.insert(
                node_id.to_string(),
                SwarmMemberState {
                    id: node_id.to_string(),
                    role: SwarmRole::Member,
                    position: None,
                    status: MemberStatus::Active,
                    last_heartbeat: chrono::Utc::now().timestamp() as u64,
                    capabilities: Vec::new(),
                },
            );
        }
        Ok(())
    }

    async fn handle_leave(&mut self, data: &serde_json::Value) -> NanoResult<()> {
        if let Some(node_id) = data.get("node_id").and_then(|v| v.as_str()) {
            let mut members = self.members.write().await;
            members.remove(node_id);
        }
        Ok(())
    }

    async fn handle_election(&mut self, data: &serde_json::Value) -> NanoResult<Option<ProtocolMessage>> {
        // Simple election: respond if we have higher ID
        if let Some(candidates) = data.get("candidates").and_then(|v| v.as_array()) {
            let my_id = &self.node_id;
            let higher_exists = candidates
                .iter()
                .filter_map(|v| v.as_str())
                .any(|id| id > my_id.as_str());

            if !higher_exists {
                // We should be leader
                self.set_leader(self.node_id.clone()).await;
                let msg = self.create_swarm_message(SwarmAction::LeaderElection, serde_json::json!({
                    "elected_leader": self.node_id,
                    "election_complete": true
                }));
                return Ok(Some(msg));
            }
        }
        Ok(None)
    }

    async fn handle_state_sync(&mut self, _data: &serde_json::Value) -> NanoResult<()> {
        // Update local state based on sync data
        Ok(())
    }

    fn create_swarm_message(&self, action: SwarmAction, data: serde_json::Value) -> ProtocolMessage {
        let mut header = ProtocolHeader::new(&self.node_id, MessageType::Swarm);
        header.flags.broadcast = true;

        ProtocolMessage::new(
            header,
            MessagePayload::Swarm(SwarmPayload {
                swarm_id: self.swarm_id.clone(),
                swarm_action: action,
                data,
            }),
        )
    }

    /// Get member count
    pub async fn member_count(&self) -> usize {
        self.members.read().await.len()
    }

    /// Get all members
    pub async fn get_members(&self) -> Vec<SwarmMemberState> {
        self.members.read().await.values().cloned().collect()
    }
}

/// Swarm role
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwarmRole {
    Leader,
    Member,
    Scout,
    Worker,
    Relay,
}

/// Swarm state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwarmState {
    Forming,
    Active,
    Migrating,
    Disbanding,
}

/// Member status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemberStatus {
    Active,
    Inactive,
    Busy,
    Failed,
}

/// Swarm member state
#[derive(Debug, Clone)]
pub struct SwarmMemberState {
    pub id: String,
    pub role: SwarmRole,
    pub position: Option<Position3D>,
    pub status: MemberStatus,
    pub last_heartbeat: u64,
    pub capabilities: Vec<String>,
}

/// Swarm configuration
#[derive(Debug, Clone)]
pub struct SwarmConfig {
    pub min_members: usize,
    pub max_members: usize,
    pub heartbeat_interval_ms: u64,
    pub election_timeout_ms: u64,
    pub consensus_threshold: f64,
}

impl Default for SwarmConfig {
    fn default() -> Self {
        Self {
            min_members: 2,
            max_members: 100,
            heartbeat_interval_ms: 1000,
            election_timeout_ms: 5000,
            consensus_threshold: 0.66,
        }
    }
}

/// Swarm task
#[derive(Debug, Clone)]
pub struct SwarmTask {
    pub id: String,
    pub task_type: SwarmTaskType,
    pub assignees: Vec<String>,
    pub parameters: serde_json::Value,
}

/// Task type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwarmTaskType {
    Navigate,
    Search,
    Assemble,
    Monitor,
    Deliver,
    Custom,
}

/// Formation configuration
#[derive(Debug, Clone)]
pub struct Formation {
    pub formation_type: FormationType,
    pub center: Option<Position3D>,
    pub spacing_nm: f64,
    pub positions: HashMap<String, Position3D>,
}

/// Formation type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormationType {
    Line,
    Circle,
    Grid,
    Sphere,
    Custom,
}

impl Formation {
    pub fn line(center: Position3D, spacing: f64, count: usize) -> Self {
        let mut positions = HashMap::new();
        for i in 0..count {
            let x_offset = (i as f64 - count as f64 / 2.0) * spacing;
            positions.insert(
                format!("pos_{}", i),
                Position3D::new(center.x + x_offset, center.y, center.z),
            );
        }

        Self {
            formation_type: FormationType::Line,
            center: Some(center),
            spacing_nm: spacing,
            positions,
        }
    }

    pub fn circle(center: Position3D, radius: f64, count: usize) -> Self {
        let mut positions = HashMap::new();
        for i in 0..count {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (count as f64);
            positions.insert(
                format!("pos_{}", i),
                Position3D::new(
                    center.x + radius * angle.cos(),
                    center.y + radius * angle.sin(),
                    center.z,
                ),
            );
        }

        Self {
            formation_type: FormationType::Circle,
            center: Some(center),
            spacing_nm: radius,
            positions,
        }
    }
}
