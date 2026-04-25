//! WIA-LLM-INTEROP Core Business Logic
//!
//! Capability matching, message routing, federation management, consensus.

use crate::types::*;
use chrono::Utc;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

// ============================================================================
// CAPABILITY REGISTRY
// ============================================================================

/// Registry for AI capabilities
pub struct CapabilityRegistry {
    capabilities: HashMap<String, CapabilityDocument>,
}

impl CapabilityRegistry {
    pub fn new() -> Self {
        Self {
            capabilities: HashMap::new(),
        }
    }

    /// Register a capability document
    pub fn register(&mut self, capability: CapabilityDocument) {
        let model_id = capability.model.model_id.clone();
        self.capabilities.insert(model_id, capability);
    }

    /// Get capability by model ID
    pub fn get(&self, model_id: &str) -> Option<&CapabilityDocument> {
        self.capabilities.get(model_id)
    }

    /// List all capabilities
    pub fn list(&self) -> Vec<&CapabilityDocument> {
        self.capabilities.values().collect()
    }

    /// Query capabilities matching requirements
    pub fn query(&self, query: &CapabilityQuery) -> Vec<CapabilityMatch> {
        let mut matches = Vec::new();

        for cap in self.capabilities.values() {
            let score = self.calculate_match_score(query, cap);
            if score > 0.0 {
                matches.push(CapabilityMatch {
                    model_id: cap.model.model_id.clone(),
                    score,
                    capability_uri: cap.endpoints.capability.clone(),
                });
            }
        }

        // Sort by score descending
        matches.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));

        matches
    }

    fn calculate_match_score(&self, query: &CapabilityQuery, cap: &CapabilityDocument) -> f64 {
        // Check minimum level
        if let Some(min_level) = query.min_level {
            if cap.level.0 < min_level {
                return 0.0;
            }
        }

        let mut score = 0.0;
        let mut factors = 0;

        // Domain matching
        if !query.required_domains.is_empty() {
            let mut domain_score = 0.0;
            let mut matched = 0;

            for required in &query.required_domains {
                for domain_exp in &cap.domains {
                    if &domain_exp.domain == required {
                        domain_score += domain_exp.proficiency;
                        matched += 1;
                        break;
                    }
                }
            }

            // All required domains must match
            if matched < query.required_domains.len() {
                return 0.0;
            }

            score += domain_score / query.required_domains.len() as f64;
            factors += 1;
        }

        // Language matching
        if !query.required_languages.is_empty() {
            let mut lang_score = 0.0;
            let mut matched = 0;

            for required in &query.required_languages {
                for lang_sup in &cap.languages {
                    if &lang_sup.code == required {
                        lang_score += lang_sup.proficiency;
                        matched += 1;
                        break;
                    }
                }
            }

            if matched < query.required_languages.len() {
                return 0.0;
            }

            score += lang_score / query.required_languages.len() as f64;
            factors += 1;
        }

        // Tool matching
        if !query.required_tools.is_empty() {
            let mut tool_matched = 0;

            for required in &query.required_tools {
                for tool in &cap.tools {
                    if &tool.tool_id == required && tool.enabled {
                        tool_matched += 1;
                        break;
                    }
                }
            }

            if tool_matched < query.required_tools.len() {
                return 0.0;
            }

            score += 1.0; // Tools are binary match
            factors += 1;
        }

        // Level bonus
        score += cap.level.0 as f64 / 4.0;
        factors += 1;

        if factors > 0 {
            score / factors as f64
        } else {
            0.5 // Default score if no specific requirements
        }
    }

    /// Remove a capability
    pub fn unregister(&mut self, model_id: &str) -> Option<CapabilityDocument> {
        self.capabilities.remove(model_id)
    }
}

impl Default for CapabilityRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Capability match result
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CapabilityMatch {
    pub model_id: String,
    pub score: f64,
    pub capability_uri: String,
}

// ============================================================================
// MESSAGE ROUTER
// ============================================================================

/// Message router for AI-to-AI communication
pub struct MessageRouter {
    registry: Arc<RwLock<CapabilityRegistry>>,
    routing_table: HashMap<String, String>, // model_id -> endpoint
}

impl MessageRouter {
    pub fn new(registry: Arc<RwLock<CapabilityRegistry>>) -> Self {
        Self {
            registry,
            routing_table: HashMap::new(),
        }
    }

    /// Register a route
    pub fn register_route(&mut self, model_id: &str, endpoint: &str) {
        self.routing_table.insert(model_id.to_string(), endpoint.to_string());
    }

    /// Route a message to destination
    pub async fn route(&self, message: &MessageEnvelope) -> Result<String, RoutingError> {
        // Direct routing
        if let Some(model_id) = &message.to.model_id {
            if let Some(endpoint) = self.routing_table.get(model_id) {
                return Ok(endpoint.clone());
            }
            return Err(RoutingError::NodeNotFound(model_id.clone()));
        }

        // Capability-based routing
        if let Some(query) = &message.to.capability_query {
            let registry = self.registry.read().await;
            let matches = registry.query(query);

            if let Some(best) = matches.first() {
                if let Some(endpoint) = self.routing_table.get(&best.model_id) {
                    return Ok(endpoint.clone());
                }
            }

            return Err(RoutingError::NoMatchingCapability);
        }

        Err(RoutingError::InvalidTarget)
    }

    /// Get all registered routes
    pub fn list_routes(&self) -> Vec<(&String, &String)> {
        self.routing_table.iter().collect()
    }
}

/// Routing errors
#[derive(Debug, thiserror::Error)]
pub enum RoutingError {
    #[error("Node not found: {0}")]
    NodeNotFound(String),

    #[error("No AI matches the capability requirements")]
    NoMatchingCapability,

    #[error("Invalid message target")]
    InvalidTarget,
}

// ============================================================================
// FEDERATION MANAGER
// ============================================================================

/// Federation manager
pub struct FederationManager {
    federations: HashMap<Uuid, FederationDocument>,
}

impl FederationManager {
    pub fn new() -> Self {
        Self {
            federations: HashMap::new(),
        }
    }

    /// Create a new federation
    pub fn create(&mut self, name: &str, topology: FederationTopology, config: FederationConfig) -> FederationDocument {
        let now = Utc::now();
        let federation = FederationDocument {
            wia_version: crate::WIA_VERSION.to_string(),
            document_type: "federation".to_string(),
            federation_id: Uuid::now_v7(),
            name: name.to_string(),
            description: None,
            created_at: now,
            updated_at: now,
            topology,
            members: Vec::new(),
            config,
            state: FederationState::Forming,
            signature: None,
        };

        self.federations.insert(federation.federation_id, federation.clone());
        federation
    }

    /// Get federation by ID
    pub fn get(&self, federation_id: &Uuid) -> Option<&FederationDocument> {
        self.federations.get(federation_id)
    }

    /// Get mutable federation
    pub fn get_mut(&mut self, federation_id: &Uuid) -> Option<&mut FederationDocument> {
        self.federations.get_mut(federation_id)
    }

    /// List all federations
    pub fn list(&self) -> Vec<&FederationDocument> {
        self.federations.values().collect()
    }

    /// Add member to federation
    pub fn add_member(
        &mut self,
        federation_id: &Uuid,
        model_id: &str,
        capability_uri: &str,
        role: FederationRole,
        domains: Vec<DomainCode>,
        endpoint: &str,
    ) -> Result<FederationMember, FederationError> {
        let federation = self.federations.get_mut(federation_id)
            .ok_or(FederationError::NotFound(*federation_id))?;

        // Check if already a member
        if federation.members.iter().any(|m| m.model_id == model_id) {
            return Err(FederationError::AlreadyMember(model_id.to_string()));
        }

        let member = FederationMember {
            member_id: Uuid::now_v7(),
            model_id: model_id.to_string(),
            capability_uri: capability_uri.to_string(),
            role,
            domains,
            status: MemberStatus::Joining,
            metrics: None,
            endpoint: endpoint.to_string(),
            last_seen: Some(Utc::now()),
        };

        federation.members.push(member.clone());
        federation.updated_at = Utc::now();

        // Update state if first orchestrator
        if federation.state == FederationState::Forming &&
           member.role == FederationRole::Orchestrator {
            federation.state = FederationState::Active;
        }

        Ok(member)
    }

    /// Remove member from federation
    pub fn remove_member(&mut self, federation_id: &Uuid, member_id: &Uuid) -> Result<(), FederationError> {
        let federation = self.federations.get_mut(federation_id)
            .ok_or(FederationError::NotFound(*federation_id))?;

        let initial_len = federation.members.len();
        federation.members.retain(|m| &m.member_id != member_id);

        if federation.members.len() == initial_len {
            return Err(FederationError::MemberNotFound(*member_id));
        }

        federation.updated_at = Utc::now();

        // Check if federation should be degraded
        let has_orchestrator = federation.members.iter().any(|m| m.role == FederationRole::Orchestrator);
        if !has_orchestrator && federation.state == FederationState::Active {
            federation.state = FederationState::Degraded;
        }

        Ok(())
    }

    /// Update member status
    pub fn update_member_status(
        &mut self,
        federation_id: &Uuid,
        member_id: &Uuid,
        status: MemberStatus,
    ) -> Result<(), FederationError> {
        let federation = self.federations.get_mut(federation_id)
            .ok_or(FederationError::NotFound(*federation_id))?;

        let member = federation.members.iter_mut()
            .find(|m| &m.member_id == member_id)
            .ok_or(FederationError::MemberNotFound(*member_id))?;

        member.status = status;
        member.last_seen = Some(Utc::now());
        federation.updated_at = Utc::now();

        Ok(())
    }

    /// Dissolve federation
    pub fn dissolve(&mut self, federation_id: &Uuid) -> Result<(), FederationError> {
        let federation = self.federations.get_mut(federation_id)
            .ok_or(FederationError::NotFound(*federation_id))?;

        federation.state = FederationState::Dissolved;
        federation.updated_at = Utc::now();

        Ok(())
    }
}

impl Default for FederationManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Federation errors
#[derive(Debug, thiserror::Error)]
pub enum FederationError {
    #[error("Federation not found: {0}")]
    NotFound(Uuid),

    #[error("Already a member: {0}")]
    AlreadyMember(String),

    #[error("Member not found: {0}")]
    MemberNotFound(Uuid),

    #[error("Invalid state transition")]
    InvalidState,
}

// ============================================================================
// TASK MANAGER
// ============================================================================

/// Task manager for federation task handling
pub struct TaskManager {
    tasks: HashMap<Uuid, TaskDocument>,
}

/// Task document
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct TaskDocument {
    pub task_id: Uuid,
    pub parent_task_id: Option<Uuid>,
    pub federation_id: Uuid,
    pub original_query: String,
    pub description: String,
    pub assigned_to: Option<String>,
    pub status: TaskStatus,
    pub result: Option<TaskResult>,
    pub created_at: chrono::DateTime<Utc>,
    pub started_at: Option<chrono::DateTime<Utc>>,
    pub completed_at: Option<chrono::DateTime<Utc>>,
    pub deadline: Option<chrono::DateTime<Utc>>,
    pub subtasks: Vec<Uuid>,
}

impl TaskManager {
    pub fn new() -> Self {
        Self {
            tasks: HashMap::new(),
        }
    }

    /// Create a new task
    pub fn create_task(
        &mut self,
        federation_id: Uuid,
        query: &str,
        description: &str,
        deadline: Option<chrono::DateTime<Utc>>,
    ) -> TaskDocument {
        let task = TaskDocument {
            task_id: Uuid::now_v7(),
            parent_task_id: None,
            federation_id,
            original_query: query.to_string(),
            description: description.to_string(),
            assigned_to: None,
            status: TaskStatus::Pending,
            result: None,
            created_at: Utc::now(),
            started_at: None,
            completed_at: None,
            deadline,
            subtasks: Vec::new(),
        };

        self.tasks.insert(task.task_id, task.clone());
        task
    }

    /// Create a subtask
    pub fn create_subtask(
        &mut self,
        parent_task_id: Uuid,
        description: &str,
    ) -> Result<TaskDocument, TaskError> {
        let parent = self.tasks.get(&parent_task_id)
            .ok_or(TaskError::NotFound(parent_task_id))?;

        let subtask = TaskDocument {
            task_id: Uuid::now_v7(),
            parent_task_id: Some(parent_task_id),
            federation_id: parent.federation_id,
            original_query: parent.original_query.clone(),
            description: description.to_string(),
            assigned_to: None,
            status: TaskStatus::Pending,
            result: None,
            created_at: Utc::now(),
            started_at: None,
            completed_at: None,
            deadline: parent.deadline,
            subtasks: Vec::new(),
        };

        let subtask_id = subtask.task_id;
        self.tasks.insert(subtask_id, subtask.clone());

        // Add to parent's subtask list
        if let Some(parent) = self.tasks.get_mut(&parent_task_id) {
            parent.subtasks.push(subtask_id);
        }

        Ok(subtask)
    }

    /// Get task by ID
    pub fn get(&self, task_id: &Uuid) -> Option<&TaskDocument> {
        self.tasks.get(task_id)
    }

    /// Assign task to a model
    pub fn assign(&mut self, task_id: &Uuid, model_id: &str) -> Result<(), TaskError> {
        let task = self.tasks.get_mut(task_id)
            .ok_or(TaskError::NotFound(*task_id))?;

        task.assigned_to = Some(model_id.to_string());
        task.status = TaskStatus::Queued;

        Ok(())
    }

    /// Start task execution
    pub fn start(&mut self, task_id: &Uuid) -> Result<(), TaskError> {
        let task = self.tasks.get_mut(task_id)
            .ok_or(TaskError::NotFound(*task_id))?;

        task.status = TaskStatus::InProgress;
        task.started_at = Some(Utc::now());

        Ok(())
    }

    /// Complete task with result
    pub fn complete(&mut self, task_id: &Uuid, result: TaskResult) -> Result<(), TaskError> {
        let task = self.tasks.get_mut(task_id)
            .ok_or(TaskError::NotFound(*task_id))?;

        task.status = if result.success { TaskStatus::Completed } else { TaskStatus::Failed };
        task.result = Some(result);
        task.completed_at = Some(Utc::now());

        Ok(())
    }

    /// List tasks by federation
    pub fn list_by_federation(&self, federation_id: &Uuid) -> Vec<&TaskDocument> {
        self.tasks.values()
            .filter(|t| &t.federation_id == federation_id)
            .collect()
    }
}

impl Default for TaskManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Task errors
#[derive(Debug, thiserror::Error)]
pub enum TaskError {
    #[error("Task not found: {0}")]
    NotFound(Uuid),

    #[error("Invalid task state")]
    InvalidState,
}

// ============================================================================
// CONSENSUS ENGINE
// ============================================================================

/// Consensus engine for multi-AI voting
pub struct ConsensusEngine {
    sessions: HashMap<Uuid, ConsensusSession>,
}

/// Consensus session
#[derive(Debug, Clone)]
pub struct ConsensusSession {
    pub consensus_id: Uuid,
    pub federation_id: Uuid,
    pub question: ConsensusQuestion,
    pub votes: Vec<ConsensusVote>,
    pub deadline: chrono::DateTime<Utc>,
    pub config: ConsensusConfig,
    pub result: Option<ConsensusResult>,
}

impl ConsensusEngine {
    pub fn new() -> Self {
        Self {
            sessions: HashMap::new(),
        }
    }

    /// Start a consensus session
    pub fn start_session(
        &mut self,
        federation_id: Uuid,
        question: ConsensusQuestion,
        config: ConsensusConfig,
        timeout_seconds: u32,
    ) -> ConsensusSession {
        let session = ConsensusSession {
            consensus_id: Uuid::now_v7(),
            federation_id,
            question,
            votes: Vec::new(),
            deadline: Utc::now() + chrono::Duration::seconds(timeout_seconds as i64),
            config,
            result: None,
        };

        self.sessions.insert(session.consensus_id, session.clone());
        session
    }

    /// Submit a vote
    pub fn vote(
        &mut self,
        consensus_id: &Uuid,
        voter_id: &str,
        vote: serde_json::Value,
        confidence: f64,
        reasoning: Option<String>,
    ) -> Result<(), ConsensusError> {
        let session = self.sessions.get_mut(consensus_id)
            .ok_or(ConsensusError::NotFound(*consensus_id))?;

        // Check if already voted
        if session.votes.iter().any(|v| v.voter_id == voter_id) {
            return Err(ConsensusError::AlreadyVoted(voter_id.to_string()));
        }

        // Check deadline
        if Utc::now() > session.deadline {
            return Err(ConsensusError::DeadlinePassed);
        }

        session.votes.push(ConsensusVote {
            voter_id: voter_id.to_string(),
            vote,
            confidence,
            reasoning,
            timestamp: Utc::now(),
        });

        Ok(())
    }

    /// Finalize consensus and calculate result
    pub fn finalize(&mut self, consensus_id: &Uuid) -> Result<ConsensusResult, ConsensusError> {
        let session = self.sessions.get_mut(consensus_id)
            .ok_or(ConsensusError::NotFound(*consensus_id))?;

        if session.result.is_some() {
            return session.result.clone().ok_or(ConsensusError::AlreadyFinalized);
        }

        let result = match session.config.algorithm {
            ConsensusAlgorithm::Majority => self.majority_vote(&session.votes, session.config.quorum_percentage),
            ConsensusAlgorithm::Weighted => self.weighted_vote(&session.votes, session.config.quorum_percentage),
            _ => self.majority_vote(&session.votes, session.config.quorum_percentage),
        };

        session.result = Some(result.clone());
        Ok(result)
    }

    fn majority_vote(&self, votes: &[ConsensusVote], quorum: f64) -> ConsensusResult {
        let mut counts: HashMap<String, u32> = HashMap::new();

        for vote in votes {
            let key = vote.vote.to_string();
            *counts.entry(key).or_insert(0) += 1;
        }

        let total = votes.len() as u32;
        let (outcome, count) = counts.iter()
            .max_by_key(|(_, c)| *c)
            .map(|(k, c)| (k.clone(), *c))
            .unwrap_or(("abstain".to_string(), 0));

        let quorum_reached = (count as f64 / total as f64) >= quorum;

        ConsensusResult {
            outcome,
            confidence: count as f64 / total as f64,
            vote_count: total,
            quorum_reached,
            vote_breakdown: counts.into_iter().map(|(k, v)| (k, v)).collect(),
            dissenting_votes: vec![],
        }
    }

    fn weighted_vote(&self, votes: &[ConsensusVote], quorum: f64) -> ConsensusResult {
        let mut weighted_scores: HashMap<String, f64> = HashMap::new();

        for vote in votes {
            let key = vote.vote.to_string();
            *weighted_scores.entry(key).or_insert(0.0) += vote.confidence;
        }

        let total_weight: f64 = weighted_scores.values().sum();
        let (outcome, weight) = weighted_scores.iter()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(k, w)| (k.clone(), *w))
            .unwrap_or(("abstain".to_string(), 0.0));

        let quorum_reached = (weight / total_weight) >= quorum;

        ConsensusResult {
            outcome,
            confidence: weight / total_weight,
            vote_count: votes.len() as u32,
            quorum_reached,
            vote_breakdown: weighted_scores.into_iter()
                .map(|(k, v)| (k, v as u32))
                .collect(),
            dissenting_votes: vec![],
        }
    }

    /// Get session by ID
    pub fn get(&self, consensus_id: &Uuid) -> Option<&ConsensusSession> {
        self.sessions.get(consensus_id)
    }
}

impl Default for ConsensusEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Consensus errors
#[derive(Debug, thiserror::Error)]
pub enum ConsensusError {
    #[error("Consensus session not found: {0}")]
    NotFound(Uuid),

    #[error("Already voted: {0}")]
    AlreadyVoted(String),

    #[error("Voting deadline has passed")]
    DeadlinePassed,

    #[error("Consensus already finalized")]
    AlreadyFinalized,
}

// ============================================================================
// CIRCUIT BREAKER
// ============================================================================

/// Circuit breaker state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircuitState {
    Closed,
    Open,
    HalfOpen,
}

/// Circuit breaker for fault tolerance
pub struct CircuitBreaker {
    state: CircuitState,
    failure_count: u32,
    success_count: u32,
    last_failure: Option<chrono::DateTime<Utc>>,
    config: CircuitBreakerConfig,
}

/// Circuit breaker config
#[derive(Debug, Clone)]
pub struct CircuitBreakerConfig {
    pub failure_threshold: u32,
    pub success_threshold: u32,
    pub timeout_seconds: u32,
    pub half_open_requests: u32,
}

impl CircuitBreaker {
    pub fn new(config: CircuitBreakerConfig) -> Self {
        Self {
            state: CircuitState::Closed,
            failure_count: 0,
            success_count: 0,
            last_failure: None,
            config,
        }
    }

    /// Check if request should be allowed
    pub fn allow_request(&mut self) -> bool {
        match self.state {
            CircuitState::Closed => true,
            CircuitState::Open => {
                if let Some(last) = self.last_failure {
                    if Utc::now() > last + chrono::Duration::seconds(self.config.timeout_seconds as i64) {
                        self.state = CircuitState::HalfOpen;
                        self.success_count = 0;
                        return true;
                    }
                }
                false
            }
            CircuitState::HalfOpen => {
                self.success_count < self.config.half_open_requests
            }
        }
    }

    /// Record a successful request
    pub fn record_success(&mut self) {
        match self.state {
            CircuitState::Closed => {
                self.failure_count = self.failure_count.saturating_sub(1);
            }
            CircuitState::HalfOpen => {
                self.success_count += 1;
                if self.success_count >= self.config.success_threshold {
                    self.state = CircuitState::Closed;
                    self.failure_count = 0;
                }
            }
            CircuitState::Open => {}
        }
    }

    /// Record a failed request
    pub fn record_failure(&mut self) {
        self.last_failure = Some(Utc::now());

        match self.state {
            CircuitState::Closed => {
                self.failure_count += 1;
                if self.failure_count >= self.config.failure_threshold {
                    self.state = CircuitState::Open;
                }
            }
            CircuitState::HalfOpen => {
                self.state = CircuitState::Open;
            }
            CircuitState::Open => {}
        }
    }

    /// Get current state
    pub fn state(&self) -> CircuitState {
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_capability_registry() {
        let mut registry = CapabilityRegistry::new();

        // Create a mock capability
        let cap = create_mock_capability("test-ai", vec![DomainCode::Code]);
        registry.register(cap);

        assert!(registry.get("did:wia:llm:test:test-ai").is_some());
    }

    #[test]
    fn test_federation_lifecycle() {
        let mut manager = FederationManager::new();

        let config = create_default_federation_config();
        let federation = manager.create("Test Federation", FederationTopology::Star, config);

        assert_eq!(federation.state, FederationState::Forming);

        // Add orchestrator
        let result = manager.add_member(
            &federation.federation_id,
            "did:wia:llm:test:orchestrator",
            "https://example.com/capability",
            FederationRole::Orchestrator,
            vec![DomainCode::General],
            "https://example.com/api",
        );

        assert!(result.is_ok());

        // Check state changed to Active
        let fed = manager.get(&federation.federation_id).unwrap();
        assert_eq!(fed.state, FederationState::Active);
    }

    #[test]
    fn test_circuit_breaker() {
        let config = CircuitBreakerConfig {
            failure_threshold: 3,
            success_threshold: 2,
            timeout_seconds: 10,
            half_open_requests: 1,
        };

        let mut cb = CircuitBreaker::new(config);

        assert!(cb.allow_request());
        assert_eq!(cb.state(), CircuitState::Closed);

        // Simulate failures
        cb.record_failure();
        cb.record_failure();
        cb.record_failure();

        assert_eq!(cb.state(), CircuitState::Open);
        assert!(!cb.allow_request());
    }

    fn create_mock_capability(name: &str, domains: Vec<DomainCode>) -> CapabilityDocument {
        let now = Utc::now();
        CapabilityDocument {
            wia_version: "1.0".to_string(),
            document_type: "capability".to_string(),
            document_id: Uuid::now_v7(),
            created_at: now,
            updated_at: now,
            model: ModelIdentity {
                model_id: format!("did:wia:llm:test:{}", name),
                display_name: name.to_string(),
                short_name: name.to_string(),
                provider: ProviderInfo {
                    id: "test".to_string(),
                    name: "Test Provider".to_string(),
                    url: "https://test.com".to_string(),
                    country: None,
                },
                version: "1.0.0".to_string(),
                release_date: None,
                family: None,
                family_rank: None,
            },
            level: CapabilityLevel(4),
            domains: domains.into_iter().map(|d| DomainExpertise {
                domain: d,
                proficiency: 0.9,
                sub_domains: vec![],
                certifications: vec![],
                benchmarks: vec![],
            }).collect(),
            languages: vec![],
            modalities: ModalitySupport {
                text: TextModality {
                    input: true,
                    output: true,
                    max_input_length: 100000,
                    max_output_length: 10000,
                    streaming: true,
                },
                image: None,
                audio: None,
                video: None,
                code: None,
            },
            tools: vec![],
            limits: ResourceLimits {
                max_input_tokens: 100000,
                max_output_tokens: 10000,
                max_context_window: 100000,
                rate_limits: RateLimits {
                    requests_per_minute: 100,
                    tokens_per_minute: 100000,
                    tokens_per_day: None,
                    burst_requests: None,
                    burst_window_seconds: None,
                },
                max_response_time_seconds: 60,
                max_concurrent_requests: None,
            },
            trust: TrustMetadata {
                confidence_threshold: 0.7,
                hallucination_rate: None,
                knowledge_cutoff: now,
                safety: SafetyInfo {
                    content_filtering: true,
                    nsfw_filter: true,
                    hate_speech_filter: true,
                    violence_filter: true,
                    self_harm_filter: true,
                    refusal_categories: vec![],
                },
                compliance: ComplianceInfo {
                    gdpr_compliant: true,
                    hipaa_compliant: None,
                    sox_compliant: None,
                    iso27001: None,
                    data_residency: None,
                    audit_logging: true,
                },
            },
            endpoints: EndpointInfo {
                api: ApiEndpoint {
                    url: "https://test.com/api".to_string(),
                    protocol: "https".to_string(),
                    version: "1.0".to_string(),
                    auth: vec![AuthMethod::ApiKey],
                },
                capability: "https://test.com/capability".to_string(),
                documentation: None,
                support: None,
            },
            signature: None,
        }
    }

    fn create_default_federation_config() -> FederationConfig {
        FederationConfig {
            task_config: TaskConfig {
                auto_decompose: true,
                max_subtasks: 10,
                max_parallel_tasks: 5,
                max_retries: 3,
                retry_delay_ms: 1000,
            },
            consensus_config: ConsensusConfig {
                algorithm: ConsensusAlgorithm::Majority,
                quorum_percentage: 0.5,
                voting_timeout_seconds: 30,
                tie_breaker: TieBreaker::Orchestrator,
            },
            fault_tolerance: FaultToleranceConfig {
                circuit_breaker_enabled: true,
                failure_threshold: 3,
                success_threshold: 2,
                timeout_seconds: 10,
                failover_enabled: true,
                min_members_for_operation: 1,
            },
            timeouts: TimeoutConfig {
                handshake_timeout_seconds: 10,
                task_timeout_seconds: 300,
                response_timeout_seconds: 60,
                heartbeat_interval_seconds: 30,
                member_timeout_seconds: 120,
            },
        }
    }
}
