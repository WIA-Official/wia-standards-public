//! WIA-LLM-INTEROP Type Definitions
//!
//! Data structures for Capability, Message, and Federation.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

// ============================================================================
// CAPABILITY TYPES
// ============================================================================

/// Capability document for an AI model
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapabilityDocument {
    pub wia_version: String,
    pub document_type: String,
    pub document_id: Uuid,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,

    pub model: ModelIdentity,
    pub level: CapabilityLevel,
    pub domains: Vec<DomainExpertise>,
    pub languages: Vec<LanguageSupport>,
    pub modalities: ModalitySupport,
    pub tools: Vec<ToolCapability>,
    pub limits: ResourceLimits,
    pub trust: TrustMetadata,
    pub endpoints: EndpointInfo,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<CryptographicSignature>,
}

/// Model identity information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelIdentity {
    pub model_id: String,
    pub display_name: String,
    pub short_name: String,
    pub provider: ProviderInfo,
    pub version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub release_date: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub family: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub family_rank: Option<u8>,
}

/// Provider information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProviderInfo {
    pub id: String,
    pub name: String,
    pub url: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub country: Option<String>,
}

/// Capability level (1-4)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CapabilityLevel(pub u8);

impl CapabilityLevel {
    pub const BASIC: Self = Self(1);
    pub const REASONING: Self = Self(2);
    pub const SPECIALIST: Self = Self(3);
    pub const ADVANCED: Self = Self(4);

    pub fn description(&self) -> &'static str {
        match self.0 {
            1 => "Basic: Simple Q&A, text generation",
            2 => "Reasoning: Multi-step reasoning, analysis",
            3 => "Specialist: Domain expert, specialized tasks",
            4 => "Advanced: Tool use, multi-modal, agentic",
            _ => "Unknown",
        }
    }
}

/// Domain expertise
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DomainExpertise {
    pub domain: DomainCode,
    pub proficiency: f64,
    #[serde(default)]
    pub sub_domains: Vec<SubDomainExpertise>,
    #[serde(default)]
    pub certifications: Vec<Certification>,
    #[serde(default)]
    pub benchmarks: Vec<BenchmarkResult>,
}

/// Standard domain codes
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DomainCode {
    Code,
    Math,
    Science,
    Engineering,
    DataScience,
    Cybersecurity,
    Medical,
    Legal,
    Finance,
    Education,
    Writing,
    Translation,
    Marketing,
    General,
    Reasoning,
    Research,
    #[serde(other)]
    Other,
}

/// Sub-domain expertise
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubDomainExpertise {
    pub name: String,
    pub proficiency: f64,
}

/// Certification information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Certification {
    pub name: String,
    pub issuer: String,
    pub issued_date: DateTime<Utc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expiry_date: Option<DateTime<Utc>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub verification_url: Option<String>,
}

/// Benchmark result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkResult {
    pub benchmark: String,
    pub score: f64,
    pub max_score: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub percentile: Option<f64>,
    pub test_date: DateTime<Utc>,
}

/// Language support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LanguageSupport {
    pub code: String,
    pub proficiency: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<LanguageCapabilities>,
    #[serde(default)]
    pub scripts: Vec<String>,
}

/// Language-specific capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LanguageCapabilities {
    pub understanding: f64,
    pub generation: f64,
    pub translation_from: f64,
    pub translation_to: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code_comments: Option<f64>,
}

/// Modality support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModalitySupport {
    pub text: TextModality,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub image: Option<ImageModality>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audio: Option<AudioModality>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub video: Option<VideoModality>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code: Option<CodeModality>,
}

/// Text modality
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TextModality {
    pub input: bool,
    pub output: bool,
    pub max_input_length: u32,
    pub max_output_length: u32,
    pub streaming: bool,
}

/// Image modality
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageModality {
    pub input: bool,
    pub output: bool,
    #[serde(default)]
    pub input_formats: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_resolution: Option<String>,
    pub ocr: bool,
    pub diagram_understanding: bool,
}

/// Audio modality
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioModality {
    pub input: bool,
    pub output: bool,
    #[serde(default)]
    pub input_formats: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_duration_seconds: Option<u32>,
    pub transcription: bool,
}

/// Video modality
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VideoModality {
    pub input: bool,
    pub output: bool,
    #[serde(default)]
    pub input_formats: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_duration_seconds: Option<u32>,
    pub frame_analysis: bool,
    pub temporal_understanding: bool,
}

/// Code modality
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CodeModality {
    pub input: bool,
    pub output: bool,
    pub execution: bool,
    #[serde(default)]
    pub languages: Vec<CodeLanguageSupport>,
}

/// Code language support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CodeLanguageSupport {
    pub language: String,
    pub proficiency: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub execution: Option<bool>,
}

/// Tool capability
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolCapability {
    pub tool_id: String,
    pub name: String,
    pub description: String,
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub limits: Option<ToolLimits>,
}

/// Tool limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolLimits {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calls_per_minute: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calls_per_session: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
}

/// Resource limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceLimits {
    pub max_input_tokens: u32,
    pub max_output_tokens: u32,
    pub max_context_window: u32,
    pub rate_limits: RateLimits,
    pub max_response_time_seconds: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_concurrent_requests: Option<u32>,
}

/// Rate limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateLimits {
    pub requests_per_minute: u32,
    pub tokens_per_minute: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tokens_per_day: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub burst_requests: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub burst_window_seconds: Option<u32>,
}

/// Trust metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrustMetadata {
    pub confidence_threshold: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hallucination_rate: Option<f64>,
    pub knowledge_cutoff: DateTime<Utc>,
    pub safety: SafetyInfo,
    pub compliance: ComplianceInfo,
}

/// Safety information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyInfo {
    pub content_filtering: bool,
    pub nsfw_filter: bool,
    pub hate_speech_filter: bool,
    pub violence_filter: bool,
    pub self_harm_filter: bool,
    #[serde(default)]
    pub refusal_categories: Vec<String>,
}

/// Compliance information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceInfo {
    pub gdpr_compliant: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hipaa_compliant: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sox_compliant: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub iso27001: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data_residency: Option<Vec<String>>,
    pub audit_logging: bool,
}

/// Endpoint information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EndpointInfo {
    pub api: ApiEndpoint,
    pub capability: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub documentation: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub support: Option<SupportInfo>,
}

/// API endpoint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiEndpoint {
    pub url: String,
    pub protocol: String,
    pub version: String,
    pub auth: Vec<AuthMethod>,
}

/// Authentication method
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthMethod {
    ApiKey,
    Oauth2,
    Jwt,
    Mtls,
    DidAuth,
}

/// Support information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupportInfo {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub email: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slack: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub discord: Option<String>,
}

// ============================================================================
// MESSAGE TYPES
// ============================================================================

/// Message envelope for AI-to-AI communication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageEnvelope {
    pub wia_version: String,
    pub document_type: String,
    pub message_id: Uuid,
    pub timestamp: DateTime<Utc>,
    pub trace_id: Uuid,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parent_message_id: Option<Uuid>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sequence_number: Option<u32>,

    pub from: MessageParticipant,
    pub to: MessageTarget,

    #[serde(rename = "type")]
    pub message_type: MessageType,
    pub priority: Priority,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ttl_seconds: Option<u32>,

    pub payload: MessagePayload,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<CryptographicSignature>,
}

/// Message participant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageParticipant {
    pub model_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capability_uri: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instance_id: Option<String>,
}

/// Message target
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageTarget {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capability_query: Option<CapabilityQuery>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub broadcast: Option<BroadcastScope>,
}

/// Capability query for routing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapabilityQuery {
    #[serde(default)]
    pub required_domains: Vec<DomainCode>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_level: Option<u8>,
    #[serde(default)]
    pub required_languages: Vec<String>,
    #[serde(default)]
    pub required_tools: Vec<String>,
}

/// Broadcast scope
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BroadcastScope {
    Federation,
    Domain,
    All,
}

/// Message type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MessageType {
    Query,
    Response,
    Stream,
    Handshake,
    Heartbeat,
    Ack,
    Error,
    TaskAssignment,
    TaskResult,
    ConsensusRequest,
    ConsensusVote,
    CapabilityRequest,
    CapabilityResponse,
}

/// Priority level
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Priority {
    Low,
    Normal,
    High,
    Critical,
}

/// Message payload (tagged union)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MessagePayload {
    Query(QueryPayload),
    Response(ResponsePayload),
    Stream(StreamPayload),
    Error(ErrorPayload),
    Handshake(HandshakePayload),
    Task(TaskPayload),
    Consensus(ConsensusPayload),
}

/// Query payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryPayload {
    pub intent: QueryIntent,
    pub content: String,
    #[serde(default)]
    pub context: Vec<ConversationContext>,
    pub constraints: QueryConstraints,
    #[serde(default)]
    pub attachments: Vec<Attachment>,
}

/// Query intent
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QueryIntent {
    Question,
    Task,
    Analysis,
    Generation,
    Translation,
    Verification,
    Delegation,
}

/// Conversation context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationContext {
    pub role: String,
    pub content: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<DateTime<Utc>>,
}

/// Query constraints
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryConstraints {
    pub response_format: ResponseFormat,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_tokens: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub response_language: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence_min: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub require_reasoning: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub require_sources: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stream: Option<bool>,
}

/// Response format
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponseFormat {
    Text,
    Json,
    Markdown,
    Code,
}

/// Attachment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Attachment {
    pub attachment_id: Uuid,
    #[serde(rename = "type")]
    pub attachment_type: AttachmentType,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filename: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mime_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size_bytes: Option<u64>,
}

/// Attachment type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AttachmentType {
    Text,
    Image,
    Audio,
    File,
    Data,
}

/// Response payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponsePayload {
    pub content: String,
    pub confidence: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reasoning: Option<String>,
    #[serde(default)]
    pub sources: Vec<Source>,
    #[serde(default)]
    pub suggestions: Vec<Suggestion>,
    #[serde(default)]
    pub delegations: Vec<DelegationSuggestion>,
    pub usage: UsageStats,
}

/// Source/citation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Source {
    pub title: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub excerpt: Option<String>,
    pub relevance: f64,
}

/// Suggestion for follow-up
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Suggestion {
    #[serde(rename = "type")]
    pub suggestion_type: SuggestionType,
    pub content: String,
    pub relevance: f64,
}

/// Suggestion type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuggestionType {
    Question,
    Action,
    Clarification,
}

/// Delegation suggestion
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DelegationSuggestion {
    pub reason: String,
    pub suggested_capability: CapabilityQuery,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub partial_result: Option<String>,
}

/// Usage statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UsageStats {
    pub input_tokens: u32,
    pub output_tokens: u32,
    pub total_tokens: u32,
    pub processing_time_ms: u64,
    pub model_id: String,
}

/// Stream payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamPayload {
    pub chunk_index: u32,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_chunks: Option<u32>,
    pub content: String,
    pub is_final: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_metadata: Option<StreamFinalMetadata>,
}

/// Stream final metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamFinalMetadata {
    pub confidence: f64,
    pub usage: UsageStats,
    #[serde(default)]
    pub sources: Vec<Source>,
}

/// Error payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorPayload {
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    pub recoverable: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_after_seconds: Option<u32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fallback: Option<FallbackSuggestion>,
}

/// Fallback suggestion
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FallbackSuggestion {
    #[serde(rename = "type")]
    pub fallback_type: FallbackType,
    pub details: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub suggested_action: Option<serde_json::Value>,
}

/// Fallback type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FallbackType {
    Retry,
    Delegate,
    Simplify,
    Split,
}

/// Handshake payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HandshakePayload {
    pub phase: HandshakePhase,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capability: Option<CapabilityDocument>,
    #[serde(default)]
    pub supported_versions: Vec<String>,
    pub preferred_encoding: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compression: Option<String>,
    #[serde(default)]
    pub supported_auth: Vec<AuthMethod>,
}

/// Handshake phase
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HandshakePhase {
    Init,
    CapabilityExchange,
    Negotiation,
    Auth,
    Complete,
}

/// Task payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskPayload {
    pub task_id: Uuid,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parent_task_id: Option<Uuid>,
    pub description: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assigned_to: Option<String>,
    pub status: TaskStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<TaskResult>,
}

/// Task status
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TaskStatus {
    Pending,
    Queued,
    InProgress,
    WaitingDependency,
    Completed,
    Failed,
    Cancelled,
    Timeout,
}

/// Task result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskResult {
    pub success: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub confidence: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<ErrorPayload>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub usage: Option<UsageStats>,
}

/// Consensus payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsensusPayload {
    pub consensus_id: Uuid,
    pub question: ConsensusQuestion,
    #[serde(default)]
    pub votes: Vec<ConsensusVote>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<ConsensusResult>,
}

/// Consensus question
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsensusQuestion {
    #[serde(rename = "type")]
    pub question_type: ConsensusQuestionType,
    pub description: String,
    #[serde(default)]
    pub options: Vec<ConsensusOption>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context: Option<String>,
}

/// Consensus question type
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConsensusQuestionType {
    YesNo,
    MultipleChoice,
    FreeForm,
    Ranking,
}

/// Consensus option
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsensusOption {
    pub option_id: String,
    pub description: String,
}

/// Consensus vote
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsensusVote {
    pub voter_id: String,
    pub vote: serde_json::Value,
    pub confidence: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reasoning: Option<String>,
    pub timestamp: DateTime<Utc>,
}

/// Consensus result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsensusResult {
    pub outcome: String,
    pub confidence: f64,
    pub vote_count: u32,
    pub quorum_reached: bool,
    pub vote_breakdown: std::collections::HashMap<String, u32>,
    #[serde(default)]
    pub dissenting_votes: Vec<ConsensusVote>,
}

// ============================================================================
// FEDERATION TYPES
// ============================================================================

/// Federation document
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FederationDocument {
    pub wia_version: String,
    pub document_type: String,
    pub federation_id: Uuid,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub topology: FederationTopology,
    pub members: Vec<FederationMember>,
    pub config: FederationConfig,
    pub state: FederationState,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<CryptographicSignature>,
}

/// Federation topology
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FederationTopology {
    Star,
    Mesh,
    Hierarchical,
    Ring,
}

/// Federation state
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FederationState {
    Forming,
    Active,
    Degraded,
    Suspended,
    Dissolved,
}

/// Federation member
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FederationMember {
    pub member_id: Uuid,
    pub model_id: String,
    pub capability_uri: String,
    pub role: FederationRole,
    #[serde(default)]
    pub domains: Vec<DomainCode>,
    pub status: MemberStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<MemberMetrics>,
    pub endpoint: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_seen: Option<DateTime<Utc>>,
}

/// Federation role
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FederationRole {
    Orchestrator,
    Specialist,
    Router,
    Aggregator,
    Validator,
    Observer,
}

/// Member status
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MemberStatus {
    Joining,
    Active,
    Busy,
    Unavailable,
    Leaving,
}

/// Member metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MemberMetrics {
    pub requests_handled: u64,
    pub average_response_time_ms: f64,
    pub success_rate: f64,
    pub uptime_percentage: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_error: Option<String>,
}

/// Federation config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FederationConfig {
    pub task_config: TaskConfig,
    pub consensus_config: ConsensusConfig,
    pub fault_tolerance: FaultToleranceConfig,
    pub timeouts: TimeoutConfig,
}

/// Task config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskConfig {
    pub auto_decompose: bool,
    pub max_subtasks: u32,
    pub max_parallel_tasks: u32,
    pub max_retries: u32,
    pub retry_delay_ms: u64,
}

/// Consensus config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConsensusConfig {
    pub algorithm: ConsensusAlgorithm,
    pub quorum_percentage: f64,
    pub voting_timeout_seconds: u32,
    pub tie_breaker: TieBreaker,
}

/// Consensus algorithm
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConsensusAlgorithm {
    Majority,
    Weighted,
    Unanimous,
    Raft,
    Pbft,
}

/// Tie breaker strategy
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TieBreaker {
    Orchestrator,
    HighestConfidence,
    Random,
    Abstain,
}

/// Fault tolerance config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultToleranceConfig {
    pub circuit_breaker_enabled: bool,
    pub failure_threshold: u32,
    pub success_threshold: u32,
    pub timeout_seconds: u32,
    pub failover_enabled: bool,
    pub min_members_for_operation: u32,
}

/// Timeout config
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeoutConfig {
    pub handshake_timeout_seconds: u32,
    pub task_timeout_seconds: u32,
    pub response_timeout_seconds: u32,
    pub heartbeat_interval_seconds: u32,
    pub member_timeout_seconds: u32,
}

// ============================================================================
// SHARED TYPES
// ============================================================================

/// Cryptographic signature
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CryptographicSignature {
    pub algorithm: SignatureAlgorithm,
    pub public_key_id: String,
    pub signature: String,
    pub timestamp: DateTime<Utc>,
}

/// Signature algorithm
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignatureAlgorithm {
    Ed25519,
    RsaPss,
    EcdsaP256,
    Dilithium,
    Sphincs,
}
