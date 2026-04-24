//! WIA AI Type Definitions
//!
//! This module contains all type definitions for the WIA AI Standard,
//! including models, agents, experiments, evaluations, safety assessments,
//! prompts, swarms, and tools.

use chrono::{DateTime, NaiveDate, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Common Types
// ============================================================================

/// WIA version string (e.g., "1.0.0")
pub type WiaVersion = String;

// ============================================================================
// Model Types
// ============================================================================

/// AI Model metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Model {
    /// WIA specification version
    pub wia_version: WiaVersion,
    /// Unique model identifier (e.g., "model-claude-opus-001")
    pub model_id: String,
    /// Model information
    pub model_info: ModelInfo,
    /// Model architecture
    pub architecture: Architecture,
    /// Model capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<Capabilities>,
    /// Training information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub training: Option<Training>,
    /// Safety information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety: Option<ModelSafety>,
    /// Deployment information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deployment: Option<Deployment>,
    /// Licensing information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub licensing: Option<Licensing>,
}

/// Model information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelInfo {
    /// Model name
    pub name: String,
    /// Model family (e.g., "gpt", "claude", "llama")
    #[serde(skip_serializing_if = "Option::is_none")]
    pub family: Option<String>,
    /// Model version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Release date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub release_date: Option<NaiveDate>,
    /// Provider organization
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider: Option<String>,
}

/// Architecture type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ArchitectureType {
    Transformer,
    TransformerDecoder,
    TransformerEncDec,
    Moe,
    Ssm,
    Diffusion,
    Neuromorphic,
    Hybrid,
    Other,
}

/// Model architecture
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Architecture {
    /// Architecture type
    #[serde(rename = "type")]
    pub arch_type: ArchitectureType,
    /// Architecture variant
    #[serde(skip_serializing_if = "Option::is_none")]
    pub variant: Option<String>,
    /// Parameter counts
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<Parameters>,
    /// Layer configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub layers: Option<Layers>,
    /// Maximum context length in tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub context_length: Option<u64>,
    /// Vocabulary size
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vocabulary_size: Option<u64>,
}

/// Parameter counts
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Parameters {
    /// Total parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total: Option<u64>,
    /// Trainable parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trainable: Option<u64>,
    /// Embedding parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub embedding: Option<u64>,
}

/// Layer configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Layers {
    /// Number of layers
    #[serde(skip_serializing_if = "Option::is_none")]
    pub num_layers: Option<u32>,
    /// Hidden size
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hidden_size: Option<u32>,
    /// Number of attention heads
    #[serde(skip_serializing_if = "Option::is_none")]
    pub num_attention_heads: Option<u32>,
    /// Intermediate size
    #[serde(skip_serializing_if = "Option::is_none")]
    pub intermediate_size: Option<u32>,
}

/// Modality type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Modality {
    Text,
    Image,
    Audio,
    Video,
    Code,
    StructuredData,
}

/// Model capabilities
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Capabilities {
    /// Supported modalities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub modalities: Option<Vec<Modality>>,
    /// Supported languages (ISO 639-1 codes)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub languages: Option<Vec<String>>,
    /// Supported tasks
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tasks: Option<Vec<String>>,
    /// Feature flags
    #[serde(skip_serializing_if = "Option::is_none")]
    pub features: Option<Features>,
}

/// Feature flags
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Features {
    /// Supports function calling
    #[serde(skip_serializing_if = "Option::is_none")]
    pub function_calling: Option<bool>,
    /// Supports structured output
    #[serde(skip_serializing_if = "Option::is_none")]
    pub structured_output: Option<bool>,
    /// Supports vision/images
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vision: Option<bool>,
    /// Supports extended thinking
    #[serde(skip_serializing_if = "Option::is_none")]
    pub extended_thinking: Option<bool>,
    /// Supports streaming
    #[serde(skip_serializing_if = "Option::is_none")]
    pub streaming: Option<bool>,
}

/// Training method
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TrainingMethod {
    Pretrain,
    Finetune,
    Rlhf,
    Dpo,
    ConstitutionalAi,
    Other,
}

/// Training information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Training {
    /// Training method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<TrainingMethod>,
    /// Base model ID if fine-tuned
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base_model: Option<String>,
    /// Dataset information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dataset: Option<DatasetInfo>,
    /// Compute information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compute: Option<ComputeInfo>,
}

/// Dataset type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum DatasetType {
    Public,
    Proprietary,
    Mixed,
}

/// Dataset information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DatasetInfo {
    /// Dataset type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub dataset_type: Option<DatasetType>,
    /// Dataset size
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size: Option<String>,
    /// Knowledge cutoff date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cutoff_date: Option<String>,
}

/// Compute information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ComputeInfo {
    /// Hardware used
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hardware: Option<String>,
    /// Training duration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<String>,
    /// Training cost
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cost: Option<String>,
}

/// Model safety information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ModelSafety {
    /// Alignment method used
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alignment_method: Option<String>,
    /// Has safety training
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety_training: Option<bool>,
    /// Content policy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content_policy: Option<String>,
    /// Paths to evaluation files
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evaluations: Option<Vec<String>>,
}

/// Deployment information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Deployment {
    /// Available via inference API
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inference_api: Option<bool>,
    /// Available for on-premise deployment
    #[serde(skip_serializing_if = "Option::is_none")]
    pub on_premise: Option<bool>,
    /// Available quantization options
    #[serde(skip_serializing_if = "Option::is_none")]
    pub quantization_options: Option<Vec<String>>,
    /// Hardware requirements
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hardware_requirements: Option<HashMap<String, serde_json::Value>>,
}

/// License type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum LicenseType {
    OpenSource,
    Proprietary,
    ResearchOnly,
    Commercial,
}

/// Licensing information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Licensing {
    /// License type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub license_type: Option<LicenseType>,
    /// License name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub license_name: Option<String>,
    /// Allows commercial use
    #[serde(skip_serializing_if = "Option::is_none")]
    pub commercial_use: Option<bool>,
    /// Requires attribution
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attribution_required: Option<bool>,
}

// ============================================================================
// Agent Types
// ============================================================================

/// AI Agent definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Agent {
    /// WIA specification version
    pub wia_version: WiaVersion,
    /// Unique agent identifier
    pub agent_id: String,
    /// Agent information
    pub agent_info: AgentInfo,
    /// Model configuration
    pub model: AgentModel,
    /// System prompt
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system_prompt: Option<SystemPrompt>,
    /// Agent capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub capabilities: Option<AgentCapabilities>,
    /// Agent behavior
    #[serde(skip_serializing_if = "Option::is_none")]
    pub behavior: Option<AgentBehavior>,
    /// Agent constraints
    #[serde(skip_serializing_if = "Option::is_none")]
    pub constraints: Option<AgentConstraints>,
    /// Safety configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub safety: Option<AgentSafety>,
}

/// Agent type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum AgentType {
    Assistant,
    Researcher,
    Coder,
    Writer,
    Analyst,
    Orchestrator,
    Specialist,
    Custom,
}

/// Agent information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentInfo {
    /// Agent name
    pub name: String,
    /// Agent version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Agent description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Agent type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub agent_type: Option<AgentType>,
    /// Author
    #[serde(skip_serializing_if = "Option::is_none")]
    pub author: Option<String>,
    /// Creation timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
}

/// Model provider
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ModelProvider {
    Anthropic,
    Openai,
    Google,
    Meta,
    Mistral,
    Local,
    Other,
}

/// Agent model configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentModel {
    /// WIA model ID reference
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_id: Option<String>,
    /// Provider
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider: Option<ModelProvider>,
    /// Provider's model name
    pub model_name: String,
    /// Temperature (0.0 - 2.0)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub temperature: Option<f32>,
    /// Maximum tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_tokens: Option<u32>,
    /// Top-p sampling
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top_p: Option<f32>,
    /// Top-k sampling
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top_k: Option<u32>,
}

/// System prompt configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SystemPrompt {
    /// Inline content
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<String>,
    /// Template file path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub template: Option<String>,
    /// Template variables
    #[serde(skip_serializing_if = "Option::is_none")]
    pub variables: Option<HashMap<String, String>>,
}

/// Agent capabilities
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AgentCapabilities {
    /// Available tools
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tools: Option<Vec<ToolReference>>,
    /// MCP servers
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mcp_servers: Option<Vec<McpServer>>,
    /// Memory configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub memory: Option<MemoryConfig>,
}

/// Tool reference
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolReference {
    /// Tool name
    pub name: String,
    /// Tool description
    pub description: String,
    /// Path to schema file
    #[serde(skip_serializing_if = "Option::is_none")]
    pub schema: Option<String>,
    /// Inline parameter schema
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parameters: Option<serde_json::Value>,
}

/// MCP server configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct McpServer {
    /// Server name
    pub name: String,
    /// Command to run
    #[serde(skip_serializing_if = "Option::is_none")]
    pub command: Option<String>,
    /// Command arguments
    #[serde(skip_serializing_if = "Option::is_none")]
    pub args: Option<Vec<String>>,
    /// Environment variables
    #[serde(skip_serializing_if = "Option::is_none")]
    pub env: Option<HashMap<String, String>>,
}

/// Memory type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum MemoryType {
    None,
    Conversation,
    VectorStore,
    KnowledgeGraph,
}

/// Memory configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MemoryConfig {
    /// Memory type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub memory_type: Option<MemoryType>,
    /// Provider
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider: Option<String>,
    /// Additional configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<HashMap<String, serde_json::Value>>,
}

/// Planning strategy
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum PlanningStrategy {
    None,
    ChainOfThought,
    TreeOfThought,
    React,
    Reflexion,
}

/// Agent behavior
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AgentBehavior {
    /// Planning configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub planning: Option<PlanningConfig>,
    /// Reflection configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reflection: Option<ReflectionConfig>,
    /// Handoff configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub handoff: Option<HandoffConfig>,
}

/// Planning configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PlanningConfig {
    /// Planning enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    /// Planning strategy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub strategy: Option<PlanningStrategy>,
}

/// Reflection frequency
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ReflectionFrequency {
    Never,
    AfterEachTask,
    AfterEachStep,
    OnError,
}

/// Reflection configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ReflectionConfig {
    /// Reflection enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    /// Reflection frequency
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency: Option<ReflectionFrequency>,
}

/// Handoff configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct HandoffConfig {
    /// Handoff enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    /// Target agent IDs
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target_agents: Option<Vec<String>>,
}

/// Agent constraints
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AgentConstraints {
    /// Maximum iterations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_iterations: Option<u32>,
    /// Timeout in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
    /// Rate limits
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_limits: Option<RateLimits>,
}

/// Rate limits
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RateLimits {
    /// Requests per minute
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requests_per_minute: Option<u32>,
    /// Tokens per minute
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tokens_per_minute: Option<u32>,
}

/// PII handling mode
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum PiiHandling {
    Allow,
    Warn,
    Redact,
    Block,
}

/// Agent safety configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AgentSafety {
    /// Content filters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content_filters: Option<Vec<String>>,
    /// PII handling mode
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pii_handling: Option<PiiHandling>,
    /// Audit logging enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub audit_logging: Option<bool>,
    /// Human-in-loop configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub human_in_loop: Option<HumanInLoop>,
}

/// Human-in-loop configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct HumanInLoop {
    /// Enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enabled: Option<bool>,
    /// Actions requiring approval
    #[serde(skip_serializing_if = "Option::is_none")]
    pub actions_requiring_approval: Option<Vec<String>>,
}

// ============================================================================
// Experiment Types
// ============================================================================

/// Experiment record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Experiment {
    /// WIA specification version
    pub wia_version: WiaVersion,
    /// Unique experiment identifier
    pub experiment_id: String,
    /// Experiment information
    pub experiment_info: ExperimentInfo,
    /// Base model
    pub base_model: BaseModel,
    /// Dataset configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dataset: Option<ExperimentDataset>,
    /// Hyperparameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hyperparameters: Option<Hyperparameters>,
    /// Training configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub training_config: Option<TrainingConfig>,
    /// Compute resources
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compute: Option<ExperimentCompute>,
    /// Results
    #[serde(skip_serializing_if = "Option::is_none")]
    pub results: Option<ExperimentResults>,
    /// Artifacts
    #[serde(skip_serializing_if = "Option::is_none")]
    pub artifacts: Option<ExperimentArtifacts>,
    /// Reproducibility information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reproducibility: Option<Reproducibility>,
}

/// Experiment type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ExperimentType {
    Pretraining,
    Finetuning,
    Rlhf,
    Dpo,
    Evaluation,
    Ablation,
    Hpo,
    Other,
}

/// Experiment status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ExperimentStatus {
    Pending,
    Running,
    Completed,
    Failed,
    Cancelled,
}

/// Experiment information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExperimentInfo {
    /// Experiment name
    pub name: String,
    /// Experiment type
    #[serde(rename = "type")]
    pub experiment_type: ExperimentType,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Status
    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<ExperimentStatus>,
    /// Creation timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
    /// Completion timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub completed_at: Option<DateTime<Utc>>,
    /// Tags
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tags: Option<Vec<String>>,
}

/// Base model reference
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BaseModel {
    /// Model ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_id: Option<String>,
    /// Provider
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider: Option<String>,
    /// Model name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model_name: Option<String>,
}

/// Experiment dataset
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ExperimentDataset {
    /// Dataset name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Dataset version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Dataset splits
    #[serde(skip_serializing_if = "Option::is_none")]
    pub splits: Option<HashMap<String, DataSplit>>,
}

/// Data split format
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum DataFormat {
    Jsonl,
    Parquet,
    Csv,
    Arrow,
    Tfrecord,
}

/// Data split
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DataSplit {
    /// File path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// Number of examples
    #[serde(skip_serializing_if = "Option::is_none")]
    pub num_examples: Option<u64>,
    /// Data format
    #[serde(skip_serializing_if = "Option::is_none")]
    pub format: Option<DataFormat>,
}

/// Hyperparameters
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Hyperparameters {
    /// Learning rate
    #[serde(skip_serializing_if = "Option::is_none")]
    pub learning_rate: Option<f64>,
    /// Batch size
    #[serde(skip_serializing_if = "Option::is_none")]
    pub batch_size: Option<u32>,
    /// Number of epochs
    #[serde(skip_serializing_if = "Option::is_none")]
    pub epochs: Option<u32>,
    /// Warmup steps
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warmup_steps: Option<u32>,
    /// Weight decay
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weight_decay: Option<f64>,
    /// Maximum sequence length
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_seq_length: Option<u32>,
    /// Gradient accumulation steps
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gradient_accumulation_steps: Option<u32>,
    /// Additional parameters
    #[serde(flatten)]
    pub extra: HashMap<String, serde_json::Value>,
}

/// Optimizer type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Optimizer {
    Sgd,
    Adam,
    Adamw,
    Adafactor,
    Lion,
    Other,
}

/// Scheduler type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Scheduler {
    Constant,
    Linear,
    Cosine,
    Polynomial,
    Other,
}

/// Precision type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Precision {
    Fp32,
    Fp16,
    Bf16,
    Int8,
    Int4,
}

/// Distributed strategy
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum DistributedStrategy {
    None,
    Ddp,
    Fsdp,
    Deepspeed,
    Megatron,
}

/// Training configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct TrainingConfig {
    /// Optimizer
    #[serde(skip_serializing_if = "Option::is_none")]
    pub optimizer: Option<Optimizer>,
    /// Scheduler
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scheduler: Option<Scheduler>,
    /// Precision
    #[serde(skip_serializing_if = "Option::is_none")]
    pub precision: Option<Precision>,
    /// Distributed training
    #[serde(skip_serializing_if = "Option::is_none")]
    pub distributed: Option<DistributedConfig>,
}

/// Distributed training configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DistributedConfig {
    /// Strategy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub strategy: Option<DistributedStrategy>,
    /// Number of GPUs
    #[serde(skip_serializing_if = "Option::is_none")]
    pub num_gpus: Option<u32>,
    /// Number of nodes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub num_nodes: Option<u32>,
}

/// Experiment compute resources
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ExperimentCompute {
    /// Hardware configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hardware: Option<HardwareConfig>,
    /// Duration in hours
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration_hours: Option<f64>,
    /// Cost in USD
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cost_usd: Option<f64>,
}

/// Hardware configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct HardwareConfig {
    /// GPU type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gpu_type: Option<String>,
    /// Number of GPUs
    #[serde(skip_serializing_if = "Option::is_none")]
    pub num_gpus: Option<u32>,
    /// GPU memory in GB
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gpu_memory_gb: Option<f64>,
    /// CPU cores
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cpu_cores: Option<u32>,
    /// RAM in GB
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ram_gb: Option<f64>,
}

/// Experiment results
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ExperimentResults {
    /// Final metrics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_metrics: Option<HashMap<String, f64>>,
    /// Checkpoints
    #[serde(skip_serializing_if = "Option::is_none")]
    pub checkpoints: Option<Vec<Checkpoint>>,
}

/// Checkpoint
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Checkpoint {
    /// Step number
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step: Option<u64>,
    /// Epoch number
    #[serde(skip_serializing_if = "Option::is_none")]
    pub epoch: Option<f64>,
    /// Path to checkpoint
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// Metrics at this checkpoint
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<HashMap<String, f64>>,
}

/// Experiment artifacts
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ExperimentArtifacts {
    /// Model path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    /// Logs path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub logs: Option<String>,
    /// TensorBoard path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tensorboard: Option<String>,
    /// Config path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<String>,
}

/// Reproducibility information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Reproducibility {
    /// Random seed
    #[serde(skip_serializing_if = "Option::is_none")]
    pub random_seed: Option<u64>,
    /// Framework
    #[serde(skip_serializing_if = "Option::is_none")]
    pub framework: Option<String>,
    /// Framework version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub framework_version: Option<String>,
    /// Dependencies file path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dependencies: Option<String>,
    /// Git commit hash
    #[serde(skip_serializing_if = "Option::is_none")]
    pub git_commit: Option<String>,
    /// Git repository URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub git_repo: Option<String>,
}

// ============================================================================
// Evaluation Types
// ============================================================================

/// Evaluation results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Evaluation {
    /// WIA specification version
    pub wia_version: WiaVersion,
    /// Unique evaluation identifier
    pub evaluation_id: String,
    /// Evaluation information
    pub evaluation_info: EvaluationInfo,
    /// Benchmark results
    #[serde(skip_serializing_if = "Option::is_none")]
    pub benchmarks: Option<Vec<BenchmarkResult>>,
    /// Custom evaluations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub custom_evaluations: Option<Vec<CustomEvaluation>>,
    /// Summary
    #[serde(skip_serializing_if = "Option::is_none")]
    pub summary: Option<EvaluationSummary>,
    /// Metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<EvaluationMetadata>,
}

/// Evaluation type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EvaluationType {
    Safety,
    Capability,
    Alignment,
    Robustness,
    Bias,
    Performance,
    Comprehensive,
}

/// Evaluation information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EvaluationInfo {
    /// Evaluation name
    pub name: String,
    /// Model ID being evaluated
    pub model_id: String,
    /// Evaluation type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub evaluation_type: Option<EvaluationType>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Creation timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
    /// Evaluator
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evaluator: Option<String>,
}

/// Benchmark result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkResult {
    /// Benchmark name
    pub name: String,
    /// Version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Overall score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub overall_score: Option<f64>,
    /// Category scores
    #[serde(skip_serializing_if = "Option::is_none")]
    pub categories: Option<HashMap<String, BenchmarkCategory>>,
    /// Scores by subject
    #[serde(skip_serializing_if = "Option::is_none")]
    pub by_subject: Option<HashMap<String, f64>>,
    /// Pass@1 score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pass_at_1: Option<f64>,
    /// Pass@10 score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pass_at_10: Option<f64>,
    /// Accuracy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub accuracy: Option<f64>,
}

/// Benchmark category
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BenchmarkCategory {
    /// Score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub score: Option<f64>,
    /// Detailed scores
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<HashMap<String, f64>>,
}

/// Custom evaluation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomEvaluation {
    /// Name
    pub name: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Dataset used
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dataset: Option<String>,
    /// Metrics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<HashMap<String, f64>>,
    /// Methodology
    #[serde(skip_serializing_if = "Option::is_none")]
    pub methodology: Option<String>,
}

/// Evaluation summary
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct EvaluationSummary {
    /// Overall score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub overall_score: Option<f64>,
    /// Grade
    #[serde(skip_serializing_if = "Option::is_none")]
    pub grade: Option<String>,
    /// Strengths
    #[serde(skip_serializing_if = "Option::is_none")]
    pub strengths: Option<Vec<String>>,
    /// Weaknesses
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weaknesses: Option<Vec<String>>,
    /// Recommendations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recommendations: Option<Vec<String>>,
}

/// Evaluation metadata
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct EvaluationMetadata {
    /// Evaluation framework
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evaluation_framework: Option<String>,
    /// Framework version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub framework_version: Option<String>,
    /// Compute used
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compute_used: Option<String>,
    /// Duration in hours
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration_hours: Option<f64>,
}

// ============================================================================
// Safety Assessment Types
// ============================================================================

/// Safety assessment
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyAssessment {
    /// WIA specification version
    pub wia_version: WiaVersion,
    /// Assessment ID
    pub assessment_id: String,
    /// Assessment information
    pub assessment_info: AssessmentInfo,
    /// Risk categories
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_categories: Option<RiskCategories>,
    /// Red team findings
    #[serde(skip_serializing_if = "Option::is_none")]
    pub red_team_findings: Option<RedTeamFindings>,
    /// Compliance information
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compliance: Option<ComplianceInfo>,
    /// Incident history
    #[serde(skip_serializing_if = "Option::is_none")]
    pub incident_history: Option<IncidentHistory>,
    /// Overall rating
    #[serde(skip_serializing_if = "Option::is_none")]
    pub overall_rating: Option<OverallRating>,
}

/// Assessment information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AssessmentInfo {
    /// Assessment name
    pub name: String,
    /// Model ID
    pub model_id: String,
    /// Assessor
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assessor: Option<String>,
    /// Methodology
    #[serde(skip_serializing_if = "Option::is_none")]
    pub methodology: Option<String>,
    /// Creation timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
}

/// Risk level
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum RiskLevel {
    Critical,
    High,
    Medium,
    Low,
    None,
}

/// Risk categories
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RiskCategories {
    /// Dangerous capabilities
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dangerous_capabilities: Option<RiskCategory>,
    /// Misuse potential
    #[serde(skip_serializing_if = "Option::is_none")]
    pub misuse_potential: Option<RiskCategory>,
    /// Alignment
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alignment: Option<AlignmentCategory>,
    /// Bias and fairness
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bias_fairness: Option<RiskCategory>,
    /// Privacy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub privacy: Option<RiskCategory>,
}

/// Risk category
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RiskCategory {
    /// Risk level
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_level: Option<RiskLevel>,
    /// Tests conducted
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tests_conducted: Option<Vec<SafetyTest>>,
    /// Concerns
    #[serde(skip_serializing_if = "Option::is_none")]
    pub concerns: Option<Vec<String>>,
    /// Mitigations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mitigations: Option<Vec<String>>,
}

/// Test result
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TestResult {
    Pass,
    Fail,
    Partial,
    NotApplicable,
}

/// Safety test
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SafetyTest {
    /// Test name
    pub name: String,
    /// Result
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<TestResult>,
    /// Details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<String>,
}

/// Alignment category
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AlignmentCategory {
    /// Risk level
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_level: Option<RiskLevel>,
    /// Evaluation scores
    #[serde(skip_serializing_if = "Option::is_none")]
    pub evaluation: Option<AlignmentEvaluation>,
}

/// Alignment evaluation scores
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct AlignmentEvaluation {
    /// Value alignment score
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value_alignment_score: Option<f64>,
    /// Goal stability
    #[serde(skip_serializing_if = "Option::is_none")]
    pub goal_stability: Option<f64>,
    /// Instruction following
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instruction_following: Option<f64>,
    /// Honesty
    #[serde(skip_serializing_if = "Option::is_none")]
    pub honesty: Option<f64>,
    /// Harmlessness
    #[serde(skip_serializing_if = "Option::is_none")]
    pub harmlessness: Option<f64>,
}

/// Remediation status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum RemediationStatus {
    NotStarted,
    InProgress,
    Completed,
    Ongoing,
}

/// Red team findings
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RedTeamFindings {
    /// Total attempts
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_attempts: Option<u64>,
    /// Successful jailbreaks
    #[serde(skip_serializing_if = "Option::is_none")]
    pub successful_jailbreaks: Option<u64>,
    /// Jailbreak rate
    #[serde(skip_serializing_if = "Option::is_none")]
    pub jailbreak_rate: Option<f64>,
    /// Categories
    #[serde(skip_serializing_if = "Option::is_none")]
    pub categories: Option<HashMap<String, u64>>,
    /// Remediation status
    #[serde(skip_serializing_if = "Option::is_none")]
    pub remediation_status: Option<RemediationStatus>,
}

/// Compliance status
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ComplianceStatus {
    Compliant,
    NonCompliant,
    InProgress,
    NotApplicable,
}

/// Compliance information
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ComplianceInfo {
    /// Frameworks
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frameworks: Option<Vec<ComplianceFramework>>,
}

/// Compliance framework
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComplianceFramework {
    /// Framework name
    pub name: String,
    /// Status
    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<ComplianceStatus>,
    /// Risk classification
    #[serde(skip_serializing_if = "Option::is_none")]
    pub risk_classification: Option<String>,
    /// Requirements met
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requirements_met: Option<u32>,
    /// Total requirements
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requirements_total: Option<u32>,
    /// Certification date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub certification_date: Option<NaiveDate>,
    /// Categories addressed
    #[serde(skip_serializing_if = "Option::is_none")]
    pub categories_addressed: Option<Vec<String>>,
}

/// Incident severity
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum IncidentSeverity {
    Critical,
    High,
    Medium,
    Low,
}

/// Incident history
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct IncidentHistory {
    /// Total incidents
    #[serde(skip_serializing_if = "Option::is_none")]
    pub total_incidents: Option<u32>,
    /// Severity breakdown
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity_breakdown: Option<SeverityBreakdown>,
    /// Recent incidents
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recent_incidents: Option<Vec<Incident>>,
}

/// Severity breakdown
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SeverityBreakdown {
    /// Critical
    #[serde(skip_serializing_if = "Option::is_none")]
    pub critical: Option<u32>,
    /// High
    #[serde(skip_serializing_if = "Option::is_none")]
    pub high: Option<u32>,
    /// Medium
    #[serde(skip_serializing_if = "Option::is_none")]
    pub medium: Option<u32>,
    /// Low
    #[serde(skip_serializing_if = "Option::is_none")]
    pub low: Option<u32>,
}

/// Incident
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Incident {
    /// Date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub date: Option<DateTime<Utc>>,
    /// Severity
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity: Option<IncidentSeverity>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Resolution
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resolution: Option<String>,
    /// Root cause
    #[serde(skip_serializing_if = "Option::is_none")]
    pub root_cause: Option<String>,
}

/// Overall rating
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct OverallRating {
    /// Score (e.g., "B+")
    #[serde(skip_serializing_if = "Option::is_none")]
    pub score: Option<String>,
    /// Numeric score (0-100)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub numeric_score: Option<f64>,
    /// Summary
    #[serde(skip_serializing_if = "Option::is_none")]
    pub summary: Option<String>,
    /// Next review date
    #[serde(skip_serializing_if = "Option::is_none")]
    pub next_review_date: Option<NaiveDate>,
}

// ============================================================================
// Prompt Types
// ============================================================================

/// Prompt template
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Prompt {
    /// WIA specification version
    pub wia_version: WiaVersion,
    /// Unique prompt identifier
    pub prompt_id: String,
    /// Prompt information
    pub prompt_info: PromptInfo,
    /// Template content
    pub template: PromptTemplate,
    /// Variables
    #[serde(skip_serializing_if = "Option::is_none")]
    pub variables: Option<HashMap<String, PromptVariable>>,
    /// Examples
    #[serde(skip_serializing_if = "Option::is_none")]
    pub examples: Option<Vec<PromptExample>>,
    /// Metadata
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<PromptMetadata>,
    /// Validation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub validation: Option<PromptValidation>,
}

/// Prompt category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum PromptCategory {
    System,
    User,
    Assistant,
    FewShot,
    ChainOfThought,
    ToolUse,
    Other,
}

/// Prompt information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PromptInfo {
    /// Name
    pub name: String,
    /// Version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Author
    #[serde(skip_serializing_if = "Option::is_none")]
    pub author: Option<String>,
    /// Tags
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tags: Option<Vec<String>>,
    /// Category
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<PromptCategory>,
}

/// Prompt template
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PromptTemplate {
    /// System prompt
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system: Option<String>,
    /// User message template
    #[serde(skip_serializing_if = "Option::is_none")]
    pub user: Option<String>,
    /// Assistant response template
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assistant: Option<String>,
    /// Assistant prefill
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assistant_prefill: Option<String>,
}

/// Variable type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum VariableType {
    String,
    Number,
    Boolean,
    Array,
    Object,
}

/// Prompt variable
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PromptVariable {
    /// Type
    #[serde(rename = "type")]
    pub variable_type: VariableType,
    /// Required
    #[serde(skip_serializing_if = "Option::is_none")]
    pub required: Option<bool>,
    /// Default value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub default: Option<serde_json::Value>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Examples
    #[serde(skip_serializing_if = "Option::is_none")]
    pub examples: Option<Vec<serde_json::Value>>,
    /// Allowed values
    #[serde(rename = "enum", skip_serializing_if = "Option::is_none")]
    pub allowed_values: Option<Vec<serde_json::Value>>,
}

/// Prompt example
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PromptExample {
    /// Name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Variables
    #[serde(skip_serializing_if = "Option::is_none")]
    pub variables: Option<HashMap<String, serde_json::Value>>,
    /// Expected output
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expected_output: Option<String>,
    /// Expected topics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub expected_topics: Option<Vec<String>>,
}

/// Prompt metadata
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PromptMetadata {
    /// Optimal temperature
    #[serde(skip_serializing_if = "Option::is_none")]
    pub optimal_temperature: Option<f32>,
    /// Optimal top-p
    #[serde(skip_serializing_if = "Option::is_none")]
    pub optimal_top_p: Option<f32>,
    /// Max tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_tokens: Option<u32>,
    /// Stop sequences
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stop_sequences: Option<Vec<String>>,
    /// Recommended models
    #[serde(skip_serializing_if = "Option::is_none")]
    pub recommended_models: Option<Vec<String>>,
}

/// Output format
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum OutputFormat {
    Text,
    Json,
    Markdown,
    Code,
    Structured,
}

/// Prompt validation
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct PromptValidation {
    /// Expected output format
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output_format: Option<OutputFormat>,
    /// Output schema
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output_schema: Option<serde_json::Value>,
    /// Required elements
    #[serde(skip_serializing_if = "Option::is_none")]
    pub required_elements: Option<Vec<String>>,
}

// ============================================================================
// Swarm Types
// ============================================================================

/// Multi-agent swarm configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Swarm {
    /// WIA specification version
    pub wia_version: WiaVersion,
    /// Unique swarm identifier
    pub swarm_id: String,
    /// Swarm information
    pub swarm_info: SwarmInfo,
    /// Agents in the swarm
    pub agents: Vec<AgentReference>,
    /// Topology
    pub topology: SwarmTopology,
    /// Communication configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub communication: Option<SwarmCommunication>,
    /// Workflow
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workflow: Option<SwarmWorkflow>,
    /// Constraints
    #[serde(skip_serializing_if = "Option::is_none")]
    pub constraints: Option<SwarmConstraints>,
    /// Error handling
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_handling: Option<ErrorHandling>,
    /// Monitoring
    #[serde(skip_serializing_if = "Option::is_none")]
    pub monitoring: Option<Monitoring>,
}

/// Swarm architecture type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SwarmArchitecture {
    Hierarchical,
    Flat,
    Swarm,
    Pipeline,
    Mesh,
    Hybrid,
}

/// Swarm information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwarmInfo {
    /// Swarm name
    pub name: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Architecture
    #[serde(skip_serializing_if = "Option::is_none")]
    pub architecture: Option<SwarmArchitecture>,
    /// Author
    #[serde(skip_serializing_if = "Option::is_none")]
    pub author: Option<String>,
    /// Creation timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<DateTime<Utc>>,
}

/// Agent reference in swarm
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentReference {
    /// Agent ID
    pub agent_id: String,
    /// Role
    pub role: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Config file path
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<String>,
    /// Number of instances
    #[serde(skip_serializing_if = "Option::is_none")]
    pub instances: Option<u32>,
}

/// Topology type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TopologyType {
    Hierarchical,
    Flat,
    Ring,
    Star,
    Mesh,
    Custom,
}

/// Swarm topology
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwarmTopology {
    /// Topology type
    #[serde(rename = "type")]
    pub topology_type: TopologyType,
    /// Root agent ID (for hierarchical)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub root: Option<String>,
    /// Connections
    #[serde(skip_serializing_if = "Option::is_none")]
    pub connections: Option<Vec<Connection>>,
}

/// Connection type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionType {
    Delegation,
    Handoff,
    Broadcast,
    Request,
    Subscribe,
}

/// Connection between agents
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Connection {
    /// Source agent
    pub from: String,
    /// Target agent
    pub to: String,
    /// Connection type
    #[serde(rename = "type")]
    pub connection_type: ConnectionType,
    /// Bidirectional
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bidirectional: Option<bool>,
    /// Condition
    #[serde(skip_serializing_if = "Option::is_none")]
    pub condition: Option<String>,
}

/// Communication protocol
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum CommunicationProtocol {
    A2a,
    Mcp,
    Custom,
    Direct,
}

/// Message format
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum MessageFormat {
    Json,
    Protobuf,
    Msgpack,
}

/// Swarm communication
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SwarmCommunication {
    /// Protocol
    #[serde(skip_serializing_if = "Option::is_none")]
    pub protocol: Option<CommunicationProtocol>,
    /// Message format
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message_format: Option<MessageFormat>,
    /// Shared memory
    #[serde(skip_serializing_if = "Option::is_none")]
    pub shared_memory: Option<SharedMemory>,
    /// Broadcast enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub broadcast: Option<bool>,
}

/// Shared memory type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum SharedMemoryType {
    None,
    InMemory,
    Redis,
    VectorStore,
    Database,
}

/// Shared memory configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SharedMemory {
    /// Type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub memory_type: Option<SharedMemoryType>,
    /// Provider
    #[serde(skip_serializing_if = "Option::is_none")]
    pub provider: Option<String>,
    /// Additional configuration
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<HashMap<String, serde_json::Value>>,
}

/// Workflow type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum WorkflowType {
    Sequential,
    Parallel,
    Conditional,
    Loop,
    Dag,
    Reactive,
}

/// Swarm workflow
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SwarmWorkflow {
    /// Workflow type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub workflow_type: Option<WorkflowType>,
    /// Steps
    #[serde(skip_serializing_if = "Option::is_none")]
    pub steps: Option<Vec<WorkflowStep>>,
    /// Entry point
    #[serde(skip_serializing_if = "Option::is_none")]
    pub entry_point: Option<String>,
    /// Exit conditions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub exit_conditions: Option<Vec<String>>,
}

/// Workflow step
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkflowStep {
    /// Step number
    pub step: u32,
    /// Step name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Agent ID
    pub agent: String,
    /// Action
    pub action: String,
    /// Inputs
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inputs: Option<HashMap<String, serde_json::Value>>,
    /// Outputs
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outputs: Option<Vec<String>>,
    /// Condition
    #[serde(skip_serializing_if = "Option::is_none")]
    pub condition: Option<String>,
    /// Loop until
    #[serde(skip_serializing_if = "Option::is_none")]
    pub loop_until: Option<String>,
    /// On success
    #[serde(skip_serializing_if = "Option::is_none")]
    pub on_success: Option<String>,
    /// On failure
    #[serde(skip_serializing_if = "Option::is_none")]
    pub on_failure: Option<String>,
    /// Timeout
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
}

/// Swarm constraints
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SwarmConstraints {
    /// Max total iterations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_total_iterations: Option<u32>,
    /// Max concurrent agents
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_concurrent_agents: Option<u32>,
    /// Timeout in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
    /// Budget in USD
    #[serde(skip_serializing_if = "Option::is_none")]
    pub budget_usd: Option<f64>,
    /// Max total tokens
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_tokens_total: Option<u64>,
}

/// Backoff strategy
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum BackoffStrategy {
    None,
    Linear,
    Exponential,
}

/// Failure action
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum FailureAction {
    Abort,
    Skip,
    Fallback,
    HumanEscalation,
}

/// Error handling
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ErrorHandling {
    /// Retry policy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub retry_policy: Option<RetryPolicy>,
    /// Fallback agent
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fallback_agent: Option<String>,
    /// On failure action
    #[serde(skip_serializing_if = "Option::is_none")]
    pub on_failure: Option<FailureAction>,
}

/// Retry policy
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct RetryPolicy {
    /// Max retries
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_retries: Option<u32>,
    /// Backoff strategy
    #[serde(skip_serializing_if = "Option::is_none")]
    pub backoff_strategy: Option<BackoffStrategy>,
}

/// Log level
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
}

/// Monitoring configuration
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Monitoring {
    /// Logging level
    #[serde(skip_serializing_if = "Option::is_none")]
    pub logging_level: Option<LogLevel>,
    /// Metrics enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics_enabled: Option<bool>,
    /// Tracing enabled
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tracing_enabled: Option<bool>,
    /// Dashboard URL
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dashboard_url: Option<String>,
}

// ============================================================================
// Tool Types
// ============================================================================

/// Tool definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tool {
    /// Tool name
    pub name: String,
    /// Description
    pub description: String,
    /// Version
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
    /// Category
    #[serde(skip_serializing_if = "Option::is_none")]
    pub category: Option<ToolCategory>,
    /// Parameters
    pub parameters: ToolParameters,
    /// Returns
    #[serde(skip_serializing_if = "Option::is_none")]
    pub returns: Option<serde_json::Value>,
    /// Errors
    #[serde(skip_serializing_if = "Option::is_none")]
    pub errors: Option<Vec<ToolError>>,
    /// Examples
    #[serde(skip_serializing_if = "Option::is_none")]
    pub examples: Option<Vec<ToolExample>>,
    /// Constraints
    #[serde(skip_serializing_if = "Option::is_none")]
    pub constraints: Option<ToolConstraints>,
    /// Permissions
    #[serde(skip_serializing_if = "Option::is_none")]
    pub permissions: Option<Vec<Permission>>,
    /// Implementation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub implementation: Option<ToolImplementation>,
}

/// Tool category
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ToolCategory {
    Web,
    File,
    Code,
    Data,
    Communication,
    Search,
    Analysis,
    Generation,
    Utility,
    Custom,
}

/// Tool parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolParameters {
    /// Type (always "object")
    #[serde(rename = "type")]
    pub param_type: String,
    /// Required parameters
    #[serde(skip_serializing_if = "Option::is_none")]
    pub required: Option<Vec<String>>,
    /// Properties
    pub properties: HashMap<String, ToolParameter>,
}

/// Tool parameter
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolParameter {
    /// Type
    #[serde(rename = "type")]
    pub param_type: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Default value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub default: Option<serde_json::Value>,
    /// Allowed values
    #[serde(rename = "enum", skip_serializing_if = "Option::is_none")]
    pub allowed_values: Option<Vec<serde_json::Value>>,
    /// Minimum
    #[serde(skip_serializing_if = "Option::is_none")]
    pub minimum: Option<f64>,
    /// Maximum
    #[serde(skip_serializing_if = "Option::is_none")]
    pub maximum: Option<f64>,
    /// Min length
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_length: Option<u32>,
    /// Max length
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_length: Option<u32>,
    /// Pattern
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pattern: Option<String>,
    /// Format
    #[serde(skip_serializing_if = "Option::is_none")]
    pub format: Option<String>,
}

/// Tool error
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolError {
    /// Error code
    pub code: String,
    /// Message
    pub message: String,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
}

/// Tool example
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ToolExample {
    /// Name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Input
    #[serde(skip_serializing_if = "Option::is_none")]
    pub input: Option<HashMap<String, serde_json::Value>>,
    /// Output
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output: Option<serde_json::Value>,
}

/// Tool constraints
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ToolConstraints {
    /// Requires confirmation
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires_confirmation: Option<bool>,
    /// Rate limit
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_limit: Option<ToolRateLimit>,
    /// Timeout in seconds
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout_seconds: Option<u32>,
    /// Max retries
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_retries: Option<u32>,
}

/// Tool rate limit
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ToolRateLimit {
    /// Calls per minute
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calls_per_minute: Option<u32>,
    /// Calls per hour
    #[serde(skip_serializing_if = "Option::is_none")]
    pub calls_per_hour: Option<u32>,
}

/// Permission
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Permission {
    ReadFile,
    WriteFile,
    ExecuteCode,
    NetworkAccess,
    SystemAccess,
    DatabaseAccess,
    ApiAccess,
}

/// Implementation type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ImplementationType {
    Builtin,
    Mcp,
    Http,
    Function,
    Script,
}

/// Tool implementation
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ToolImplementation {
    /// Type
    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    pub impl_type: Option<ImplementationType>,
    /// Endpoint
    #[serde(skip_serializing_if = "Option::is_none")]
    pub endpoint: Option<String>,
    /// HTTP method
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<String>,
    /// MCP server
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mcp_server: Option<String>,
    /// Function name
    #[serde(skip_serializing_if = "Option::is_none")]
    pub function: Option<String>,
}
