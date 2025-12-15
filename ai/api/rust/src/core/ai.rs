//! WIA AI Core Operations
//!
//! This module provides core AI operations including model management,
//! agent execution, and swarm orchestration.

use crate::error::{Result, WiaAiError};
use crate::types::*;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

// ============================================================================
// Constants
// ============================================================================

/// Current WIA AI specification version
pub const WIA_VERSION: &str = "1.0.0";

// ============================================================================
// Model Registry
// ============================================================================

/// Model registry for managing AI models
#[derive(Debug, Default)]
pub struct ModelRegistry {
    models: Arc<RwLock<HashMap<String, Model>>>,
}

impl ModelRegistry {
    /// Create a new model registry
    pub fn new() -> Self {
        Self {
            models: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Register a model
    pub async fn register(&self, model: Model) -> Result<()> {
        let mut models = self.models.write().await;
        models.insert(model.model_id.clone(), model);
        Ok(())
    }

    /// Get a model by ID
    pub async fn get(&self, model_id: &str) -> Result<Model> {
        let models = self.models.read().await;
        models
            .get(model_id)
            .cloned()
            .ok_or_else(|| WiaAiError::ModelNotFound(model_id.to_string()))
    }

    /// List all models
    pub async fn list(&self) -> Vec<Model> {
        let models = self.models.read().await;
        models.values().cloned().collect()
    }

    /// Remove a model
    pub async fn remove(&self, model_id: &str) -> Result<Model> {
        let mut models = self.models.write().await;
        models
            .remove(model_id)
            .ok_or_else(|| WiaAiError::ModelNotFound(model_id.to_string()))
    }
}

// ============================================================================
// Agent Registry
// ============================================================================

/// Agent registry for managing AI agents
#[derive(Debug, Default)]
pub struct AgentRegistry {
    agents: Arc<RwLock<HashMap<String, Agent>>>,
}

impl AgentRegistry {
    /// Create a new agent registry
    pub fn new() -> Self {
        Self {
            agents: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Register an agent
    pub async fn register(&self, agent: Agent) -> Result<()> {
        let mut agents = self.agents.write().await;
        agents.insert(agent.agent_id.clone(), agent);
        Ok(())
    }

    /// Get an agent by ID
    pub async fn get(&self, agent_id: &str) -> Result<Agent> {
        let agents = self.agents.read().await;
        agents
            .get(agent_id)
            .cloned()
            .ok_or_else(|| WiaAiError::AgentNotFound(agent_id.to_string()))
    }

    /// List all agents
    pub async fn list(&self) -> Vec<Agent> {
        let agents = self.agents.read().await;
        agents.values().cloned().collect()
    }

    /// Remove an agent
    pub async fn remove(&self, agent_id: &str) -> Result<Agent> {
        let mut agents = self.agents.write().await;
        agents
            .remove(agent_id)
            .ok_or_else(|| WiaAiError::AgentNotFound(agent_id.to_string()))
    }
}

// ============================================================================
// Tool Registry
// ============================================================================

/// Tool registry for managing AI tools
#[derive(Debug, Default)]
pub struct ToolRegistry {
    tools: Arc<RwLock<HashMap<String, Tool>>>,
}

impl ToolRegistry {
    /// Create a new tool registry
    pub fn new() -> Self {
        Self {
            tools: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Register a tool
    pub async fn register(&self, tool: Tool) -> Result<()> {
        let mut tools = self.tools.write().await;
        tools.insert(tool.name.clone(), tool);
        Ok(())
    }

    /// Get a tool by name
    pub async fn get(&self, name: &str) -> Result<Tool> {
        let tools = self.tools.read().await;
        tools
            .get(name)
            .cloned()
            .ok_or_else(|| WiaAiError::ToolNotFound(name.to_string()))
    }

    /// List all tools
    pub async fn list(&self) -> Vec<Tool> {
        let tools = self.tools.read().await;
        tools.values().cloned().collect()
    }

    /// Remove a tool
    pub async fn remove(&self, name: &str) -> Result<Tool> {
        let mut tools = self.tools.write().await;
        tools
            .remove(name)
            .ok_or_else(|| WiaAiError::ToolNotFound(name.to_string()))
    }
}

// ============================================================================
// Agent Executor Trait
// ============================================================================

/// Message role
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum MessageRole {
    System,
    User,
    Assistant,
    Tool,
}

/// Chat message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Message {
    /// Role
    pub role: MessageRole,
    /// Content
    pub content: String,
    /// Name (for tool messages)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Tool call ID (for tool messages)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_call_id: Option<String>,
}

impl Message {
    /// Create a system message
    pub fn system(content: impl Into<String>) -> Self {
        Self {
            role: MessageRole::System,
            content: content.into(),
            name: None,
            tool_call_id: None,
        }
    }

    /// Create a user message
    pub fn user(content: impl Into<String>) -> Self {
        Self {
            role: MessageRole::User,
            content: content.into(),
            name: None,
            tool_call_id: None,
        }
    }

    /// Create an assistant message
    pub fn assistant(content: impl Into<String>) -> Self {
        Self {
            role: MessageRole::Assistant,
            content: content.into(),
            name: None,
            tool_call_id: None,
        }
    }

    /// Create a tool message
    pub fn tool(content: impl Into<String>, tool_call_id: impl Into<String>) -> Self {
        Self {
            role: MessageRole::Tool,
            content: content.into(),
            name: None,
            tool_call_id: Some(tool_call_id.into()),
        }
    }
}

/// Tool call request
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolCall {
    /// Tool call ID
    pub id: String,
    /// Tool name
    pub name: String,
    /// Arguments
    pub arguments: serde_json::Value,
}

/// Agent response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AgentResponse {
    /// Response content
    pub content: String,
    /// Tool calls (if any)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tool_calls: Option<Vec<ToolCall>>,
    /// Usage statistics
    #[serde(skip_serializing_if = "Option::is_none")]
    pub usage: Option<UsageStats>,
    /// Stop reason
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stop_reason: Option<String>,
}

/// Usage statistics
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct UsageStats {
    /// Input tokens
    pub input_tokens: u64,
    /// Output tokens
    pub output_tokens: u64,
    /// Total tokens
    pub total_tokens: u64,
}

/// Agent executor trait
#[async_trait]
pub trait AgentExecutor: Send + Sync {
    /// Execute the agent with messages
    async fn execute(&self, messages: Vec<Message>) -> Result<AgentResponse>;

    /// Execute the agent with a single user message
    async fn chat(&self, message: &str) -> Result<AgentResponse> {
        self.execute(vec![Message::user(message)]).await
    }
}

// ============================================================================
// Tool Executor Trait
// ============================================================================

/// Tool executor trait
#[async_trait]
pub trait ToolExecutor: Send + Sync {
    /// Execute the tool with arguments
    async fn execute(&self, arguments: serde_json::Value) -> Result<serde_json::Value>;

    /// Get tool schema
    fn schema(&self) -> &Tool;
}

// ============================================================================
// Swarm Orchestrator
// ============================================================================

/// Swarm execution state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwarmState {
    /// Current step
    pub current_step: u32,
    /// Active agent
    pub active_agent: Option<String>,
    /// Completed steps
    pub completed_steps: Vec<u32>,
    /// Variables
    pub variables: HashMap<String, serde_json::Value>,
    /// Messages history
    pub messages: Vec<Message>,
    /// Total tokens used
    pub total_tokens: u64,
    /// Total cost (USD)
    pub total_cost: f64,
}

impl Default for SwarmState {
    fn default() -> Self {
        Self {
            current_step: 0,
            active_agent: None,
            completed_steps: Vec::new(),
            variables: HashMap::new(),
            messages: Vec::new(),
            total_tokens: 0,
            total_cost: 0.0,
        }
    }
}

/// Swarm orchestrator
pub struct SwarmOrchestrator {
    /// Swarm configuration
    pub swarm: Swarm,
    /// Agent registry
    pub agents: AgentRegistry,
    /// Execution state
    pub state: Arc<RwLock<SwarmState>>,
}

impl SwarmOrchestrator {
    /// Create a new swarm orchestrator
    pub fn new(swarm: Swarm) -> Self {
        Self {
            swarm,
            agents: AgentRegistry::new(),
            state: Arc::new(RwLock::new(SwarmState::default())),
        }
    }

    /// Get current state
    pub async fn get_state(&self) -> SwarmState {
        self.state.read().await.clone()
    }

    /// Reset state
    pub async fn reset(&self) {
        let mut state = self.state.write().await;
        *state = SwarmState::default();
    }

    /// Check if constraints are violated
    pub async fn check_constraints(&self) -> Result<()> {
        let state = self.state.read().await;

        if let Some(constraints) = &self.swarm.constraints {
            // Check iteration limit
            if let Some(max_iter) = constraints.max_total_iterations {
                if state.completed_steps.len() as u32 >= max_iter {
                    return Err(WiaAiError::ResourceExhausted(
                        "Maximum iterations exceeded".to_string(),
                    ));
                }
            }

            // Check token limit
            if let Some(max_tokens) = constraints.max_tokens_total {
                if state.total_tokens >= max_tokens {
                    return Err(WiaAiError::ResourceExhausted(
                        "Maximum tokens exceeded".to_string(),
                    ));
                }
            }

            // Check budget limit
            if let Some(budget) = constraints.budget_usd {
                if state.total_cost >= budget {
                    return Err(WiaAiError::ResourceExhausted(
                        "Budget exceeded".to_string(),
                    ));
                }
            }
        }

        Ok(())
    }
}

// ============================================================================
// Builders
// ============================================================================

/// Model builder
#[derive(Debug, Default)]
pub struct ModelBuilder {
    model_id: Option<String>,
    name: Option<String>,
    family: Option<String>,
    version: Option<String>,
    provider: Option<String>,
    arch_type: Option<ArchitectureType>,
    context_length: Option<u64>,
    modalities: Option<Vec<Modality>>,
}

impl ModelBuilder {
    /// Create a new model builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set model ID
    pub fn model_id(mut self, id: impl Into<String>) -> Self {
        self.model_id = Some(id.into());
        self
    }

    /// Set model name
    pub fn name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set model family
    pub fn family(mut self, family: impl Into<String>) -> Self {
        self.family = Some(family.into());
        self
    }

    /// Set version
    pub fn version(mut self, version: impl Into<String>) -> Self {
        self.version = Some(version.into());
        self
    }

    /// Set provider
    pub fn provider(mut self, provider: impl Into<String>) -> Self {
        self.provider = Some(provider.into());
        self
    }

    /// Set architecture type
    pub fn architecture(mut self, arch_type: ArchitectureType) -> Self {
        self.arch_type = Some(arch_type);
        self
    }

    /// Set context length
    pub fn context_length(mut self, length: u64) -> Self {
        self.context_length = Some(length);
        self
    }

    /// Set modalities
    pub fn modalities(mut self, modalities: Vec<Modality>) -> Self {
        self.modalities = Some(modalities);
        self
    }

    /// Build the model
    pub fn build(self) -> Result<Model> {
        let model_id = self
            .model_id
            .ok_or_else(|| WiaAiError::validation_field("model_id is required", "model_id"))?;

        let name = self
            .name
            .ok_or_else(|| WiaAiError::validation_field("name is required", "name"))?;

        let arch_type = self.arch_type.unwrap_or(ArchitectureType::Transformer);

        Ok(Model {
            wia_version: WIA_VERSION.to_string(),
            model_id,
            model_info: ModelInfo {
                name,
                family: self.family,
                version: self.version,
                release_date: None,
                provider: self.provider,
            },
            architecture: Architecture {
                arch_type,
                variant: None,
                parameters: None,
                layers: None,
                context_length: self.context_length,
                vocabulary_size: None,
            },
            capabilities: self.modalities.map(|m| Capabilities {
                modalities: Some(m),
                ..Default::default()
            }),
            training: None,
            safety: None,
            deployment: None,
            licensing: None,
        })
    }
}

/// Agent builder
#[derive(Debug, Default)]
pub struct AgentBuilder {
    agent_id: Option<String>,
    name: Option<String>,
    agent_type: Option<AgentType>,
    model_name: Option<String>,
    provider: Option<ModelProvider>,
    system_prompt: Option<String>,
    temperature: Option<f32>,
    max_tokens: Option<u32>,
    tools: Vec<ToolReference>,
}

impl AgentBuilder {
    /// Create a new agent builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set agent ID
    pub fn agent_id(mut self, id: impl Into<String>) -> Self {
        self.agent_id = Some(id.into());
        self
    }

    /// Set agent name
    pub fn name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Set agent type
    pub fn agent_type(mut self, agent_type: AgentType) -> Self {
        self.agent_type = Some(agent_type);
        self
    }

    /// Set model name
    pub fn model(mut self, model_name: impl Into<String>) -> Self {
        self.model_name = Some(model_name.into());
        self
    }

    /// Set provider
    pub fn provider(mut self, provider: ModelProvider) -> Self {
        self.provider = Some(provider);
        self
    }

    /// Set system prompt
    pub fn system_prompt(mut self, prompt: impl Into<String>) -> Self {
        self.system_prompt = Some(prompt.into());
        self
    }

    /// Set temperature
    pub fn temperature(mut self, temp: f32) -> Self {
        self.temperature = Some(temp);
        self
    }

    /// Set max tokens
    pub fn max_tokens(mut self, tokens: u32) -> Self {
        self.max_tokens = Some(tokens);
        self
    }

    /// Add a tool
    pub fn tool(mut self, name: impl Into<String>, description: impl Into<String>) -> Self {
        self.tools.push(ToolReference {
            name: name.into(),
            description: description.into(),
            schema: None,
            parameters: None,
        });
        self
    }

    /// Build the agent
    pub fn build(self) -> Result<Agent> {
        let agent_id = self
            .agent_id
            .ok_or_else(|| WiaAiError::validation_field("agent_id is required", "agent_id"))?;

        let name = self
            .name
            .ok_or_else(|| WiaAiError::validation_field("name is required", "name"))?;

        let model_name = self
            .model_name
            .ok_or_else(|| WiaAiError::validation_field("model is required", "model"))?;

        Ok(Agent {
            wia_version: WIA_VERSION.to_string(),
            agent_id,
            agent_info: AgentInfo {
                name,
                version: Some("1.0.0".to_string()),
                description: None,
                agent_type: self.agent_type,
                author: None,
                created_at: Some(chrono::Utc::now()),
            },
            model: AgentModel {
                model_id: None,
                provider: self.provider,
                model_name,
                temperature: self.temperature,
                max_tokens: self.max_tokens,
                top_p: None,
                top_k: None,
            },
            system_prompt: self.system_prompt.map(|p| SystemPrompt {
                content: Some(p),
                template: None,
                variables: None,
            }),
            capabilities: if self.tools.is_empty() {
                None
            } else {
                Some(AgentCapabilities {
                    tools: Some(self.tools),
                    mcp_servers: None,
                    memory: None,
                })
            },
            behavior: None,
            constraints: None,
            safety: None,
        })
    }
}

// ============================================================================
// Serialization Helpers
// ============================================================================

use serde::{Deserialize, Serialize};

/// Load model from JSON string
pub fn load_model(json: &str) -> Result<Model> {
    serde_json::from_str(json).map_err(WiaAiError::from)
}

/// Load model from file
pub async fn load_model_file(path: &str) -> Result<Model> {
    let content = tokio::fs::read_to_string(path).await?;
    load_model(&content)
}

/// Save model to JSON string
pub fn save_model(model: &Model) -> Result<String> {
    serde_json::to_string_pretty(model).map_err(WiaAiError::from)
}

/// Save model to file
pub async fn save_model_file(model: &Model, path: &str) -> Result<()> {
    let json = save_model(model)?;
    tokio::fs::write(path, json).await?;
    Ok(())
}

/// Load agent from JSON string
pub fn load_agent(json: &str) -> Result<Agent> {
    serde_json::from_str(json).map_err(WiaAiError::from)
}

/// Load agent from file
pub async fn load_agent_file(path: &str) -> Result<Agent> {
    let content = tokio::fs::read_to_string(path).await?;
    load_agent(&content)
}

/// Save agent to JSON string
pub fn save_agent(agent: &Agent) -> Result<String> {
    serde_json::to_string_pretty(agent).map_err(WiaAiError::from)
}

/// Save agent to file
pub async fn save_agent_file(agent: &Agent, path: &str) -> Result<()> {
    let json = save_agent(agent)?;
    tokio::fs::write(path, json).await?;
    Ok(())
}

/// Load swarm from JSON string
pub fn load_swarm(json: &str) -> Result<Swarm> {
    serde_json::from_str(json).map_err(WiaAiError::from)
}

/// Load swarm from file
pub async fn load_swarm_file(path: &str) -> Result<Swarm> {
    let content = tokio::fs::read_to_string(path).await?;
    load_swarm(&content)
}

/// Save swarm to JSON string
pub fn save_swarm(swarm: &Swarm) -> Result<String> {
    serde_json::to_string_pretty(swarm).map_err(WiaAiError::from)
}

/// Save swarm to file
pub async fn save_swarm_file(swarm: &Swarm, path: &str) -> Result<()> {
    let json = save_swarm(swarm)?;
    tokio::fs::write(path, json).await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_model_builder() {
        let model = ModelBuilder::new()
            .model_id("model-test-001")
            .name("Test Model")
            .provider("test")
            .architecture(ArchitectureType::Transformer)
            .context_length(4096)
            .build()
            .unwrap();

        assert_eq!(model.model_id, "model-test-001");
        assert_eq!(model.model_info.name, "Test Model");
        assert_eq!(model.architecture.context_length, Some(4096));
    }

    #[test]
    fn test_agent_builder() {
        let agent = AgentBuilder::new()
            .agent_id("agent-test-001")
            .name("Test Agent")
            .agent_type(AgentType::Assistant)
            .model("claude-3-sonnet")
            .provider(ModelProvider::Anthropic)
            .system_prompt("You are a helpful assistant.")
            .temperature(0.7)
            .build()
            .unwrap();

        assert_eq!(agent.agent_id, "agent-test-001");
        assert_eq!(agent.agent_info.name, "Test Agent");
        assert_eq!(agent.model.model_name, "claude-3-sonnet");
    }

    #[tokio::test]
    async fn test_model_registry() {
        let registry = ModelRegistry::new();

        let model = ModelBuilder::new()
            .model_id("model-test-001")
            .name("Test Model")
            .build()
            .unwrap();

        registry.register(model.clone()).await.unwrap();

        let retrieved = registry.get("model-test-001").await.unwrap();
        assert_eq!(retrieved.model_id, "model-test-001");

        let all = registry.list().await;
        assert_eq!(all.len(), 1);

        registry.remove("model-test-001").await.unwrap();
        assert!(registry.get("model-test-001").await.is_err());
    }

    #[test]
    fn test_message_helpers() {
        let system = Message::system("You are helpful");
        assert_eq!(system.role, MessageRole::System);

        let user = Message::user("Hello");
        assert_eq!(user.role, MessageRole::User);

        let assistant = Message::assistant("Hi there!");
        assert_eq!(assistant.role, MessageRole::Assistant);

        let tool = Message::tool("result", "call-123");
        assert_eq!(tool.role, MessageRole::Tool);
        assert_eq!(tool.tool_call_id, Some("call-123".to_string()));
    }
}
