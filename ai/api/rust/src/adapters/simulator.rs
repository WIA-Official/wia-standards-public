//! WIA AI Simulator Adapter
//!
//! This module provides a simulator adapter for testing AI agents
//! without connecting to actual AI providers.

use crate::core::{AgentExecutor, AgentResponse, Message, ToolCall, UsageStats};
use crate::error::{Result, WiaAiError};
use crate::types::*;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

// ============================================================================
// Simulated Agent
// ============================================================================

/// Simulated response configuration
#[derive(Debug, Clone)]
pub struct SimulatedResponse {
    /// Response content
    pub content: String,
    /// Tool calls to make
    pub tool_calls: Option<Vec<ToolCall>>,
    /// Delay in milliseconds before responding
    pub delay_ms: Option<u64>,
    /// Whether to fail
    pub should_fail: Option<String>,
}

impl SimulatedResponse {
    /// Create a simple text response
    pub fn text(content: impl Into<String>) -> Self {
        Self {
            content: content.into(),
            tool_calls: None,
            delay_ms: None,
            should_fail: None,
        }
    }

    /// Create a response with tool calls
    pub fn with_tools(content: impl Into<String>, tool_calls: Vec<ToolCall>) -> Self {
        Self {
            content: content.into(),
            tool_calls: Some(tool_calls),
            delay_ms: None,
            should_fail: None,
        }
    }

    /// Add delay
    pub fn with_delay(mut self, delay_ms: u64) -> Self {
        self.delay_ms = Some(delay_ms);
        self
    }

    /// Make it fail
    pub fn fail(error: impl Into<String>) -> Self {
        Self {
            content: String::new(),
            tool_calls: None,
            delay_ms: None,
            should_fail: Some(error.into()),
        }
    }
}

/// Simulated agent executor for testing
pub struct SimulatedAgent {
    /// Agent configuration
    pub agent: Agent,
    /// Response queue
    responses: Arc<RwLock<Vec<SimulatedResponse>>>,
    /// Default response
    default_response: SimulatedResponse,
    /// Call history
    call_history: Arc<RwLock<Vec<Vec<Message>>>>,
}

impl SimulatedAgent {
    /// Create a new simulated agent
    pub fn new(agent: Agent) -> Self {
        Self {
            agent,
            responses: Arc::new(RwLock::new(Vec::new())),
            default_response: SimulatedResponse::text("I am a simulated AI assistant."),
            call_history: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Set default response
    pub fn with_default_response(mut self, response: SimulatedResponse) -> Self {
        self.default_response = response;
        self
    }

    /// Queue a response
    pub async fn queue_response(&self, response: SimulatedResponse) {
        let mut responses = self.responses.write().await;
        responses.push(response);
    }

    /// Queue multiple responses
    pub async fn queue_responses(&self, responses: Vec<SimulatedResponse>) {
        let mut queue = self.responses.write().await;
        queue.extend(responses);
    }

    /// Get call history
    pub async fn get_call_history(&self) -> Vec<Vec<Message>> {
        self.call_history.read().await.clone()
    }

    /// Clear call history
    pub async fn clear_history(&self) {
        let mut history = self.call_history.write().await;
        history.clear();
    }

    /// Get number of calls
    pub async fn call_count(&self) -> usize {
        self.call_history.read().await.len()
    }
}

#[async_trait]
impl AgentExecutor for SimulatedAgent {
    async fn execute(&self, messages: Vec<Message>) -> Result<AgentResponse> {
        // Record call
        {
            let mut history = self.call_history.write().await;
            history.push(messages.clone());
        }

        // Get next response
        let response = {
            let mut responses = self.responses.write().await;
            if responses.is_empty() {
                self.default_response.clone()
            } else {
                responses.remove(0)
            }
        };

        // Check for failure
        if let Some(error) = response.should_fail {
            return Err(WiaAiError::agent(&self.agent.agent_id, error));
        }

        // Apply delay
        if let Some(delay) = response.delay_ms {
            tokio::time::sleep(tokio::time::Duration::from_millis(delay)).await;
        }

        // Calculate fake usage based on message length
        let input_tokens = messages
            .iter()
            .map(|m| m.content.len() as u64 / 4)
            .sum::<u64>();
        let output_tokens = response.content.len() as u64 / 4;

        Ok(AgentResponse {
            content: response.content,
            tool_calls: response.tool_calls,
            usage: Some(UsageStats {
                input_tokens,
                output_tokens,
                total_tokens: input_tokens + output_tokens,
            }),
            stop_reason: Some("end_turn".to_string()),
        })
    }
}

// ============================================================================
// Simulated Tool
// ============================================================================

/// Simulated tool handler
pub type SimulatedToolHandler =
    Box<dyn Fn(serde_json::Value) -> Result<serde_json::Value> + Send + Sync>;

/// Simulated tool executor
pub struct SimulatedTool {
    /// Tool definition
    pub tool: Tool,
    /// Handler function
    handler: Option<SimulatedToolHandler>,
    /// Static responses
    responses: Arc<RwLock<HashMap<String, serde_json::Value>>>,
}

impl SimulatedTool {
    /// Create a new simulated tool
    pub fn new(tool: Tool) -> Self {
        Self {
            tool,
            handler: None,
            responses: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Set handler function
    pub fn with_handler<F>(mut self, handler: F) -> Self
    where
        F: Fn(serde_json::Value) -> Result<serde_json::Value> + Send + Sync + 'static,
    {
        self.handler = Some(Box::new(handler));
        self
    }

    /// Add a static response for specific input
    pub async fn add_response(&self, input_key: &str, response: serde_json::Value) {
        let mut responses = self.responses.write().await;
        responses.insert(input_key.to_string(), response);
    }

    /// Execute the tool
    pub async fn execute(&self, arguments: serde_json::Value) -> Result<serde_json::Value> {
        // Check for static response
        if let Some(key) = arguments.get("key").and_then(|v| v.as_str()) {
            let responses = self.responses.read().await;
            if let Some(response) = responses.get(key) {
                return Ok(response.clone());
            }
        }

        // Use handler if available
        if let Some(ref handler) = self.handler {
            return handler(arguments);
        }

        // Default response
        Ok(serde_json::json!({
            "status": "success",
            "result": "Simulated tool execution complete"
        }))
    }
}

// ============================================================================
// Mock Provider
// ============================================================================

/// Mock AI provider for testing
pub struct MockProvider {
    /// Provider name
    pub name: String,
    /// Available models
    pub models: Vec<String>,
    /// Request log
    request_log: Arc<RwLock<Vec<MockRequest>>>,
    /// Configured responses
    responses: Arc<RwLock<HashMap<String, String>>>,
}

/// Mock request log entry
#[derive(Debug, Clone)]
pub struct MockRequest {
    /// Model used
    pub model: String,
    /// Messages sent
    pub messages: Vec<Message>,
    /// Timestamp
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

impl MockProvider {
    /// Create a new mock provider
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            models: vec![
                "mock-model-small".to_string(),
                "mock-model-large".to_string(),
            ],
            request_log: Arc::new(RwLock::new(Vec::new())),
            responses: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Add available model
    pub fn add_model(&mut self, model: impl Into<String>) {
        self.models.push(model.into());
    }

    /// Configure response for a pattern
    pub async fn set_response(&self, pattern: &str, response: &str) {
        let mut responses = self.responses.write().await;
        responses.insert(pattern.to_string(), response.to_string());
    }

    /// Get request log
    pub async fn get_request_log(&self) -> Vec<MockRequest> {
        self.request_log.read().await.clone()
    }

    /// Clear request log
    pub async fn clear_log(&self) {
        let mut log = self.request_log.write().await;
        log.clear();
    }

    /// Generate completion
    pub async fn complete(&self, model: &str, messages: Vec<Message>) -> Result<String> {
        // Validate model
        if !self.models.contains(&model.to_string()) {
            return Err(WiaAiError::ModelNotFound(model.to_string()));
        }

        // Log request
        {
            let mut log = self.request_log.write().await;
            log.push(MockRequest {
                model: model.to_string(),
                messages: messages.clone(),
                timestamp: chrono::Utc::now(),
            });
        }

        // Check for configured response
        let responses = self.responses.read().await;
        for message in messages.iter().rev() {
            for (pattern, response) in responses.iter() {
                if message.content.contains(pattern) {
                    return Ok(response.clone());
                }
            }
        }

        // Default response
        Ok(format!(
            "Mock response from {} using model {}",
            self.name, model
        ))
    }
}

// ============================================================================
// Test Helpers
// ============================================================================

/// Create a test model
pub fn create_test_model() -> Model {
    Model {
        wia_version: "1.0.0".to_string(),
        model_id: "model-test-001".to_string(),
        model_info: ModelInfo {
            name: "Test Model".to_string(),
            family: Some("test".to_string()),
            version: Some("1.0".to_string()),
            release_date: None,
            provider: Some("Test Provider".to_string()),
        },
        architecture: Architecture {
            arch_type: ArchitectureType::Transformer,
            variant: Some("decoder_only".to_string()),
            parameters: Some(Parameters {
                total: Some(1_000_000),
                trainable: Some(1_000_000),
                embedding: Some(100_000),
            }),
            layers: Some(Layers {
                num_layers: Some(12),
                hidden_size: Some(768),
                num_attention_heads: Some(12),
                intermediate_size: Some(3072),
            }),
            context_length: Some(4096),
            vocabulary_size: Some(50000),
        },
        capabilities: Some(Capabilities {
            modalities: Some(vec![Modality::Text]),
            languages: Some(vec!["en".to_string()]),
            tasks: Some(vec!["text_generation".to_string()]),
            features: Some(Features {
                function_calling: Some(true),
                structured_output: Some(true),
                vision: Some(false),
                extended_thinking: Some(false),
                streaming: Some(true),
            }),
        }),
        training: None,
        safety: None,
        deployment: None,
        licensing: None,
    }
}

/// Create a test agent
pub fn create_test_agent() -> Agent {
    Agent {
        wia_version: "1.0.0".to_string(),
        agent_id: "agent-test-001".to_string(),
        agent_info: AgentInfo {
            name: "Test Agent".to_string(),
            version: Some("1.0.0".to_string()),
            description: Some("A test agent for unit testing".to_string()),
            agent_type: Some(AgentType::Assistant),
            author: Some("Test Author".to_string()),
            created_at: Some(chrono::Utc::now()),
        },
        model: AgentModel {
            model_id: Some("model-test-001".to_string()),
            provider: Some(ModelProvider::Other),
            model_name: "test-model".to_string(),
            temperature: Some(0.7),
            max_tokens: Some(1000),
            top_p: None,
            top_k: None,
        },
        system_prompt: Some(SystemPrompt {
            content: Some("You are a helpful test assistant.".to_string()),
            template: None,
            variables: None,
        }),
        capabilities: Some(AgentCapabilities {
            tools: Some(vec![ToolReference {
                name: "test_tool".to_string(),
                description: "A test tool".to_string(),
                schema: None,
                parameters: None,
            }]),
            mcp_servers: None,
            memory: None,
        }),
        behavior: None,
        constraints: Some(AgentConstraints {
            max_iterations: Some(10),
            timeout_seconds: Some(60),
            rate_limits: None,
        }),
        safety: None,
    }
}

/// Create a test tool
pub fn create_test_tool() -> Tool {
    Tool {
        name: "test_tool".to_string(),
        description: "A test tool for unit testing".to_string(),
        version: Some("1.0.0".to_string()),
        category: Some(ToolCategory::Utility),
        parameters: ToolParameters {
            param_type: "object".to_string(),
            required: Some(vec!["input".to_string()]),
            properties: {
                let mut props = HashMap::new();
                props.insert(
                    "input".to_string(),
                    ToolParameter {
                        param_type: "string".to_string(),
                        description: Some("Input value".to_string()),
                        default: None,
                        allowed_values: None,
                        minimum: None,
                        maximum: None,
                        min_length: None,
                        max_length: None,
                        pattern: None,
                        format: None,
                    },
                );
                props
            },
        },
        returns: Some(serde_json::json!({
            "type": "object",
            "properties": {
                "result": {"type": "string"}
            }
        })),
        errors: None,
        examples: None,
        constraints: None,
        permissions: None,
        implementation: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_simulated_agent() {
        let agent = create_test_agent();
        let sim = SimulatedAgent::new(agent);

        // Queue a response
        sim.queue_response(SimulatedResponse::text("Hello from simulation!"))
            .await;

        // Execute
        let response = sim.chat("Hello").await.unwrap();
        assert_eq!(response.content, "Hello from simulation!");

        // Check call history
        let history = sim.get_call_history().await;
        assert_eq!(history.len(), 1);
        assert_eq!(history[0][0].content, "Hello");
    }

    #[tokio::test]
    async fn test_simulated_agent_default_response() {
        let agent = create_test_agent();
        let sim = SimulatedAgent::new(agent)
            .with_default_response(SimulatedResponse::text("Default response"));

        let response = sim.chat("Test").await.unwrap();
        assert_eq!(response.content, "Default response");
    }

    #[tokio::test]
    async fn test_simulated_agent_failure() {
        let agent = create_test_agent();
        let sim = SimulatedAgent::new(agent);

        sim.queue_response(SimulatedResponse::fail("Simulated error"))
            .await;

        let result = sim.chat("Hello").await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_mock_provider() {
        let provider = MockProvider::new("test-provider");
        provider
            .set_response("hello", "Hi there!")
            .await;

        let response = provider
            .complete("mock-model-small", vec![Message::user("hello world")])
            .await
            .unwrap();

        assert_eq!(response, "Hi there!");
    }

    #[tokio::test]
    async fn test_mock_provider_unknown_model() {
        let provider = MockProvider::new("test-provider");

        let result = provider
            .complete("unknown-model", vec![Message::user("hello")])
            .await;

        assert!(result.is_err());
    }

    #[test]
    fn test_create_test_model() {
        let model = create_test_model();
        assert_eq!(model.model_id, "model-test-001");
        assert_eq!(model.model_info.name, "Test Model");
    }

    #[test]
    fn test_create_test_agent() {
        let agent = create_test_agent();
        assert_eq!(agent.agent_id, "agent-test-001");
        assert_eq!(agent.agent_info.name, "Test Agent");
    }

    #[test]
    fn test_create_test_tool() {
        let tool = create_test_tool();
        assert_eq!(tool.name, "test_tool");
    }
}
