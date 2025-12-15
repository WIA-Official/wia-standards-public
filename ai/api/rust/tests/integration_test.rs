//! Integration tests for WIA AI SDK

use wia_ai::prelude::*;
use wia_ai::adapters::{
    create_test_agent, create_test_model, create_test_tool, MockProvider, SimulatedAgent,
    SimulatedResponse,
};

// ============================================================================
// Model Tests
// ============================================================================

#[test]
fn test_model_creation_with_builder() {
    let model = ModelBuilder::new()
        .model_id("model-integration-001")
        .name("Integration Test Model")
        .family("test")
        .version("1.0.0")
        .provider("WIA")
        .architecture(ArchitectureType::TransformerDecoder)
        .context_length(32768)
        .modalities(vec![Modality::Text, Modality::Code])
        .build()
        .unwrap();

    assert_eq!(model.model_id, "model-integration-001");
    assert_eq!(model.model_info.name, "Integration Test Model");
    assert_eq!(model.model_info.family, Some("test".to_string()));
    assert_eq!(model.architecture.arch_type, ArchitectureType::TransformerDecoder);
    assert_eq!(model.architecture.context_length, Some(32768));
}

#[test]
fn test_model_serialization_roundtrip() {
    let model = create_test_model();

    // Serialize
    let json = save_model(&model).unwrap();
    assert!(json.contains("model-test-001"));

    // Deserialize
    let loaded: Model = load_model(&json).unwrap();
    assert_eq!(model.model_id, loaded.model_id);
    assert_eq!(model.model_info.name, loaded.model_info.name);
}

#[test]
fn test_model_builder_validation() {
    // Missing model_id
    let result = ModelBuilder::new()
        .name("Test")
        .build();
    assert!(result.is_err());

    // Missing name
    let result = ModelBuilder::new()
        .model_id("test")
        .build();
    assert!(result.is_err());
}

// ============================================================================
// Agent Tests
// ============================================================================

#[test]
fn test_agent_creation_with_builder() {
    let agent = AgentBuilder::new()
        .agent_id("agent-integration-001")
        .name("Integration Test Agent")
        .agent_type(AgentType::Coder)
        .model("claude-3-sonnet")
        .provider(ModelProvider::Anthropic)
        .system_prompt("You are an expert coder.")
        .temperature(0.3)
        .max_tokens(4096)
        .tool("read_file", "Read contents of a file")
        .tool("write_file", "Write contents to a file")
        .build()
        .unwrap();

    assert_eq!(agent.agent_id, "agent-integration-001");
    assert_eq!(agent.agent_info.name, "Integration Test Agent");
    assert_eq!(agent.model.model_name, "claude-3-sonnet");
    assert_eq!(agent.model.provider, Some(ModelProvider::Anthropic));
    assert_eq!(agent.model.temperature, Some(0.3));

    let tools = agent.capabilities.unwrap().tools.unwrap();
    assert_eq!(tools.len(), 2);
}

#[test]
fn test_agent_serialization_roundtrip() {
    let agent = create_test_agent();

    let json = save_agent(&agent).unwrap();
    let loaded: Agent = load_agent(&json).unwrap();

    assert_eq!(agent.agent_id, loaded.agent_id);
    assert_eq!(agent.agent_info.name, loaded.agent_info.name);
}

#[test]
fn test_agent_builder_validation() {
    // Missing agent_id
    let result = AgentBuilder::new()
        .name("Test")
        .model("test")
        .build();
    assert!(result.is_err());

    // Missing model
    let result = AgentBuilder::new()
        .agent_id("test")
        .name("Test")
        .build();
    assert!(result.is_err());
}

// ============================================================================
// Registry Tests
// ============================================================================

#[tokio::test]
async fn test_model_registry_operations() {
    let registry = ModelRegistry::new();

    // Register models
    let model1 = ModelBuilder::new()
        .model_id("model-reg-001")
        .name("Model 1")
        .build()
        .unwrap();

    let model2 = ModelBuilder::new()
        .model_id("model-reg-002")
        .name("Model 2")
        .build()
        .unwrap();

    registry.register(model1).await.unwrap();
    registry.register(model2).await.unwrap();

    // List
    let models = registry.list().await;
    assert_eq!(models.len(), 2);

    // Get
    let model = registry.get("model-reg-001").await.unwrap();
    assert_eq!(model.model_info.name, "Model 1");

    // Get non-existent
    let result = registry.get("model-nonexistent").await;
    assert!(result.is_err());

    // Remove
    let removed = registry.remove("model-reg-001").await.unwrap();
    assert_eq!(removed.model_id, "model-reg-001");

    // Verify removed
    let models = registry.list().await;
    assert_eq!(models.len(), 1);
}

#[tokio::test]
async fn test_agent_registry_operations() {
    let registry = AgentRegistry::new();

    let agent = create_test_agent();
    registry.register(agent.clone()).await.unwrap();

    let retrieved = registry.get(&agent.agent_id).await.unwrap();
    assert_eq!(retrieved.agent_info.name, agent.agent_info.name);

    let agents = registry.list().await;
    assert_eq!(agents.len(), 1);
}

#[tokio::test]
async fn test_tool_registry_operations() {
    let registry = ToolRegistry::new();

    let tool = create_test_tool();
    registry.register(tool.clone()).await.unwrap();

    let retrieved = registry.get(&tool.name).await.unwrap();
    assert_eq!(retrieved.description, tool.description);
}

// ============================================================================
// Simulator Tests
// ============================================================================

#[tokio::test]
async fn test_simulated_agent_execution() {
    let agent = create_test_agent();
    let sim = SimulatedAgent::new(agent);

    // Queue responses
    sim.queue_response(SimulatedResponse::text("First response")).await;
    sim.queue_response(SimulatedResponse::text("Second response")).await;

    // Execute
    let response1 = sim.chat("Hello").await.unwrap();
    assert_eq!(response1.content, "First response");

    let response2 = sim.chat("Hi again").await.unwrap();
    assert_eq!(response2.content, "Second response");

    // Check history
    assert_eq!(sim.call_count().await, 2);
}

#[tokio::test]
async fn test_simulated_agent_with_tool_calls() {
    let agent = create_test_agent();
    let sim = SimulatedAgent::new(agent);

    let tool_call = ToolCall {
        id: "call-001".to_string(),
        name: "test_tool".to_string(),
        arguments: serde_json::json!({"input": "test"}),
    };

    sim.queue_response(SimulatedResponse::with_tools(
        "I'll use a tool",
        vec![tool_call],
    ))
    .await;

    let response = sim.chat("Use a tool").await.unwrap();
    assert!(response.tool_calls.is_some());
    assert_eq!(response.tool_calls.unwrap().len(), 1);
}

#[tokio::test]
async fn test_simulated_agent_with_delay() {
    let agent = create_test_agent();
    let sim = SimulatedAgent::new(agent);

    sim.queue_response(SimulatedResponse::text("Delayed").with_delay(100))
        .await;

    let start = std::time::Instant::now();
    let response = sim.chat("Test").await.unwrap();
    let elapsed = start.elapsed();

    assert_eq!(response.content, "Delayed");
    assert!(elapsed.as_millis() >= 100);
}

#[tokio::test]
async fn test_simulated_agent_failure() {
    let agent = create_test_agent();
    let sim = SimulatedAgent::new(agent);

    sim.queue_response(SimulatedResponse::fail("Test error")).await;

    let result = sim.chat("Test").await;
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("Test error"));
}

#[tokio::test]
async fn test_mock_provider() {
    let provider = MockProvider::new("test-provider");

    // Configure response
    provider.set_response("help", "I can help you!").await;

    // Test with configured response
    let response = provider
        .complete("mock-model-small", vec![Message::user("I need help")])
        .await
        .unwrap();
    assert_eq!(response, "I can help you!");

    // Test with default response
    let response = provider
        .complete("mock-model-large", vec![Message::user("Hello")])
        .await
        .unwrap();
    assert!(response.contains("Mock response"));

    // Verify request log
    let log = provider.get_request_log().await;
    assert_eq!(log.len(), 2);
}

// ============================================================================
// Message Tests
// ============================================================================

#[test]
fn test_message_creation() {
    let system = Message::system("System prompt");
    assert_eq!(system.role, MessageRole::System);
    assert_eq!(system.content, "System prompt");

    let user = Message::user("User input");
    assert_eq!(user.role, MessageRole::User);

    let assistant = Message::assistant("Response");
    assert_eq!(assistant.role, MessageRole::Assistant);

    let tool = Message::tool("Result", "call-123");
    assert_eq!(tool.role, MessageRole::Tool);
    assert_eq!(tool.tool_call_id, Some("call-123".to_string()));
}

// ============================================================================
// Error Tests
// ============================================================================

#[test]
fn test_error_types() {
    let err = WiaAiError::validation("Invalid input");
    assert!(err.to_string().contains("Invalid input"));

    let err = WiaAiError::validation_field("Must be positive", "temperature");
    assert!(err.to_string().contains("Must be positive"));

    let err = WiaAiError::api(500, "Internal error");
    assert!(err.to_string().contains("500"));

    let err = WiaAiError::agent("agent-001", "Failed to execute");
    assert!(err.to_string().contains("Failed to execute"));
}

#[test]
fn test_error_retryable() {
    assert!(WiaAiError::NetworkError("timeout".to_string()).is_retryable());
    assert!(WiaAiError::Timeout("request".to_string()).is_retryable());
    assert!(WiaAiError::api(503, "unavailable").is_retryable());

    assert!(!WiaAiError::api(400, "bad request").is_retryable());
    assert!(!WiaAiError::validation("invalid").is_retryable());
}

// ============================================================================
// Swarm Tests
// ============================================================================

#[tokio::test]
async fn test_swarm_orchestrator() {
    let swarm = Swarm {
        wia_version: WIA_VERSION.to_string(),
        swarm_id: "swarm-test-001".to_string(),
        swarm_info: SwarmInfo {
            name: "Test Swarm".to_string(),
            description: None,
            version: Some("1.0.0".to_string()),
            architecture: Some(SwarmArchitecture::Hierarchical),
            author: None,
            created_at: None,
        },
        agents: vec![
            AgentReference {
                agent_id: "agent-1".to_string(),
                role: "leader".to_string(),
                description: None,
                config: None,
                instances: Some(1),
            },
        ],
        topology: SwarmTopology {
            topology_type: TopologyType::Star,
            root: Some("agent-1".to_string()),
            connections: None,
        },
        communication: None,
        workflow: None,
        constraints: Some(SwarmConstraints {
            max_total_iterations: Some(10),
            max_concurrent_agents: None,
            timeout_seconds: Some(300),
            budget_usd: Some(1.0),
            max_tokens_total: Some(10000),
        }),
        error_handling: None,
        monitoring: None,
    };

    let orchestrator = SwarmOrchestrator::new(swarm);

    // Check initial state
    let state = orchestrator.get_state().await;
    assert_eq!(state.current_step, 0);
    assert!(state.completed_steps.is_empty());

    // Check constraints
    orchestrator.check_constraints().await.unwrap();

    // Reset
    orchestrator.reset().await;
    let state = orchestrator.get_state().await;
    assert!(state.variables.is_empty());
}

#[test]
fn test_swarm_serialization() {
    let swarm = Swarm {
        wia_version: WIA_VERSION.to_string(),
        swarm_id: "swarm-serial-001".to_string(),
        swarm_info: SwarmInfo {
            name: "Serialization Test".to_string(),
            description: None,
            version: None,
            architecture: None,
            author: None,
            created_at: None,
        },
        agents: vec![],
        topology: SwarmTopology {
            topology_type: TopologyType::Flat,
            root: None,
            connections: None,
        },
        communication: None,
        workflow: None,
        constraints: None,
        error_handling: None,
        monitoring: None,
    };

    let json = save_swarm(&swarm).unwrap();
    let loaded: Swarm = load_swarm(&json).unwrap();

    assert_eq!(swarm.swarm_id, loaded.swarm_id);
}

// ============================================================================
// Type Enum Tests
// ============================================================================

#[test]
fn test_architecture_types() {
    let types = vec![
        ArchitectureType::Transformer,
        ArchitectureType::TransformerDecoder,
        ArchitectureType::TransformerEncDec,
        ArchitectureType::Moe,
        ArchitectureType::Ssm,
        ArchitectureType::Diffusion,
        ArchitectureType::Neuromorphic,
    ];

    for arch_type in types {
        let model = ModelBuilder::new()
            .model_id("test")
            .name("Test")
            .architecture(arch_type.clone())
            .build()
            .unwrap();

        assert_eq!(model.architecture.arch_type, arch_type);
    }
}

#[test]
fn test_agent_types() {
    let types = vec![
        AgentType::Assistant,
        AgentType::Researcher,
        AgentType::Coder,
        AgentType::Writer,
        AgentType::Analyst,
        AgentType::Orchestrator,
        AgentType::Specialist,
    ];

    for agent_type in types {
        let agent = AgentBuilder::new()
            .agent_id("test")
            .name("Test")
            .model("test")
            .agent_type(agent_type.clone())
            .build()
            .unwrap();

        assert_eq!(agent.agent_info.agent_type, Some(agent_type));
    }
}

#[test]
fn test_modality_types() {
    let modalities = vec![
        Modality::Text,
        Modality::Image,
        Modality::Audio,
        Modality::Video,
        Modality::Code,
        Modality::StructuredData,
    ];

    let model = ModelBuilder::new()
        .model_id("test")
        .name("Test")
        .modalities(modalities.clone())
        .build()
        .unwrap();

    let caps = model.capabilities.unwrap();
    assert_eq!(caps.modalities.unwrap(), modalities);
}
