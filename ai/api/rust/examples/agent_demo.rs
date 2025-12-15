//! Agent demonstration example for WIA AI SDK
//!
//! This example demonstrates how to use the simulated agent
//! for testing and development purposes.

use wia_ai::adapters::{create_test_agent, SimulatedAgent, SimulatedResponse};
use wia_ai::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    println!("=== WIA AI Agent Demo ===\n");

    // ========================================================================
    // 1. Create a Simulated Agent
    // ========================================================================
    println!("1. Creating a Simulated Agent...");

    let agent_config = AgentBuilder::new()
        .agent_id("agent-demo-001")
        .name("Demo Assistant")
        .agent_type(AgentType::Assistant)
        .model("simulated-model")
        .provider(ModelProvider::Other)
        .system_prompt("You are a helpful demo assistant.")
        .temperature(0.7)
        .max_tokens(1000)
        .build()?;

    let agent = SimulatedAgent::new(agent_config)
        .with_default_response(SimulatedResponse::text(
            "I'm a simulated assistant. How can I help you?",
        ));

    println!("   Agent created successfully!\n");

    // ========================================================================
    // 2. Queue Some Responses
    // ========================================================================
    println!("2. Queueing responses...");

    agent
        .queue_response(SimulatedResponse::text(
            "Hello! I'm your AI assistant. I can help you with various tasks.",
        ))
        .await;

    agent
        .queue_response(SimulatedResponse::text(
            "The capital of France is Paris. It's known as the 'City of Light'.",
        ))
        .await;

    agent
        .queue_response(SimulatedResponse::text(
            "I can help you write code in many languages including Rust, Python, and TypeScript.",
        ))
        .await;

    println!("   Queued 3 responses\n");

    // ========================================================================
    // 3. Chat with the Agent
    // ========================================================================
    println!("3. Chatting with the agent...\n");

    let conversations = vec![
        "Hello, who are you?",
        "What is the capital of France?",
        "Can you help me write code?",
        "Tell me a joke", // This will use default response
    ];

    for (i, input) in conversations.iter().enumerate() {
        println!("   User: {}", input);

        let response = agent.chat(input).await?;
        println!("   Assistant: {}", response.content);

        if let Some(usage) = &response.usage {
            println!(
                "   (tokens: {} in, {} out)",
                usage.input_tokens, usage.output_tokens
            );
        }
        println!();
    }

    // ========================================================================
    // 4. Check Call History
    // ========================================================================
    println!("4. Checking call history...");

    let history = agent.get_call_history().await;
    println!("   Total calls: {}", history.len());

    for (i, messages) in history.iter().enumerate() {
        println!(
            "   Call {}: {} message(s)",
            i + 1,
            messages.len()
        );
    }
    println!();

    // ========================================================================
    // 5. Demonstrate Tool Calls
    // ========================================================================
    println!("5. Demonstrating tool calls...");

    let tool_call = ToolCall {
        id: "call-001".to_string(),
        name: "web_search".to_string(),
        arguments: serde_json::json!({
            "query": "WIA AI Standard",
            "num_results": 5
        }),
    };

    agent
        .queue_response(SimulatedResponse::with_tools(
            "Let me search for that information...",
            vec![tool_call],
        ))
        .await;

    let response = agent.chat("Search for WIA AI Standard").await?;
    println!("   Response: {}", response.content);

    if let Some(tool_calls) = &response.tool_calls {
        println!("   Tool calls requested:");
        for call in tool_calls {
            println!("     - Tool: {}", call.name);
            println!("       ID: {}", call.id);
            println!("       Args: {}", call.arguments);
        }
    }
    println!();

    // ========================================================================
    // 6. Demonstrate Error Handling
    // ========================================================================
    println!("6. Demonstrating error handling...");

    agent
        .queue_response(SimulatedResponse::fail("Simulated network error"))
        .await;

    match agent.chat("This should fail").await {
        Ok(_) => println!("   Unexpected success!"),
        Err(e) => println!("   Expected error: {}", e),
    }
    println!();

    // ========================================================================
    // 7. Using Registries
    // ========================================================================
    println!("7. Using registries...");

    let model_registry = ModelRegistry::new();
    let agent_registry = AgentRegistry::new();

    // Register a model
    let model = ModelBuilder::new()
        .model_id("model-registered-001")
        .name("Registered Model")
        .build()?;
    model_registry.register(model).await?;

    // Register an agent
    let agent_config = create_test_agent();
    agent_registry.register(agent_config).await?;

    println!("   Models registered: {}", model_registry.list().await.len());
    println!("   Agents registered: {}", agent_registry.list().await.len());

    // Retrieve
    let retrieved_model = model_registry.get("model-registered-001").await?;
    println!("   Retrieved model: {}", retrieved_model.model_info.name);
    println!();

    // ========================================================================
    // 8. Multi-Agent Swarm Configuration
    // ========================================================================
    println!("8. Creating a multi-agent swarm configuration...");

    let swarm = Swarm {
        wia_version: WIA_VERSION.to_string(),
        swarm_id: "swarm-demo-001".to_string(),
        swarm_info: SwarmInfo {
            name: "Demo Development Team".to_string(),
            description: Some("A demonstration swarm for code development".to_string()),
            version: Some("1.0.0".to_string()),
            architecture: Some(SwarmArchitecture::Hierarchical),
            author: Some("WIA".to_string()),
            created_at: Some(chrono::Utc::now()),
        },
        agents: vec![
            AgentReference {
                agent_id: "agent-pm".to_string(),
                role: "project_manager".to_string(),
                description: Some("Manages the project and delegates tasks".to_string()),
                config: None,
                instances: Some(1),
            },
            AgentReference {
                agent_id: "agent-architect".to_string(),
                role: "architect".to_string(),
                description: Some("Designs system architecture".to_string()),
                config: None,
                instances: Some(1),
            },
            AgentReference {
                agent_id: "agent-developer".to_string(),
                role: "developer".to_string(),
                description: Some("Implements features".to_string()),
                config: None,
                instances: Some(3),
            },
            AgentReference {
                agent_id: "agent-reviewer".to_string(),
                role: "reviewer".to_string(),
                description: Some("Reviews code".to_string()),
                config: None,
                instances: Some(1),
            },
        ],
        topology: SwarmTopology {
            topology_type: TopologyType::Hierarchical,
            root: Some("agent-pm".to_string()),
            connections: Some(vec![
                Connection {
                    from: "agent-pm".to_string(),
                    to: "agent-architect".to_string(),
                    connection_type: ConnectionType::Delegation,
                    bidirectional: Some(false),
                    condition: None,
                },
                Connection {
                    from: "agent-architect".to_string(),
                    to: "agent-developer".to_string(),
                    connection_type: ConnectionType::Delegation,
                    bidirectional: Some(false),
                    condition: None,
                },
                Connection {
                    from: "agent-developer".to_string(),
                    to: "agent-reviewer".to_string(),
                    connection_type: ConnectionType::Handoff,
                    bidirectional: Some(false),
                    condition: None,
                },
            ]),
        },
        communication: Some(SwarmCommunication {
            protocol: Some(CommunicationProtocol::A2a),
            message_format: Some(MessageFormat::Json),
            shared_memory: Some(SharedMemory {
                memory_type: Some(SharedMemoryType::VectorStore),
                provider: Some("pinecone".to_string()),
                config: None,
            }),
            broadcast: Some(false),
        }),
        workflow: Some(SwarmWorkflow {
            workflow_type: Some(WorkflowType::Sequential),
            steps: Some(vec![
                WorkflowStep {
                    step: 1,
                    name: Some("Plan".to_string()),
                    agent: "agent-pm".to_string(),
                    action: "create_plan".to_string(),
                    inputs: None,
                    outputs: Some(vec!["plan".to_string()]),
                    condition: None,
                    loop_until: None,
                    on_success: None,
                    on_failure: None,
                    timeout_seconds: Some(60),
                },
                WorkflowStep {
                    step: 2,
                    name: Some("Design".to_string()),
                    agent: "agent-architect".to_string(),
                    action: "design_architecture".to_string(),
                    inputs: None,
                    outputs: Some(vec!["architecture".to_string()]),
                    condition: None,
                    loop_until: None,
                    on_success: None,
                    on_failure: None,
                    timeout_seconds: Some(120),
                },
                WorkflowStep {
                    step: 3,
                    name: Some("Implement".to_string()),
                    agent: "agent-developer".to_string(),
                    action: "implement".to_string(),
                    inputs: None,
                    outputs: Some(vec!["code".to_string()]),
                    condition: None,
                    loop_until: Some("all_tasks_complete".to_string()),
                    on_success: None,
                    on_failure: Some("goto_step_2".to_string()),
                    timeout_seconds: Some(300),
                },
                WorkflowStep {
                    step: 4,
                    name: Some("Review".to_string()),
                    agent: "agent-reviewer".to_string(),
                    action: "review".to_string(),
                    inputs: None,
                    outputs: Some(vec!["review_result".to_string()]),
                    condition: None,
                    loop_until: None,
                    on_success: None,
                    on_failure: Some("goto_step_3".to_string()),
                    timeout_seconds: Some(120),
                },
            ]),
            entry_point: Some("step_1".to_string()),
            exit_conditions: Some(vec!["review_passed".to_string()]),
        }),
        constraints: Some(SwarmConstraints {
            max_total_iterations: Some(50),
            max_concurrent_agents: Some(3),
            timeout_seconds: Some(3600),
            budget_usd: Some(10.0),
            max_tokens_total: Some(100000),
        }),
        error_handling: Some(ErrorHandling {
            retry_policy: Some(RetryPolicy {
                max_retries: Some(3),
                backoff_strategy: Some(BackoffStrategy::Exponential),
            }),
            fallback_agent: Some("agent-pm".to_string()),
            on_failure: Some(FailureAction::HumanEscalation),
        }),
        monitoring: Some(Monitoring {
            logging_level: Some(LogLevel::Info),
            metrics_enabled: Some(true),
            tracing_enabled: Some(true),
            dashboard_url: Some("https://dashboard.example.com/swarm-demo-001".to_string()),
        }),
    };

    println!("   Swarm: {}", swarm.swarm_info.name);
    println!("   Agents: {}", swarm.agents.len());
    if let Some(workflow) = &swarm.workflow {
        if let Some(steps) = &workflow.steps {
            println!("   Workflow steps: {}", steps.len());
        }
    }

    // Serialize swarm
    let swarm_json = save_swarm(&swarm)?;
    println!("   Swarm JSON size: {} bytes", swarm_json.len());

    println!("\n=== Demo Complete ===");

    Ok(())
}
