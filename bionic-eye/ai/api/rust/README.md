# WIA AI Rust SDK

**Rust SDK for WIA AI Standard**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Rust](https://img.shields.io/badge/rust-1.70+-orange.svg)](https://www.rust-lang.org)
[![Crates.io](https://img.shields.io/badge/crates.io-wia--ai-blue.svg)](https://crates.io/crates/wia-ai)

---

## Overview

The WIA AI Rust SDK provides a high-performance implementation of the WIA AI Standard for working with AI models, agents, experiments, evaluations, safety assessments, and multi-agent systems.

## Features

- **Type-Safe**: Full Rust type definitions for all WIA AI schemas
- **Async-First**: Built on Tokio for async operations
- **Builder Pattern**: Fluent APIs for creating models and agents
- **Simulation Support**: Test agents without connecting to AI providers
- **Serialization**: Full JSON serialization/deserialization support
- **Multi-Agent**: Swarm orchestration for multi-agent systems

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
wia-ai = "1.0.0"
tokio = { version = "1", features = ["full"] }
```

## Quick Start

### Create a Model

```rust
use wia_ai::prelude::*;

let model = ModelBuilder::new()
    .model_id("model-example-001")
    .name("Example Model")
    .provider("example")
    .architecture(ArchitectureType::TransformerDecoder)
    .context_length(128000)
    .modalities(vec![Modality::Text, Modality::Image])
    .build()?;

// Serialize to JSON
let json = save_model(&model)?;
```

### Create an Agent

```rust
use wia_ai::prelude::*;

let agent = AgentBuilder::new()
    .agent_id("agent-assistant-001")
    .name("AI Assistant")
    .agent_type(AgentType::Assistant)
    .model("claude-3-5-sonnet")
    .provider(ModelProvider::Anthropic)
    .system_prompt("You are a helpful assistant.")
    .temperature(0.7)
    .max_tokens(4096)
    .tool("web_search", "Search the web")
    .tool("read_file", "Read file contents")
    .build()?;
```

### Using Simulated Agents

```rust
use wia_ai::prelude::*;
use wia_ai::adapters::{SimulatedAgent, SimulatedResponse};

#[tokio::main]
async fn main() -> Result<()> {
    let agent_config = AgentBuilder::new()
        .agent_id("agent-test")
        .name("Test Agent")
        .model("test-model")
        .build()?;

    let sim = SimulatedAgent::new(agent_config);

    // Queue responses
    sim.queue_response(SimulatedResponse::text("Hello!")).await;

    // Chat
    let response = sim.chat("Hi there").await?;
    println!("{}", response.content);

    Ok(())
}
```

### Working with Registries

```rust
use wia_ai::prelude::*;

#[tokio::main]
async fn main() -> Result<()> {
    let registry = ModelRegistry::new();

    // Register
    let model = ModelBuilder::new()
        .model_id("model-001")
        .name("My Model")
        .build()?;
    registry.register(model).await?;

    // Retrieve
    let model = registry.get("model-001").await?;

    // List all
    let models = registry.list().await;

    Ok(())
}
```

### Multi-Agent Swarm

```rust
use wia_ai::prelude::*;

let swarm = Swarm {
    wia_version: WIA_VERSION.to_string(),
    swarm_id: "swarm-dev-team".to_string(),
    swarm_info: SwarmInfo {
        name: "Development Team".to_string(),
        architecture: Some(SwarmArchitecture::Hierarchical),
        ..Default::default()
    },
    agents: vec![
        AgentReference {
            agent_id: "agent-planner".to_string(),
            role: "planner".to_string(),
            ..Default::default()
        },
        AgentReference {
            agent_id: "agent-coder".to_string(),
            role: "coder".to_string(),
            ..Default::default()
        },
    ],
    topology: SwarmTopology {
        topology_type: TopologyType::Hierarchical,
        root: Some("agent-planner".to_string()),
        connections: Some(vec![
            Connection {
                from: "agent-planner".to_string(),
                to: "agent-coder".to_string(),
                connection_type: ConnectionType::Delegation,
                ..Default::default()
            },
        ]),
    },
    ..Default::default()
};

let orchestrator = SwarmOrchestrator::new(swarm);
```

## API Reference

### Types

| Type | Description |
|------|-------------|
| `Model` | AI model metadata |
| `Agent` | Agent configuration |
| `Experiment` | Training/evaluation experiment |
| `Evaluation` | Evaluation results |
| `SafetyAssessment` | Safety assessment |
| `Prompt` | Prompt template |
| `Swarm` | Multi-agent configuration |
| `Tool` | Tool definition |

### Enums

| Enum | Values |
|------|--------|
| `ArchitectureType` | Transformer, TransformerDecoder, Moe, Ssm, Diffusion, Neuromorphic |
| `AgentType` | Assistant, Researcher, Coder, Writer, Analyst, Orchestrator |
| `ModelProvider` | Anthropic, Openai, Google, Meta, Mistral, Local, Other |
| `Modality` | Text, Image, Audio, Video, Code, StructuredData |

### Core Functions

```rust
// Model operations
fn save_model(model: &Model) -> Result<String>;
fn load_model(json: &str) -> Result<Model>;
async fn save_model_file(model: &Model, path: &str) -> Result<()>;
async fn load_model_file(path: &str) -> Result<Model>;

// Agent operations
fn save_agent(agent: &Agent) -> Result<String>;
fn load_agent(json: &str) -> Result<Agent>;

// Swarm operations
fn save_swarm(swarm: &Swarm) -> Result<String>;
fn load_swarm(json: &str) -> Result<Swarm>;
```

### Traits

```rust
#[async_trait]
trait AgentExecutor: Send + Sync {
    async fn execute(&self, messages: Vec<Message>) -> Result<AgentResponse>;
    async fn chat(&self, message: &str) -> Result<AgentResponse>;
}

#[async_trait]
trait ToolExecutor: Send + Sync {
    async fn execute(&self, arguments: serde_json::Value) -> Result<serde_json::Value>;
    fn schema(&self) -> &Tool;
}
```

## Project Structure

```
src/
├── lib.rs           # Main library entry
├── types.rs         # Type definitions
├── error.rs         # Error types
├── core/
│   ├── mod.rs       # Core module
│   └── ai.rs        # AI operations
└── adapters/
    ├── mod.rs       # Adapters module
    └── simulator.rs # Test simulators
tests/
└── integration_test.rs
examples/
├── basic_usage.rs
└── agent_demo.rs
```

## Running Examples

```bash
# Basic usage
cargo run --example basic_usage

# Agent demo
cargo run --example agent_demo
```

## Running Tests

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture

# Run specific test
cargo test test_model_creation
```

## Features

| Feature | Description |
|---------|-------------|
| `default` | Basic functionality |
| `wasm` | WebAssembly support |
| `python` | Python bindings (PyO3) |
| `full` | All features |

## License

MIT License - This standard belongs to humanity.

---

弘益人間 - *Benefit All Humanity*
