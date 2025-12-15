//! # WIA AI SDK
//!
//! WIA AI is an open standard for artificial intelligence systems,
//! providing unified data formats for models, agents, experiments,
//! evaluations, safety assessments, and multi-agent systems.
//!
//! ## Features
//!
//! - **Model Management**: Define and manage AI model metadata
//! - **Agent Configuration**: Configure AI agents with tools and behaviors
//! - **Experiment Tracking**: Record training and evaluation experiments
//! - **Safety Assessment**: Document safety evaluations and compliance
//! - **Multi-Agent Systems**: Orchestrate swarms of cooperating agents
//!
//! ## Quick Start
//!
//! ```rust
//! use wia_ai::prelude::*;
//!
//! // Create a model
//! let model = ModelBuilder::new()
//!     .model_id("model-example-001")
//!     .name("Example Model")
//!     .provider("example")
//!     .architecture(ArchitectureType::Transformer)
//!     .context_length(8192)
//!     .build()
//!     .unwrap();
//!
//! // Create an agent
//! let agent = AgentBuilder::new()
//!     .agent_id("agent-example-001")
//!     .name("Example Agent")
//!     .model("example-model")
//!     .provider(ModelProvider::Other)
//!     .system_prompt("You are a helpful assistant.")
//!     .temperature(0.7)
//!     .build()
//!     .unwrap();
//!
//! // Serialize to JSON
//! let json = save_agent(&agent).unwrap();
//! println!("{}", json);
//! ```
//!
//! ## Working with Simulators
//!
//! ```rust
//! use wia_ai::prelude::*;
//! use wia_ai::adapters::{SimulatedAgent, SimulatedResponse, create_test_agent};
//!
//! #[tokio::main]
//! async fn main() {
//!     let agent = create_test_agent();
//!     let sim = SimulatedAgent::new(agent);
//!
//!     // Queue responses for testing
//!     sim.queue_response(SimulatedResponse::text("Hello!")).await;
//!
//!     // Execute
//!     let response = sim.chat("Hi").await.unwrap();
//!     println!("{}", response.content);
//! }
//! ```
//!
//! ## Multi-Agent Swarms
//!
//! ```rust
//! use wia_ai::prelude::*;
//!
//! // Create swarm configuration
//! let swarm = Swarm {
//!     wia_version: WIA_VERSION.to_string(),
//!     swarm_id: "swarm-dev-team-001".to_string(),
//!     swarm_info: SwarmInfo {
//!         name: "Development Team".to_string(),
//!         description: Some("Multi-agent development team".to_string()),
//!         version: Some("1.0.0".to_string()),
//!         architecture: Some(SwarmArchitecture::Hierarchical),
//!         author: None,
//!         created_at: None,
//!     },
//!     agents: vec![
//!         AgentReference {
//!             agent_id: "agent-planner".to_string(),
//!             role: "planner".to_string(),
//!             description: Some("Plans tasks".to_string()),
//!             config: None,
//!             instances: Some(1),
//!         },
//!         AgentReference {
//!             agent_id: "agent-coder".to_string(),
//!             role: "coder".to_string(),
//!             description: Some("Writes code".to_string()),
//!             config: None,
//!             instances: Some(1),
//!         },
//!     ],
//!     topology: SwarmTopology {
//!         topology_type: TopologyType::Hierarchical,
//!         root: Some("agent-planner".to_string()),
//!         connections: Some(vec![
//!             Connection {
//!                 from: "agent-planner".to_string(),
//!                 to: "agent-coder".to_string(),
//!                 connection_type: ConnectionType::Delegation,
//!                 bidirectional: Some(false),
//!                 condition: None,
//!             },
//!         ]),
//!     },
//!     communication: None,
//!     workflow: None,
//!     constraints: None,
//!     error_handling: None,
//!     monitoring: None,
//! };
//! ```

#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]

pub mod adapters;
pub mod core;
pub mod error;
pub mod integration;
pub mod types;

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::core::*;
    pub use crate::error::{Result, WiaAiError};
    pub use crate::types::*;
}

// Re-export commonly used items at crate root
pub use core::{
    load_agent, load_agent_file, load_model, load_model_file, load_swarm, load_swarm_file,
    save_agent, save_agent_file, save_model, save_model_file, save_swarm, save_swarm_file,
    AgentBuilder, AgentExecutor, AgentRegistry, AgentResponse, Message, MessageRole,
    ModelBuilder, ModelRegistry, SwarmOrchestrator, SwarmState, ToolCall, ToolExecutor,
    ToolRegistry, UsageStats, WIA_VERSION,
};

pub use error::{Result, WiaAiError};

pub use types::{
    Agent, AgentType, ArchitectureType, Evaluation, Experiment, Model, ModelProvider, Modality,
    Prompt, SafetyAssessment, Swarm, Tool,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert_eq!(WIA_VERSION, "1.0.0");
    }

    #[test]
    fn test_prelude_imports() {
        use crate::prelude::*;

        // Verify key types are available
        let _: fn() -> ModelBuilder = ModelBuilder::new;
        let _: fn() -> AgentBuilder = AgentBuilder::new;
    }

    #[test]
    fn test_model_serialization() {
        let model = ModelBuilder::new()
            .model_id("model-test")
            .name("Test")
            .build()
            .unwrap();

        let json = save_model(&model).unwrap();
        let loaded: Model = load_model(&json).unwrap();

        assert_eq!(model.model_id, loaded.model_id);
    }

    #[test]
    fn test_agent_serialization() {
        let agent = AgentBuilder::new()
            .agent_id("agent-test")
            .name("Test")
            .model("test-model")
            .build()
            .unwrap();

        let json = save_agent(&agent).unwrap();
        let loaded: Agent = load_agent(&json).unwrap();

        assert_eq!(agent.agent_id, loaded.agent_id);
    }
}
