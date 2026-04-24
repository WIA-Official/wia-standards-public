//! Basic usage example for WIA AI SDK
//!
//! This example demonstrates how to create and work with
//! AI models, agents, and basic operations.

use wia_ai::prelude::*;

fn main() -> Result<()> {
    println!("=== WIA AI SDK Basic Usage Example ===\n");

    // ========================================================================
    // 1. Create a Model
    // ========================================================================
    println!("1. Creating a Model...");

    let model = ModelBuilder::new()
        .model_id("model-example-001")
        .name("Example LLM")
        .family("example")
        .version("1.0.0")
        .provider("Example Corp")
        .architecture(ArchitectureType::TransformerDecoder)
        .context_length(128000)
        .modalities(vec![Modality::Text, Modality::Image, Modality::Code])
        .build()?;

    println!("   Model ID: {}", model.model_id);
    println!("   Name: {}", model.model_info.name);
    println!(
        "   Context Length: {}",
        model.architecture.context_length.unwrap_or(0)
    );
    println!();

    // ========================================================================
    // 2. Serialize Model to JSON
    // ========================================================================
    println!("2. Serializing Model to JSON...");

    let model_json = save_model(&model)?;
    println!("   JSON preview (first 200 chars):");
    println!("   {}", &model_json[..model_json.len().min(200)]);
    println!("   ...\n");

    // ========================================================================
    // 3. Create an Agent
    // ========================================================================
    println!("3. Creating an Agent...");

    let agent = AgentBuilder::new()
        .agent_id("agent-assistant-001")
        .name("Helpful Assistant")
        .agent_type(AgentType::Assistant)
        .model("claude-3-5-sonnet")
        .provider(ModelProvider::Anthropic)
        .system_prompt(
            "You are a helpful AI assistant. You provide clear, accurate, \
             and concise answers to user questions.",
        )
        .temperature(0.7)
        .max_tokens(4096)
        .tool("web_search", "Search the web for information")
        .tool("read_file", "Read contents of a file")
        .tool("write_file", "Write contents to a file")
        .build()?;

    println!("   Agent ID: {}", agent.agent_id);
    println!("   Name: {}", agent.agent_info.name);
    println!("   Model: {}", agent.model.model_name);
    println!(
        "   Provider: {:?}",
        agent.model.provider.as_ref().unwrap()
    );
    if let Some(caps) = &agent.capabilities {
        if let Some(tools) = &caps.tools {
            println!("   Tools: {}", tools.len());
            for tool in tools {
                println!("     - {}: {}", tool.name, tool.description);
            }
        }
    }
    println!();

    // ========================================================================
    // 4. Serialize Agent to JSON
    // ========================================================================
    println!("4. Serializing Agent to JSON...");

    let agent_json = save_agent(&agent)?;
    println!("   JSON preview (first 200 chars):");
    println!("   {}", &agent_json[..agent_json.len().min(200)]);
    println!("   ...\n");

    // ========================================================================
    // 5. Load from JSON
    // ========================================================================
    println!("5. Loading from JSON...");

    let loaded_model: Model = load_model(&model_json)?;
    let loaded_agent: Agent = load_agent(&agent_json)?;

    println!("   Loaded Model: {}", loaded_model.model_id);
    println!("   Loaded Agent: {}", loaded_agent.agent_id);
    println!();

    // ========================================================================
    // 6. Create Messages
    // ========================================================================
    println!("6. Creating Messages...");

    let messages = vec![
        Message::system("You are a helpful assistant."),
        Message::user("What is the capital of France?"),
        Message::assistant("The capital of France is Paris."),
        Message::user("What about Germany?"),
    ];

    for (i, msg) in messages.iter().enumerate() {
        println!("   Message {}: {:?} - {}", i + 1, msg.role, &msg.content[..msg.content.len().min(50)]);
    }
    println!();

    // ========================================================================
    // 7. Create a Coder Agent
    // ========================================================================
    println!("7. Creating a Specialized Coder Agent...");

    let coder_agent = AgentBuilder::new()
        .agent_id("agent-coder-001")
        .name("Expert Coder")
        .agent_type(AgentType::Coder)
        .model("claude-3-5-sonnet")
        .provider(ModelProvider::Anthropic)
        .system_prompt(
            "You are an expert software engineer. You write clean, \
             efficient, and well-documented code. You follow best practices \
             and consider edge cases.",
        )
        .temperature(0.2) // Lower temperature for more deterministic code
        .max_tokens(8192)
        .tool("execute_code", "Execute code in a sandbox environment")
        .tool("run_tests", "Run test suite")
        .tool("lint_code", "Run linter on code")
        .build()?;

    println!("   Created coder agent: {}", coder_agent.agent_info.name);
    println!(
        "   Temperature: {} (lower for deterministic output)",
        coder_agent.model.temperature.unwrap_or(0.0)
    );
    println!();

    // ========================================================================
    // 8. Version Info
    // ========================================================================
    println!("8. Version Information...");
    println!("   WIA AI SDK Version: {}", WIA_VERSION);
    println!("   Model WIA Version: {}", model.wia_version);
    println!("   Agent WIA Version: {}", agent.wia_version);

    println!("\n=== Example Complete ===");

    Ok(())
}
