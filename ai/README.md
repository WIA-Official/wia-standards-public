# WIA AI Standard

**Artificial Intelligence Standards**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/standard-WIA%20AI-orange.svg)](https://ai.wia.live)

---

<div align="center">

🤖 **Part of WIA Standards Ecosystem**

[WIA Standards Hub](https://wia.live/standards) • [API Portal](https://api.wia.live)

---

**홍익인간 (弘益人間)** - *Benefit All Humanity*

</div>

---

## 🌍 Overview

WIA AI is an open standard for artificial intelligence standards.

This standard aims to:
- Unify data formats across the industry
- Provide standard APIs for developers  
- Enable interoperability between devices and systems
- Accelerate innovation through open collaboration

---

## 📋 Specification Phases

| Phase | Title | Description | Status |
|:-----:|-------|-------------|:------:|
| **1** | Data Format | Standard data format | ✅ Complete |
| **2** | API Interface | SDK for developers | ✅ Complete |
| **3** | Communication Protocol | Device protocols | ⏳ Planned |
| **4** | Ecosystem Integration | WIA integration | ✅ Complete |

---

## 🚀 Quick Start

### Phase 1 Specifications

The Phase 1 specification defines standard data formats for:

- **Model Metadata** (`model.schema.json`) - AI model definitions and capabilities
- **Agent Definition** (`agent.schema.json`) - AI agent configuration and tools
- **Experiment Record** (`experiment.schema.json`) - Training/evaluation experiments
- **Evaluation Results** (`evaluation.schema.json`) - Benchmark and test results
- **Safety Assessment** (`safety-assessment.schema.json`) - Safety evaluation reports
- **Prompt Templates** (`prompt.schema.json`) - Prompt engineering templates
- **Multi-Agent Swarm** (`swarm.schema.json`) - Multi-agent system configuration
- **Tool Definition** (`tool.schema.json`) - Agent tool schemas

See [spec/PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) for complete documentation.

### Phase 2: Rust SDK

The Phase 2 implementation provides a high-performance Rust SDK:

```rust
use wia_ai::prelude::*;

// Create an AI model
let model = ModelBuilder::new()
    .model_id("model-example-001")
    .name("Example Model")
    .architecture(ArchitectureType::TransformerDecoder)
    .context_length(128000)
    .build()?;

// Create an AI agent
let agent = AgentBuilder::new()
    .agent_id("agent-assistant-001")
    .name("AI Assistant")
    .model("claude-3-5-sonnet")
    .provider(ModelProvider::Anthropic)
    .system_prompt("You are a helpful assistant.")
    .build()?;
```

See [api/rust/README.md](api/rust/README.md) for complete documentation.

### Phase 4: WIA Ecosystem Integration

The Phase 4 implementation enables AI integration with other WIA standards:

```rust
use wia_ai::integration::*;
use wia_ai::integration::adapters::*;

// Create the WIA-AI Hub
let hub = WiaAiHub::new();

// Register input adapters (WIA → AI)
hub.register_input_adapter(Box::new(AacInputAdapter::new())).await;
hub.register_input_adapter(Box::new(BciInputAdapter::new())).await;
hub.register_input_adapter(Box::new(VoiceInputAdapter::new())).await;

// Register output adapters (AI → WIA)
hub.register_output_adapter(Box::new(TtsOutputAdapter::new())).await;
hub.register_output_adapter(Box::new(BrailleOutputAdapter::new())).await;

// Process AAC input → AI → TTS/Braille output
let message = WiaMessage::text(WiaStandardType::Aac, WiaStandardType::Ai, "Hello");
let responses = hub.conversation(message, vec![WiaStandardType::Tts]).await?;
```

Supported integrations:
- **Input**: AAC, BCI, Voice
- **Output**: TTS, Braille, ISP (Sign Language)

See [spec/PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) for complete documentation.

---

## 📁 Structure

```
ai/
├── spec/                    # Specifications
│   ├── RESEARCH-PHASE-1.md  # Phase 1 research report
│   ├── PHASE-1-DATA-FORMAT.md # Data format specification
│   ├── RESEARCH-PHASE-4.md  # Phase 4 research report
│   ├── PHASE-4-INTEGRATION.md # Ecosystem integration spec
│   └── schemas/             # JSON Schema definitions
│       ├── model.schema.json
│       ├── agent.schema.json
│       ├── experiment.schema.json
│       ├── evaluation.schema.json
│       ├── safety-assessment.schema.json
│       ├── prompt.schema.json
│       ├── swarm.schema.json
│       └── tool.schema.json
├── api/
│   └── rust/                # Rust SDK (Phase 2 & 4)
│       ├── Cargo.toml
│       ├── src/
│       │   ├── lib.rs
│       │   ├── types.rs
│       │   ├── error.rs
│       │   ├── core/
│       │   ├── adapters/
│       │   └── integration/  # WIA Ecosystem Integration
│       │       ├── mod.rs
│       │       ├── hub.rs
│       │       ├── connector.rs
│       │       └── adapters/  # AAC, BCI, TTS, Braille
│       ├── tests/
│       └── examples/
├── examples/
├── prompts/                 # Claude Code prompts
└── docs/
```

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| **Website** | https://ai.wia.live |
| **Standards Hub** | https://wia.live/standards |
| **GitHub** | https://github.com/WIA-Official/wia-standards/tree/main/ai |

---

## 📜 License

MIT License - This standard belongs to humanity.

---

<div align="center">

🤟 **홍익인간 (弘益人間)** - Benefit All Humanity

© 2025 SmileStory Inc. / WIA

</div>
