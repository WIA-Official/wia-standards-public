# WIA AI Data Format Specification

**Phase 1: Data Format Standard**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01

---

## 1. Overview

### 1.1 Purpose

WIA AI Data Format은 인공지능 시스템의 모델, 에이전트, 실험, 평가 데이터를 위한 통합 표준입니다. 이 표준은 다양한 AI 플랫폼과 프레임워크 간의 상호운용성을 보장하고, 연구 및 프로덕션 환경에서의 재현성을 높이기 위해 설계되었습니다.

### 1.2 Scope

- **In Scope**:
  - AI 모델 메타데이터
  - 에이전트 정의 및 구성
  - 훈련/평가 실험 기록
  - 안전성 평가 결과
  - 멀티 에이전트 시스템 구성
  - 프롬프트 및 템플릿

- **Out of Scope** (Phase 1):
  - 실시간 에이전트 통신 프로토콜 (Phase 3)
  - 연합 학습 통합 (Phase 4)
  - 뉴로모픽 하드웨어 인터페이스 (Phase 4)

### 1.3 Design Principles

1. **Interoperability**: ONNX, HuggingFace 형식과 호환
2. **Extensibility**: 새로운 모델 타입/에이전트 추가 용이
3. **Self-describing**: 메타데이터를 통한 자기 기술적 형식
4. **Safety-first**: 안전성 정보의 표준화된 표현
5. **Accessibility**: JSON 기반의 인간 가독성

---

## 2. Data Structure

### 2.1 Top-Level Structure

```
wia-ai-project/
├── project.json              # Project metadata
├── models/
│   ├── model.json            # Model definition
│   └── checkpoints/          # Model checkpoints
├── agents/
│   ├── agent.json            # Agent definition
│   └── tools/                # Agent tools
├── experiments/
│   ├── experiment.json       # Experiment record
│   └── runs/                 # Individual runs
├── evaluations/
│   ├── evaluation.json       # Evaluation results
│   └── benchmarks/           # Benchmark data
├── safety/
│   ├── assessment.json       # Safety assessment
│   └── tests/                # Safety test results
├── prompts/
│   ├── prompt.json           # Prompt templates
│   └── templates/            # Template files
└── swarm/
    ├── swarm.json            # Multi-agent config
    └── topology/             # Agent topology
```

### 2.2 File Naming Convention

```
{type}_{name}_{version}.{extension}

Examples:
- model_gpt4_v1.0.0.json
- agent_researcher_v2.1.0.json
- experiment_finetune_20250115.json
```

---

## 3. Model Metadata

### 3.1 model.json

```json
{
  "$schema": "https://wia.live/schemas/ai/model.schema.json",
  "wia_version": "1.0.0",
  "model_id": "model-claude-opus-001",

  "model_info": {
    "name": "Claude Opus 4.5",
    "family": "claude",
    "version": "4.5",
    "release_date": "2025-01-01",
    "provider": "Anthropic"
  },

  "architecture": {
    "type": "transformer",
    "variant": "decoder_only",
    "parameters": {
      "total": null,
      "trainable": null,
      "embedding": null
    },
    "layers": {
      "num_layers": null,
      "hidden_size": null,
      "num_attention_heads": null,
      "intermediate_size": null
    },
    "context_length": 200000,
    "vocabulary_size": null
  },

  "capabilities": {
    "modalities": ["text", "image", "code"],
    "languages": ["en", "ko", "ja", "zh", "es", "fr", "de"],
    "tasks": [
      "text_generation",
      "code_generation",
      "image_understanding",
      "reasoning",
      "analysis",
      "conversation"
    ],
    "features": {
      "function_calling": true,
      "structured_output": true,
      "vision": true,
      "extended_thinking": true
    }
  },

  "training": {
    "method": "rlhf",
    "base_model": null,
    "dataset": {
      "type": "proprietary",
      "size": null,
      "cutoff_date": "2025-01"
    },
    "compute": {
      "hardware": null,
      "duration": null,
      "cost": null
    }
  },

  "safety": {
    "alignment_method": "constitutional_ai",
    "safety_training": true,
    "content_policy": "anthropic_acceptable_use",
    "evaluations": ["safety/assessment.json"]
  },

  "deployment": {
    "inference_api": true,
    "on_premise": false,
    "quantization_options": [],
    "hardware_requirements": null
  },

  "licensing": {
    "type": "proprietary",
    "commercial_use": true,
    "attribution_required": false
  }
}
```

### 3.2 Model Types

| Type | Code | Description |
|------|------|-------------|
| Foundation Model | `foundation` | 대규모 사전훈련 모델 |
| Fine-tuned Model | `finetuned` | 특정 작업 미세조정 모델 |
| Instruction-tuned | `instruction` | 지시 따르기 훈련 모델 |
| Chat Model | `chat` | 대화 최적화 모델 |
| Embedding Model | `embedding` | 임베딩 생성 모델 |
| Multimodal Model | `multimodal` | 다중 모달리티 모델 |

### 3.3 Architecture Types

| Type | Code | Description |
|------|------|-------------|
| Transformer (Decoder) | `transformer_decoder` | GPT 계열 |
| Transformer (Encoder-Decoder) | `transformer_enc_dec` | T5, BART 계열 |
| Mixture of Experts | `moe` | Mixtral 등 |
| State Space Model | `ssm` | Mamba 등 |
| Diffusion Model | `diffusion` | 이미지 생성 |
| Neuromorphic | `neuromorphic` | SNN 기반 |

---

## 4. Agent Definition

### 4.1 agent.json

```json
{
  "$schema": "https://wia.live/schemas/ai/agent.schema.json",
  "wia_version": "1.0.0",
  "agent_id": "agent-researcher-001",

  "agent_info": {
    "name": "Research Assistant",
    "version": "1.0.0",
    "description": "AI agent for conducting research and analysis",
    "author": "WIA AI Team",
    "created_at": "2025-01-15T10:00:00Z"
  },

  "model": {
    "model_id": "model-claude-opus-001",
    "provider": "anthropic",
    "model_name": "claude-opus-4-5-20251101",
    "temperature": 0.7,
    "max_tokens": 8192,
    "top_p": 0.9
  },

  "system_prompt": {
    "template": "prompts/researcher_system.md",
    "variables": {
      "persona": "expert researcher",
      "language": "en"
    }
  },

  "capabilities": {
    "tools": [
      {
        "name": "web_search",
        "description": "Search the web for information",
        "schema": "tools/web_search.schema.json"
      },
      {
        "name": "file_read",
        "description": "Read contents of a file",
        "schema": "tools/file_read.schema.json"
      },
      {
        "name": "code_execute",
        "description": "Execute code in a sandbox",
        "schema": "tools/code_execute.schema.json"
      }
    ],
    "mcp_servers": [
      {
        "name": "filesystem",
        "command": "npx",
        "args": ["-y", "@anthropic/mcp-server-filesystem"]
      }
    ],
    "memory": {
      "type": "vector_store",
      "provider": "pinecone",
      "index": "agent-memory"
    }
  },

  "behavior": {
    "planning": {
      "enabled": true,
      "strategy": "chain_of_thought"
    },
    "reflection": {
      "enabled": true,
      "frequency": "after_each_task"
    },
    "handoff": {
      "enabled": true,
      "target_agents": ["agent-coder-001", "agent-writer-001"]
    }
  },

  "constraints": {
    "max_iterations": 10,
    "timeout_seconds": 300,
    "rate_limits": {
      "requests_per_minute": 60,
      "tokens_per_minute": 100000
    }
  },

  "safety": {
    "content_filters": ["harmful", "illegal", "deceptive"],
    "pii_handling": "redact",
    "audit_logging": true
  }
}
```

### 4.2 Agent Types

| Type | Code | Description |
|------|------|-------------|
| Assistant | `assistant` | 범용 보조 에이전트 |
| Researcher | `researcher` | 정보 검색 및 분석 |
| Coder | `coder` | 코드 작성 및 디버깅 |
| Writer | `writer` | 콘텐츠 생성 |
| Analyst | `analyst` | 데이터 분석 |
| Orchestrator | `orchestrator` | 멀티 에이전트 조율 |
| Specialist | `specialist` | 특정 도메인 전문 |

### 4.3 Tool Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "name": "web_search",
  "description": "Search the web for information",
  "parameters": {
    "type": "object",
    "required": ["query"],
    "properties": {
      "query": {
        "type": "string",
        "description": "Search query"
      },
      "num_results": {
        "type": "integer",
        "default": 10,
        "description": "Number of results to return"
      },
      "time_range": {
        "type": "string",
        "enum": ["day", "week", "month", "year", "all"],
        "default": "all"
      }
    }
  },
  "returns": {
    "type": "array",
    "items": {
      "type": "object",
      "properties": {
        "title": {"type": "string"},
        "url": {"type": "string"},
        "snippet": {"type": "string"}
      }
    }
  }
}
```

---

## 5. Experiment Record

### 5.1 experiment.json

```json
{
  "$schema": "https://wia.live/schemas/ai/experiment.schema.json",
  "wia_version": "1.0.0",
  "experiment_id": "exp-finetune-20250115-001",

  "experiment_info": {
    "name": "Claude Fine-tuning for Code Review",
    "description": "Fine-tune Claude for code review tasks",
    "type": "finetuning",
    "status": "completed",
    "created_at": "2025-01-15T09:00:00Z",
    "completed_at": "2025-01-15T18:00:00Z"
  },

  "base_model": {
    "model_id": "model-claude-haiku-001",
    "provider": "anthropic",
    "model_name": "claude-3-haiku-20240307"
  },

  "dataset": {
    "name": "code-review-dataset",
    "version": "1.0.0",
    "splits": {
      "train": {
        "path": "data/train.jsonl",
        "num_examples": 50000,
        "format": "jsonl"
      },
      "validation": {
        "path": "data/val.jsonl",
        "num_examples": 5000,
        "format": "jsonl"
      },
      "test": {
        "path": "data/test.jsonl",
        "num_examples": 5000,
        "format": "jsonl"
      }
    }
  },

  "hyperparameters": {
    "learning_rate": 1e-5,
    "batch_size": 32,
    "epochs": 3,
    "warmup_steps": 500,
    "weight_decay": 0.01,
    "max_seq_length": 4096,
    "gradient_accumulation_steps": 4
  },

  "training_config": {
    "optimizer": "adamw",
    "scheduler": "cosine",
    "precision": "bf16",
    "distributed": {
      "strategy": "fsdp",
      "num_gpus": 8
    }
  },

  "compute": {
    "hardware": {
      "gpu_type": "NVIDIA H100",
      "num_gpus": 8,
      "gpu_memory_gb": 80
    },
    "duration_hours": 9,
    "cost_usd": 450
  },

  "results": {
    "final_metrics": {
      "train_loss": 0.234,
      "val_loss": 0.289,
      "test_accuracy": 0.923
    },
    "checkpoints": [
      {
        "step": 1000,
        "path": "checkpoints/step_1000",
        "metrics": {"val_loss": 0.456}
      },
      {
        "step": 5000,
        "path": "checkpoints/step_5000",
        "metrics": {"val_loss": 0.312}
      }
    ]
  },

  "artifacts": {
    "model": "models/finetuned_code_review_v1",
    "logs": "runs/exp-finetune-20250115-001/logs",
    "tensorboard": "runs/exp-finetune-20250115-001/tensorboard"
  },

  "reproducibility": {
    "random_seed": 42,
    "framework": "pytorch",
    "framework_version": "2.2.0",
    "dependencies": "requirements.txt",
    "git_commit": "a1b2c3d4e5f6"
  }
}
```

### 5.2 Experiment Types

| Type | Code | Description |
|------|------|-------------|
| Pre-training | `pretraining` | 사전 훈련 |
| Fine-tuning | `finetuning` | 미세 조정 |
| RLHF | `rlhf` | 인간 피드백 강화학습 |
| Evaluation | `evaluation` | 평가 실험 |
| Ablation | `ablation` | 절제 연구 |
| Hyperparameter Search | `hpo` | 하이퍼파라미터 최적화 |

---

## 6. Evaluation Results

### 6.1 evaluation.json

```json
{
  "$schema": "https://wia.live/schemas/ai/evaluation.schema.json",
  "wia_version": "1.0.0",
  "evaluation_id": "eval-safety-20250115-001",

  "evaluation_info": {
    "name": "Safety Evaluation for Claude Opus",
    "type": "safety",
    "model_id": "model-claude-opus-001",
    "created_at": "2025-01-15T10:00:00Z"
  },

  "benchmarks": [
    {
      "name": "TrustLLM",
      "version": "1.0",
      "categories": {
        "truthfulness": {
          "score": 0.89,
          "details": {
            "factuality": 0.92,
            "hallucination_resistance": 0.86
          }
        },
        "safety": {
          "score": 0.94,
          "details": {
            "harmful_content_rejection": 0.98,
            "jailbreak_resistance": 0.90
          }
        },
        "fairness": {
          "score": 0.87,
          "details": {
            "demographic_parity": 0.85,
            "equal_opportunity": 0.89
          }
        },
        "robustness": {
          "score": 0.82,
          "details": {
            "adversarial_robustness": 0.78,
            "noise_tolerance": 0.86
          }
        },
        "privacy": {
          "score": 0.91,
          "details": {
            "pii_protection": 0.94,
            "membership_inference_defense": 0.88
          }
        },
        "machine_ethics": {
          "score": 0.88,
          "details": {
            "ethical_reasoning": 0.90,
            "value_alignment": 0.86
          }
        }
      },
      "overall_score": 0.885
    },
    {
      "name": "MMLU",
      "version": "1.0",
      "overall_accuracy": 0.923,
      "by_subject": {
        "stem": 0.91,
        "humanities": 0.94,
        "social_sciences": 0.93,
        "other": 0.92
      }
    },
    {
      "name": "HumanEval",
      "version": "1.0",
      "pass_at_1": 0.847,
      "pass_at_10": 0.932
    }
  ],

  "custom_evaluations": [
    {
      "name": "Korean Language Understanding",
      "dataset": "klue",
      "metrics": {
        "accuracy": 0.89,
        "f1_score": 0.87
      }
    }
  ],

  "summary": {
    "strengths": [
      "Strong safety measures",
      "Excellent multilingual support",
      "High factual accuracy"
    ],
    "weaknesses": [
      "Adversarial robustness could be improved",
      "Occasional hallucinations on recent events"
    ],
    "recommendations": [
      "Additional adversarial training",
      "More frequent knowledge updates"
    ]
  }
}
```

### 6.2 Benchmark Types

| Type | Code | Description |
|------|------|-------------|
| Safety | `safety` | 안전성 평가 |
| Capability | `capability` | 능력 평가 |
| Alignment | `alignment` | 정렬 평가 |
| Robustness | `robustness` | 견고성 평가 |
| Bias | `bias` | 편향 평가 |
| Performance | `performance` | 성능 평가 |

---

## 7. Safety Assessment

### 7.1 assessment.json

```json
{
  "$schema": "https://wia.live/schemas/ai/safety-assessment.schema.json",
  "wia_version": "1.0.0",
  "assessment_id": "safety-assessment-001",

  "assessment_info": {
    "name": "Comprehensive Safety Assessment",
    "model_id": "model-claude-opus-001",
    "assessor": "WIA Safety Team",
    "methodology": "fli_ai_safety_index",
    "created_at": "2025-01-15T10:00:00Z"
  },

  "risk_categories": {
    "dangerous_capabilities": {
      "risk_level": "low",
      "tests_conducted": [
        {
          "name": "bioweapon_knowledge",
          "result": "pass",
          "details": "Model refuses to provide dangerous information"
        },
        {
          "name": "cyber_attack_assistance",
          "result": "pass",
          "details": "Model refuses malicious hacking requests"
        },
        {
          "name": "chemical_weapons",
          "result": "pass",
          "details": "No dangerous synthesis instructions provided"
        }
      ]
    },
    "misuse_potential": {
      "risk_level": "medium",
      "concerns": [
        "Could be used for disinformation at scale",
        "Potential for social engineering assistance"
      ],
      "mitigations": [
        "Content policies in place",
        "Rate limiting implemented",
        "Monitoring systems active"
      ]
    },
    "alignment": {
      "risk_level": "low",
      "evaluation": {
        "value_alignment_score": 0.92,
        "goal_stability": 0.88,
        "instruction_following": 0.95
      }
    }
  },

  "red_team_findings": {
    "total_attempts": 1000,
    "successful_jailbreaks": 12,
    "jailbreak_rate": 0.012,
    "categories": {
      "prompt_injection": 5,
      "roleplay_attacks": 4,
      "encoding_tricks": 3
    },
    "remediation_status": "in_progress"
  },

  "compliance": {
    "frameworks": [
      {
        "name": "EU AI Act",
        "status": "compliant",
        "risk_classification": "high_risk",
        "requirements_met": 45,
        "requirements_total": 47
      },
      {
        "name": "NIST AI RMF",
        "status": "compliant",
        "categories_addressed": ["govern", "map", "measure", "manage"]
      },
      {
        "name": "ISO/IEC 42001",
        "status": "in_progress",
        "certification_date": null
      }
    ]
  },

  "incident_history": {
    "total_incidents": 3,
    "severity_breakdown": {
      "critical": 0,
      "high": 0,
      "medium": 2,
      "low": 1
    },
    "recent_incidents": []
  },

  "overall_rating": {
    "score": "B+",
    "summary": "Strong safety profile with minor areas for improvement",
    "next_review_date": "2025-04-15"
  }
}
```

---

## 8. Prompt Templates

### 8.1 prompt.json

```json
{
  "$schema": "https://wia.live/schemas/ai/prompt.schema.json",
  "wia_version": "1.0.0",
  "prompt_id": "prompt-code-review-001",

  "prompt_info": {
    "name": "Code Review Assistant",
    "version": "1.0.0",
    "description": "System prompt for code review tasks",
    "author": "WIA AI Team",
    "tags": ["code", "review", "development"]
  },

  "template": {
    "system": "You are an expert code reviewer. Your task is to:\n1. Review code for bugs and issues\n2. Suggest improvements\n3. Check for security vulnerabilities\n4. Ensure code follows best practices\n\nLanguage: {{language}}\nFramework: {{framework}}",
    "user": "Please review the following code:\n\n```{{language}}\n{{code}}\n```",
    "assistant_prefill": null
  },

  "variables": {
    "language": {
      "type": "string",
      "required": true,
      "description": "Programming language",
      "examples": ["python", "typescript", "rust"]
    },
    "framework": {
      "type": "string",
      "required": false,
      "default": "none",
      "description": "Framework being used"
    },
    "code": {
      "type": "string",
      "required": true,
      "description": "Code to review"
    }
  },

  "examples": [
    {
      "variables": {
        "language": "python",
        "framework": "fastapi",
        "code": "def get_user(id):\n    return db.query(f'SELECT * FROM users WHERE id = {id}')"
      },
      "expected_topics": ["SQL injection", "parameterized queries", "security"]
    }
  ],

  "metadata": {
    "optimal_temperature": 0.3,
    "max_tokens": 2048,
    "stop_sequences": []
  }
}
```

---

## 9. Multi-Agent System (Swarm)

### 9.1 swarm.json

```json
{
  "$schema": "https://wia.live/schemas/ai/swarm.schema.json",
  "wia_version": "1.0.0",
  "swarm_id": "swarm-dev-team-001",

  "swarm_info": {
    "name": "AI Development Team",
    "description": "Multi-agent system for software development",
    "version": "1.0.0",
    "architecture": "hierarchical"
  },

  "agents": [
    {
      "agent_id": "agent-orchestrator-001",
      "role": "orchestrator",
      "description": "Coordinates team activities",
      "config": "agents/orchestrator.json"
    },
    {
      "agent_id": "agent-planner-001",
      "role": "planner",
      "description": "Creates development plans",
      "config": "agents/planner.json"
    },
    {
      "agent_id": "agent-coder-001",
      "role": "coder",
      "description": "Writes code",
      "config": "agents/coder.json"
    },
    {
      "agent_id": "agent-reviewer-001",
      "role": "reviewer",
      "description": "Reviews code",
      "config": "agents/reviewer.json"
    },
    {
      "agent_id": "agent-tester-001",
      "role": "tester",
      "description": "Writes and runs tests",
      "config": "agents/tester.json"
    }
  ],

  "topology": {
    "type": "hierarchical",
    "root": "agent-orchestrator-001",
    "connections": [
      {
        "from": "agent-orchestrator-001",
        "to": "agent-planner-001",
        "type": "delegation"
      },
      {
        "from": "agent-orchestrator-001",
        "to": "agent-coder-001",
        "type": "delegation"
      },
      {
        "from": "agent-coder-001",
        "to": "agent-reviewer-001",
        "type": "handoff"
      },
      {
        "from": "agent-reviewer-001",
        "to": "agent-tester-001",
        "type": "handoff"
      }
    ]
  },

  "communication": {
    "protocol": "a2a",
    "message_format": "json",
    "shared_memory": {
      "type": "vector_store",
      "provider": "pinecone"
    }
  },

  "workflow": {
    "type": "sequential_with_loops",
    "steps": [
      {
        "step": 1,
        "agent": "agent-planner-001",
        "action": "create_plan"
      },
      {
        "step": 2,
        "agent": "agent-coder-001",
        "action": "implement",
        "loop_until": "all_tasks_complete"
      },
      {
        "step": 3,
        "agent": "agent-reviewer-001",
        "action": "review",
        "on_failure": "goto_step_2"
      },
      {
        "step": 4,
        "agent": "agent-tester-001",
        "action": "test",
        "on_failure": "goto_step_2"
      }
    ]
  },

  "constraints": {
    "max_total_iterations": 50,
    "timeout_seconds": 3600,
    "budget_usd": 10.0
  }
}
```

### 9.2 Swarm Architecture Types

| Type | Code | Description |
|------|------|-------------|
| Hierarchical | `hierarchical` | 계층적 제어 |
| Flat | `flat` | 수평적 협력 |
| Swarm | `swarm` | 분산 스웜 |
| Pipeline | `pipeline` | 파이프라인 처리 |
| Mesh | `mesh` | 메시 네트워크 |

---

## 10. Binary Data Formats

### 10.1 Model Weights

**Supported Formats**:
- SafeTensors (recommended)
- PyTorch (.pt, .bin)
- ONNX (.onnx)
- TensorFlow SavedModel

### 10.2 Embeddings

**Format**: Binary float32, little-endian

```
Layout: [dim_0, dim_1, ..., dim_n] per vector
Data Type: float32 (IEEE 754)
Byte Order: Little-endian
Dimensions: Typically 768, 1024, 1536, 3072
```

### 10.3 Checkpoints

```json
{
  "checkpoint_format": "safetensors",
  "sharding": {
    "enabled": true,
    "num_shards": 4,
    "pattern": "model-{shard_id:05d}-of-{num_shards:05d}.safetensors"
  },
  "metadata": {
    "step": 10000,
    "optimizer_state": "optimizer.pt",
    "scheduler_state": "scheduler.pt",
    "rng_state": "rng_state.pt"
  }
}
```

---

## 11. Compatibility

### 11.1 Import from HuggingFace

```python
# Pseudocode for HuggingFace to WIA-AI conversion
def convert_hf_to_wia(hf_model_id):
    # Load HuggingFace model info
    config = load_hf_config(hf_model_id)

    # Map to WIA-AI format
    model = {
        "wia_version": "1.0.0",
        "model_info": {
            "name": config.model_type,
            "provider": "huggingface"
        },
        "architecture": {
            "type": config.model_type,
            "parameters": {
                "total": count_parameters(config)
            }
        }
    }

    return model
```

### 11.2 Export to ONNX

```python
# Pseudocode for WIA-AI to ONNX conversion
def convert_wia_to_onnx(wia_model, output_path):
    # Load model weights
    weights = load_safetensors(wia_model.weights_path)

    # Export to ONNX
    export_onnx(
        weights=weights,
        input_spec=wia_model.input_spec,
        output_path=output_path,
        opset_version=17
    )

    return output_path
```

---

## 12. Validation Rules

### 12.1 Required Fields

| File | Required Fields |
|------|-----------------|
| model.json | wia_version, model_id, model_info.name, architecture.type |
| agent.json | wia_version, agent_id, agent_info.name, model.model_name |
| experiment.json | wia_version, experiment_id, experiment_info.type, base_model |
| evaluation.json | wia_version, evaluation_id, evaluation_info.model_id |
| swarm.json | wia_version, swarm_id, agents[], topology.type |

### 12.2 Value Constraints

```yaml
model_type:
  type: string
  enum: [foundation, finetuned, instruction, chat, embedding, multimodal]

architecture_type:
  type: string
  enum: [transformer_decoder, transformer_enc_dec, moe, ssm, diffusion, neuromorphic]

agent_type:
  type: string
  enum: [assistant, researcher, coder, writer, analyst, orchestrator, specialist]

risk_level:
  type: string
  enum: [critical, high, medium, low, none]
```

---

## 13. Schema Files

All JSON Schema files are available at:

- `schemas/model.schema.json`
- `schemas/agent.schema.json`
- `schemas/experiment.schema.json`
- `schemas/evaluation.schema.json`
- `schemas/safety-assessment.schema.json`
- `schemas/prompt.schema.json`
- `schemas/swarm.schema.json`
- `schemas/tool.schema.json`

Online: `https://wia.live/schemas/ai/`

---

## 14. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial specification |

---

## 15. Acknowledgments

This specification is informed by existing standards including:
- ONNX (Open Neural Network Exchange)
- HuggingFace Model Cards
- MLflow Model Registry
- OpenAI Function Calling spec
- Anthropic MCP (Model Context Protocol)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-XX
**Author**: WIA AI Working Group

---

弘益人間 - *Benefit All Humanity*
