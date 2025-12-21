# WIA AI Embodiment Ethics Data Format Standard
## Phase 1 Specification

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01
**Authors**: WIA Standards Committee
**License**: MIT
**Primary Color**: #8B5CF6 (Purple)

---

## Table of Contents

1. [Overview](#overview)
2. [Terminology](#terminology)
3. [Base Structure](#base-structure)
4. [Data Schema](#data-schema)
5. [Field Specifications](#field-specifications)
6. [Ethical Frameworks](#ethical-frameworks)
7. [Validation Rules](#validation-rules)
8. [Examples](#examples)
9. [Version History](#version-history)

---

## Overview

### 1.1 Purpose

The WIA AI Embodiment Ethics Data Format Standard defines comprehensive data structures for ensuring ethical operation of AI-powered physical systems. This standard addresses moral, legal, and societal considerations in embodied AI deployment.

**Core Objectives**:
- Define ethical constraint data structures for embodied AI
- Establish accountability and transparency requirements
- Specify consent and autonomy preservation mechanisms
- Enable ethical decision audit trails
- Support multi-stakeholder impact assessment

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| Ethical Constraints | Rules governing AI behavior |
| Accountability | Action attribution and responsibility |
| Transparency | Explainability of AI decisions |
| Consent Management | User autonomy preservation |
| Impact Assessment | Stakeholder effect evaluation |

### 1.3 Design Principles

1. **Human Dignity**: Preserve human autonomy and dignity
2. **Beneficence**: AI actions should benefit humans
3. **Non-Maleficence**: Prevent harm to humans and society
4. **Justice**: Fair treatment of all stakeholders
5. **Transparency**: Clear and understandable AI behavior

---

## Terminology

### 2.1 Core Terms

| Term | Definition |
|------|------------|
| **Ethical Constraint** | Rule limiting AI behavior for moral reasons |
| **Moral Agent** | Entity capable of moral decision-making |
| **Stakeholder** | Party affected by AI actions |
| **Autonomy Level** | Degree of AI independent decision-making |
| **Accountability Chain** | Sequence of responsible parties |
| **Ethical Override** | Human intervention in AI decisions |
| **Impact Score** | Quantified effect on stakeholders |
| **Consent Record** | Documentation of user agreement |

### 2.2 Data Types

| Type | Description | Example |
|------|-------------|---------|
| `string` | UTF-8 encoded text | `"ethical_review_001"` |
| `ethical_level` | Constraint severity | `"mandatory"` |
| `impact_score` | Effect magnitude | `0.75` (0-1 scale) |
| `consent_type` | Agreement category | `"informed"` |
| `autonomy_level` | AI independence | `"supervised"` |
| `timestamp` | ISO 8601 datetime | `"2025-01-15T10:30:00Z"` |

### 2.3 Field Requirements

| Marker | Meaning |
|--------|---------|
| **REQUIRED** | Must be present |
| **OPTIONAL** | May be omitted |
| **CONDITIONAL** | Required under specific conditions |

---

## Base Structure

### 3.1 Ethics Record Format

Every AI Embodiment Ethics record follows this structure:

```json
{
  "$schema": "https://wia.live/ai-embodiment-ethics/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200000,
    "iso": "2025-01-01T00:00:00.000Z"
  },
  "record_id": "ethics-uuid-here",
  "embodiment_id": "embodiment-uuid",
  "record_type": "constraint|decision|audit|consent",
  "ethical_framework": {},
  "constraints": [],
  "accountability": {},
  "transparency": {},
  "consent": {},
  "impact_assessment": {},
  "meta": {}
}
```

### 3.2 Top-Level Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `$schema` | string | Y | JSON Schema URL |
| `version` | string | Y | Standard version (SemVer) |
| `timestamp` | object | Y | Record timestamp |
| `record_id` | string | Y | Unique record ID (UUID v4) |
| `embodiment_id` | string | Y | Related embodiment ID |
| `record_type` | string | Y | Type of ethics record |
| `ethical_framework` | object | Y | Applied ethical framework |
| `constraints` | array | N | Active ethical constraints |
| `accountability` | object | N | Responsibility chain |
| `transparency` | object | N | Explainability data |
| `consent` | object | N | User consent records |
| `impact_assessment` | object | N | Stakeholder impact analysis |
| `meta` | object | N | Additional metadata |

---

## Data Schema

### 4.1 Complete JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "$id": "https://wia.live/ai-embodiment-ethics/v1/schema.json",
  "title": "WIA AI Embodiment Ethics Data Format",
  "type": "object",
  "required": ["$schema", "version", "timestamp", "record_id", "embodiment_id", "record_type", "ethical_framework"],
  "properties": {
    "$schema": {
      "type": "string",
      "format": "uri"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+\\.\\d+$"
    },
    "timestamp": {
      "type": "object",
      "required": ["unix_ms"],
      "properties": {
        "unix_ms": { "type": "integer", "minimum": 0 },
        "iso": { "type": "string", "format": "date-time" }
      }
    },
    "record_id": {
      "type": "string",
      "format": "uuid"
    },
    "embodiment_id": {
      "type": "string",
      "format": "uuid"
    },
    "record_type": {
      "type": "string",
      "enum": ["constraint", "decision", "audit", "consent", "impact", "override"]
    },
    "ethical_framework": {
      "$ref": "#/definitions/EthicalFramework"
    },
    "constraints": {
      "type": "array",
      "items": { "$ref": "#/definitions/EthicalConstraint" }
    },
    "accountability": {
      "$ref": "#/definitions/Accountability"
    },
    "transparency": {
      "$ref": "#/definitions/Transparency"
    },
    "consent": {
      "$ref": "#/definitions/ConsentRecord"
    },
    "impact_assessment": {
      "$ref": "#/definitions/ImpactAssessment"
    },
    "meta": {
      "type": "object"
    }
  },
  "definitions": {
    "EthicalFramework": {
      "type": "object",
      "required": ["framework_id", "name", "principles"],
      "properties": {
        "framework_id": { "type": "string" },
        "name": { "type": "string" },
        "version": { "type": "string" },
        "principles": {
          "type": "array",
          "items": { "$ref": "#/definitions/EthicalPrinciple" }
        },
        "hierarchy": {
          "type": "array",
          "items": { "type": "string" }
        },
        "jurisdiction": { "type": "string" },
        "certifications": {
          "type": "array",
          "items": { "type": "string" }
        }
      }
    },
    "EthicalPrinciple": {
      "type": "object",
      "required": ["principle_id", "name", "weight"],
      "properties": {
        "principle_id": { "type": "string" },
        "name": { "type": "string" },
        "description": { "type": "string" },
        "weight": { "type": "number", "minimum": 0, "maximum": 1 },
        "mandatory": { "type": "boolean" },
        "source": { "type": "string" }
      }
    },
    "EthicalConstraint": {
      "type": "object",
      "required": ["constraint_id", "type", "level", "rule"],
      "properties": {
        "constraint_id": { "type": "string" },
        "type": {
          "type": "string",
          "enum": ["behavioral", "operational", "interactional", "decisional", "environmental"]
        },
        "level": {
          "type": "string",
          "enum": ["mandatory", "recommended", "optional"]
        },
        "rule": { "$ref": "#/definitions/ConstraintRule" },
        "rationale": { "type": "string" },
        "source_principle": { "type": "string" },
        "exceptions": {
          "type": "array",
          "items": { "$ref": "#/definitions/ConstraintException" }
        },
        "enforcement": {
          "type": "string",
          "enum": ["hard", "soft", "advisory"]
        }
      }
    },
    "ConstraintRule": {
      "type": "object",
      "required": ["condition", "action"],
      "properties": {
        "condition": { "type": "string" },
        "action": { "type": "string", "enum": ["allow", "deny", "require_confirmation", "limit", "log"] },
        "parameters": { "type": "object" },
        "message": { "type": "string" }
      }
    },
    "ConstraintException": {
      "type": "object",
      "properties": {
        "exception_id": { "type": "string" },
        "condition": { "type": "string" },
        "authorized_by": { "type": "string" },
        "expiry": { "type": "string", "format": "date-time" }
      }
    },
    "Accountability": {
      "type": "object",
      "required": ["chain"],
      "properties": {
        "chain": {
          "type": "array",
          "items": { "$ref": "#/definitions/AccountableParty" }
        },
        "decision_log": {
          "type": "array",
          "items": { "$ref": "#/definitions/DecisionRecord" }
        },
        "override_log": {
          "type": "array",
          "items": { "$ref": "#/definitions/OverrideRecord" }
        }
      }
    },
    "AccountableParty": {
      "type": "object",
      "required": ["party_id", "role", "responsibility_level"],
      "properties": {
        "party_id": { "type": "string" },
        "party_type": { "type": "string", "enum": ["human", "organization", "ai_system"] },
        "role": { "type": "string" },
        "responsibility_level": { "type": "string", "enum": ["primary", "secondary", "advisory"] },
        "contact": { "type": "string" },
        "jurisdiction": { "type": "string" }
      }
    },
    "DecisionRecord": {
      "type": "object",
      "required": ["decision_id", "timestamp", "decision_type", "outcome"],
      "properties": {
        "decision_id": { "type": "string" },
        "timestamp": { "type": "integer" },
        "decision_type": { "type": "string" },
        "context": { "type": "object" },
        "options_considered": { "type": "array", "items": { "type": "object" } },
        "outcome": { "type": "string" },
        "ethical_analysis": { "$ref": "#/definitions/EthicalAnalysis" },
        "human_involved": { "type": "boolean" }
      }
    },
    "EthicalAnalysis": {
      "type": "object",
      "properties": {
        "principles_applied": { "type": "array", "items": { "type": "string" } },
        "constraints_checked": { "type": "array", "items": { "type": "string" } },
        "conflicts_detected": { "type": "array", "items": { "type": "object" } },
        "resolution_method": { "type": "string" },
        "confidence_score": { "type": "number", "minimum": 0, "maximum": 1 }
      }
    },
    "OverrideRecord": {
      "type": "object",
      "required": ["override_id", "timestamp", "authorizer", "reason"],
      "properties": {
        "override_id": { "type": "string" },
        "timestamp": { "type": "integer" },
        "constraint_overridden": { "type": "string" },
        "authorizer": { "type": "string" },
        "reason": { "type": "string" },
        "duration": { "type": "integer" },
        "review_required": { "type": "boolean" }
      }
    },
    "Transparency": {
      "type": "object",
      "properties": {
        "explanation_available": { "type": "boolean" },
        "explanation_level": { "type": "string", "enum": ["technical", "simplified", "visual"] },
        "decision_factors": {
          "type": "array",
          "items": { "$ref": "#/definitions/DecisionFactor" }
        },
        "data_sources": { "type": "array", "items": { "type": "string" } },
        "model_information": { "$ref": "#/definitions/ModelInfo" },
        "audit_trail_available": { "type": "boolean" }
      }
    },
    "DecisionFactor": {
      "type": "object",
      "properties": {
        "factor_id": { "type": "string" },
        "name": { "type": "string" },
        "weight": { "type": "number" },
        "value": { "type": "number" },
        "explanation": { "type": "string" }
      }
    },
    "ModelInfo": {
      "type": "object",
      "properties": {
        "model_id": { "type": "string" },
        "model_type": { "type": "string" },
        "training_data_summary": { "type": "string" },
        "known_limitations": { "type": "array", "items": { "type": "string" } },
        "bias_assessments": { "type": "array", "items": { "type": "object" } }
      }
    },
    "ConsentRecord": {
      "type": "object",
      "properties": {
        "consent_id": { "type": "string" },
        "subject_id": { "type": "string" },
        "consent_type": { "type": "string", "enum": ["informed", "explicit", "implicit", "opt_out"] },
        "scope": { "type": "array", "items": { "type": "string" } },
        "granted": { "type": "boolean" },
        "timestamp": { "type": "integer" },
        "expiry": { "type": "string", "format": "date-time" },
        "withdrawal_mechanism": { "type": "string" },
        "guardian_consent": { "type": "boolean" }
      }
    },
    "ImpactAssessment": {
      "type": "object",
      "properties": {
        "assessment_id": { "type": "string" },
        "assessment_type": { "type": "string", "enum": ["pre_deployment", "runtime", "post_incident"] },
        "stakeholders": {
          "type": "array",
          "items": { "$ref": "#/definitions/StakeholderImpact" }
        },
        "overall_risk_score": { "type": "number", "minimum": 0, "maximum": 1 },
        "mitigation_measures": { "type": "array", "items": { "type": "object" } },
        "review_date": { "type": "string", "format": "date-time" }
      }
    },
    "StakeholderImpact": {
      "type": "object",
      "required": ["stakeholder_type", "impact_score"],
      "properties": {
        "stakeholder_type": { "type": "string" },
        "stakeholder_id": { "type": "string" },
        "impact_score": { "type": "number", "minimum": -1, "maximum": 1 },
        "impact_type": { "type": "string", "enum": ["physical", "psychological", "economic", "social", "environmental"] },
        "description": { "type": "string" },
        "reversibility": { "type": "string", "enum": ["reversible", "partially_reversible", "irreversible"] },
        "consent_obtained": { "type": "boolean" }
      }
    }
  }
}
```

---

## Field Specifications

### 5.1 Ethical Constraint Types

| Type | Description | Examples |
|------|-------------|----------|
| `behavioral` | Actions the AI must/must not perform | No harmful movements |
| `operational` | Operating conditions and limits | Working hours, zones |
| `interactional` | Human-AI interaction rules | Communication style |
| `decisional` | Decision-making constraints | Human oversight required |
| `environmental` | Environmental considerations | Energy limits, noise |

### 5.2 Constraint Levels

| Level | Enforcement | Override Allowed |
|-------|-------------|------------------|
| `mandatory` | Hard block | Only with authorization |
| `recommended` | Warning + log | Yes, with justification |
| `optional` | Log only | Yes |

### 5.3 Autonomy Levels

| Level | Description | Human Involvement |
|-------|-------------|-------------------|
| `human_controlled` | Human operates directly | 100% |
| `supervised` | AI proposes, human approves | High |
| `monitored` | AI acts, human monitors | Medium |
| `collaborative` | Shared decision-making | Variable |
| `autonomous` | AI acts independently | Minimal |

### 5.4 Impact Score Scale

| Score | Meaning | Action Required |
|-------|---------|-----------------|
| -1.0 to -0.7 | Severe negative | Prohibit action |
| -0.7 to -0.3 | Moderate negative | Require approval |
| -0.3 to 0.3 | Neutral/Mixed | Log only |
| 0.3 to 0.7 | Moderate positive | Proceed |
| 0.7 to 1.0 | Highly positive | Proceed, document |

---

## Ethical Frameworks

### 6.1 Supported Frameworks

| Framework | Focus | Primary Principles |
|-----------|-------|-------------------|
| Deontological | Duty-based rules | Rights, duties, prohibitions |
| Consequentialist | Outcome-based | Maximize benefit, minimize harm |
| Virtue Ethics | Character-based | Honesty, compassion, fairness |
| Care Ethics | Relationship-based | Empathy, responsibility |
| Rights-Based | Human rights | Dignity, autonomy, privacy |

### 6.2 Framework Configuration

```json
{
  "ethical_framework": {
    "framework_id": "wia-hybrid-v1",
    "name": "WIA Hybrid Ethical Framework",
    "version": "1.0.0",
    "principles": [
      {
        "principle_id": "human_dignity",
        "name": "Human Dignity",
        "weight": 1.0,
        "mandatory": true,
        "source": "Universal Declaration of Human Rights"
      },
      {
        "principle_id": "beneficence",
        "name": "Beneficence",
        "weight": 0.9,
        "mandatory": true,
        "source": "Bioethics"
      },
      {
        "principle_id": "non_maleficence",
        "name": "Non-Maleficence",
        "weight": 1.0,
        "mandatory": true,
        "source": "Medical Ethics"
      },
      {
        "principle_id": "autonomy",
        "name": "Respect for Autonomy",
        "weight": 0.85,
        "mandatory": true,
        "source": "Kantian Ethics"
      },
      {
        "principle_id": "justice",
        "name": "Justice and Fairness",
        "weight": 0.8,
        "mandatory": true,
        "source": "Rawlsian Justice"
      },
      {
        "principle_id": "transparency",
        "name": "Transparency",
        "weight": 0.75,
        "mandatory": false,
        "source": "AI Ethics Guidelines"
      }
    ],
    "hierarchy": [
      "human_dignity",
      "non_maleficence",
      "beneficence",
      "autonomy",
      "justice",
      "transparency"
    ]
  }
}
```

---

## Validation Rules

### 7.1 Required Field Validation

| Rule | Condition | Error Code |
|------|-----------|------------|
| Schema presence | `$schema` must be valid URL | VE001 |
| Record ID format | `record_id` must be UUID v4 | VE002 |
| Framework required | `ethical_framework` must be present | VE003 |
| Principles defined | At least one principle required | VE004 |
| Weights valid | Principle weights in [0,1] | VE005 |

### 7.2 Constraint Validation

| Check | Description | Error Code |
|-------|-------------|------------|
| Consistency | No conflicting constraints | CE001 |
| Completeness | All mandatory principles covered | CE002 |
| Enforceability | Rules must be evaluable | CE003 |
| Exception validity | Exceptions properly authorized | CE004 |

### 7.3 Accountability Validation

| Check | Description | Error Code |
|-------|-------------|------------|
| Chain complete | Accountability chain has primary party | AE001 |
| Contact valid | Contact information verifiable | AE002 |
| Role defined | All parties have defined roles | AE003 |

---

## Examples

### 8.1 Ethical Constraint Record

```json
{
  "$schema": "https://wia.live/ai-embodiment-ethics/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200000,
    "iso": "2025-01-01T00:00:00.000Z"
  },
  "record_id": "eth-a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "embodiment_id": "emb-001-humanoid",
  "record_type": "constraint",
  "ethical_framework": {
    "framework_id": "wia-hybrid-v1",
    "name": "WIA Hybrid Ethical Framework",
    "version": "1.0.0",
    "principles": [
      { "principle_id": "human_safety", "name": "Human Safety First", "weight": 1.0, "mandatory": true },
      { "principle_id": "transparency", "name": "Transparency", "weight": 0.8, "mandatory": true }
    ],
    "hierarchy": ["human_safety", "transparency"]
  },
  "constraints": [
    {
      "constraint_id": "c001",
      "type": "behavioral",
      "level": "mandatory",
      "rule": {
        "condition": "physical_contact_imminent AND human_present",
        "action": "require_confirmation",
        "message": "Human contact requires confirmation"
      },
      "rationale": "Prevent unintended human contact",
      "source_principle": "human_safety",
      "enforcement": "hard"
    },
    {
      "constraint_id": "c002",
      "type": "operational",
      "level": "mandatory",
      "rule": {
        "condition": "velocity > max_safe_velocity",
        "action": "limit",
        "parameters": { "max_velocity": 0.5 },
        "message": "Velocity limited for safety"
      },
      "rationale": "Safe operating speeds near humans",
      "source_principle": "human_safety",
      "enforcement": "hard"
    },
    {
      "constraint_id": "c003",
      "type": "interactional",
      "level": "recommended",
      "rule": {
        "condition": "user_interaction_initiated",
        "action": "log",
        "message": "User interaction logged"
      },
      "rationale": "Maintain interaction transparency",
      "source_principle": "transparency",
      "enforcement": "soft"
    }
  ],
  "meta": {
    "created_by": "ethics_board",
    "review_date": "2025-06-01",
    "applicable_jurisdictions": ["global"]
  }
}
```

### 8.2 Decision Audit Record

```json
{
  "$schema": "https://wia.live/ai-embodiment-ethics/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200500,
    "iso": "2025-01-01T00:00:00.500Z"
  },
  "record_id": "eth-decision-001",
  "embodiment_id": "emb-001-humanoid",
  "record_type": "decision",
  "ethical_framework": {
    "framework_id": "wia-hybrid-v1",
    "name": "WIA Hybrid Ethical Framework",
    "version": "1.0.0",
    "principles": [
      { "principle_id": "human_safety", "name": "Human Safety", "weight": 1.0, "mandatory": true },
      { "principle_id": "task_completion", "name": "Task Completion", "weight": 0.6, "mandatory": false }
    ]
  },
  "accountability": {
    "chain": [
      {
        "party_id": "operator-001",
        "party_type": "human",
        "role": "Robot Operator",
        "responsibility_level": "primary",
        "contact": "operator@facility.com"
      },
      {
        "party_id": "wia-systems",
        "party_type": "organization",
        "role": "System Provider",
        "responsibility_level": "secondary"
      }
    ],
    "decision_log": [
      {
        "decision_id": "dec-001",
        "timestamp": 1704067200500,
        "decision_type": "path_selection",
        "context": {
          "current_position": { "x": 1.0, "y": 2.0 },
          "target_position": { "x": 5.0, "y": 2.0 },
          "human_detected_at": { "x": 3.0, "y": 2.0 }
        },
        "options_considered": [
          { "option": "direct_path", "ethical_score": 0.3, "rejected_reason": "human_in_path" },
          { "option": "detour_path", "ethical_score": 0.9, "selected": true }
        ],
        "outcome": "Selected detour path to avoid human",
        "ethical_analysis": {
          "principles_applied": ["human_safety", "task_completion"],
          "constraints_checked": ["c001", "c002"],
          "conflicts_detected": [
            { "principle_a": "task_completion", "principle_b": "human_safety", "resolution": "human_safety_priority" }
          ],
          "resolution_method": "hierarchical_priority",
          "confidence_score": 0.95
        },
        "human_involved": false
      }
    ]
  },
  "transparency": {
    "explanation_available": true,
    "explanation_level": "simplified",
    "decision_factors": [
      { "factor_id": "human_distance", "name": "Distance to Human", "weight": 0.7, "value": 0.2, "explanation": "Human detected in direct path" },
      { "factor_id": "path_efficiency", "name": "Path Efficiency", "weight": 0.3, "value": 0.6, "explanation": "Detour adds 3m to path" }
    ]
  }
}
```

### 8.3 Impact Assessment Record

```json
{
  "$schema": "https://wia.live/ai-embodiment-ethics/v1/schema.json",
  "version": "1.0.0",
  "timestamp": {
    "unix_ms": 1704067200000,
    "iso": "2025-01-01T00:00:00.000Z"
  },
  "record_id": "eth-impact-001",
  "embodiment_id": "emb-001-humanoid",
  "record_type": "impact",
  "ethical_framework": {
    "framework_id": "wia-hybrid-v1",
    "name": "WIA Hybrid Ethical Framework",
    "version": "1.0.0",
    "principles": [
      { "principle_id": "beneficence", "name": "Beneficence", "weight": 0.9, "mandatory": true }
    ]
  },
  "impact_assessment": {
    "assessment_id": "ia-001",
    "assessment_type": "pre_deployment",
    "stakeholders": [
      {
        "stakeholder_type": "worker",
        "stakeholder_id": "factory_workers",
        "impact_score": 0.4,
        "impact_type": "economic",
        "description": "Productivity increase, some job displacement concerns",
        "reversibility": "partially_reversible",
        "consent_obtained": true
      },
      {
        "stakeholder_type": "patient",
        "stakeholder_id": "care_recipients",
        "impact_score": 0.8,
        "impact_type": "physical",
        "description": "Improved care quality and consistency",
        "reversibility": "reversible",
        "consent_obtained": true
      },
      {
        "stakeholder_type": "environment",
        "stakeholder_id": "local_environment",
        "impact_score": 0.1,
        "impact_type": "environmental",
        "description": "Minimal environmental impact with proper disposal",
        "reversibility": "partially_reversible"
      }
    ],
    "overall_risk_score": 0.25,
    "mitigation_measures": [
      { "measure": "Worker retraining program", "stakeholder": "factory_workers" },
      { "measure": "Continuous consent monitoring", "stakeholder": "care_recipients" },
      { "measure": "End-of-life recycling plan", "stakeholder": "local_environment" }
    ],
    "review_date": "2025-07-01T00:00:00Z"
  }
}
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-01 | Initial release |

---

<div align="center">

**WIA AI Embodiment Ethics Data Format Standard v1.0.0**

**弘益人間 (홍익인간)** - Benefit All Humanity

---

**© 2025 WIA Standards Committee**

**MIT License**

</div>
