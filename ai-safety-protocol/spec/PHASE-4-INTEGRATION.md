# WIA-AI-010 Phase 4: Integration & Compliance
## System Integration and Regulatory Compliance

**Version:** 1.0
**Status:** Official Standard
**Last Updated:** 2025-12-25

---

## Overview

This specification defines integration patterns for incorporating WIA-AI-010 safety protocols into existing AI systems and ensuring ongoing regulatory compliance. It provides practical guidance for real-world deployment across diverse technology stacks and regulatory environments.

**弘益人間** (Benefit All Humanity) - Through seamless integration, we make AI safety accessible to all developers.

---

## 1. Integration Patterns

### 1.1 Proxy Pattern

Intercept all AI API calls through a safety proxy:

```typescript
// Safety Proxy Architecture
import { SafetyProtocol } from '@wia/ai-safety-protocol';

class SafetyProxy {
    private safety: SafetyProtocol;
    private upstreamAPI: AIClient;

    async proxy(request: AIRequest): Promise<AIResponse> {
        // Pre-processing safety checks
        const inputCheck = await this.safety.validateInput(request.input);
        if (!inputCheck.safe) {
            return this.buildBlockedResponse(inputCheck.reason);
        }

        // Call upstream AI
        const response = await this.upstreamAPI.generate(
            inputCheck.sanitizedInput
        );

        // Post-processing safety checks
        const outputCheck = await this.safety.validateOutput(response.output);
        if (!outputCheck.safe) {
            return this.buildFilteredResponse(outputCheck.reason);
        }

        // Log for monitoring
        await this.safety.logInteraction({
            input: request.input,
            output: response.output,
            safetyChecks: {input: inputCheck, output: outputCheck}
        });

        return response;
    }
}
```

**Advantages:**
- No changes to upstream AI system
- Centralized safety enforcement
- Easy to enable/disable
- Simple monitoring

**Disadvantages:**
- Additional latency
- Single point of failure
- Limited access to model internals

### 1.2 Middleware Pattern

Integrate as middleware in existing application framework:

```python
# FastAPI Middleware Example
from fastapi import FastAPI, Request
from wia_safety import SafetyMiddleware

app = FastAPI()

app.add_middleware(
    SafetyMiddleware,
    api_key=os.environ['WIA_API_KEY'],
    strictness='balanced',
    config={
        'input_filtering': True,
        'output_filtering': True,
        'rate_limiting': True,
        'monitoring': True
    }
)

@app.post("/ai/chat")
async def chat(request: ChatRequest):
    # Safety checks handled by middleware
    response = await ai_model.generate(request.message)
    return {"response": response}
```

### 1.3 SDK Wrapper Pattern

Wrap AI SDK with safety layer:

```typescript
import { OpenAI } from 'openai';
import { SafetyWrapper } from '@wia/ai-safety-protocol';

const openai = new OpenAI({apiKey: process.env.OPENAI_API_KEY});
const safeAI = SafetyWrapper.wrap(openai, {
    inputGuardrails: ['prompt-injection', 'toxicity'],
    outputGuardrails: ['content-filter', 'pii-redaction'],
    monitoring: {enabled: true, sampleRate: 0.1}
});

// Use exactly like OpenAI SDK, but with safety
const completion = await safeAI.chat.completions.create({
    model: "gpt-4",
    messages: [{role: "user", content: "Hello!"}]
});
```

### 1.4 Sidecar Pattern

Deploy safety service as sidecar container:

```yaml
# Kubernetes Deployment
apiVersion: v1
kind: Pod
metadata:
  name: ai-service
spec:
  containers:
  - name: ai-model
    image: mycompany/ai-model:latest
    ports:
    - containerPort: 8080

  - name: safety-sidecar
    image: wia/safety-protocol:latest
    env:
    - name: UPSTREAM_URL
      value: "http://localhost:8080"
    - name: WIA_API_KEY
      valueFrom:
        secretKeyRef:
          name: wia-credentials
          key: api-key
    ports:
    - containerPort: 9000

  # Service routes through sidecar
  - name: nginx-proxy
    image: nginx:latest
    ports:
    - containerPort: 80
```

---

## 2. Framework Integration

### 2.1 LangChain Integration

```python
from langchain.llms import OpenAI
from wia_safety.langchain import SafetyCallbackHandler

# Add safety callbacks to LangChain
safety_handler = SafetyCallbackHandler(
    api_key=os.environ['WIA_API_KEY'],
    config={'strictness': 'balanced'}
)

llm = OpenAI(callbacks=[safety_handler])

# All LangChain operations now have safety checks
result = llm("What is AI safety?")
```

### 2.2 LlamaIndex Integration

```python
from llama_index import VectorStoreIndex, SimpleDirectoryReader
from wia_safety.llama_index import SafetyNodePostprocessor

documents = SimpleDirectoryReader('data').load_data()
index = VectorStoreIndex.from_documents(documents)

# Add safety post-processing
safety_processor = SafetyNodePostprocessor(
    filters=['toxicity', 'pii', 'bias']
)

query_engine = index.as_query_engine(
    node_postprocessors=[safety_processor]
)

response = query_engine.query("Tell me about...")
```

### 2.3 Hugging Face Integration

```python
from transformers import pipeline
from wia_safety.transformers import SafetyPipeline

# Wrap Hugging Face pipeline
base_pipeline = pipeline("text-generation", model="gpt2")
safe_pipeline = SafetyPipeline(
    base_pipeline,
    input_filters=['prompt-injection'],
    output_filters=['toxicity', 'pii']
)

# Use like normal pipeline, with safety
result = safe_pipeline("Generate a story about...")
```

---

## 3. Cloud Platform Integration

### 3.1 AWS Integration

```typescript
// Lambda Function with Safety Layer
import { SafetyProtocol } from '@wia/ai-safety-protocol';

const safety = new SafetyProtocol({
    apiKey: process.env.WIA_API_KEY
});

export const handler = async (event) => {
    // Extract user input
    const input = JSON.parse(event.body).message;

    // Safety check
    const safetyResult = await safety.check(input);
    if (!safetyResult.safe) {
        return {
            statusCode: 400,
            body: JSON.stringify({
                error: 'Input blocked by safety filter',
                reason: safetyResult.reason
            })
        };
    }

    // Process with AI model (Bedrock, SageMaker, etc.)
    const aiResponse = await invokeAI(input);

    // Output safety check
    const outputCheck = await safety.check(aiResponse);
    if (!outputCheck.safe) {
        return {
            statusCode: 500,
            body: JSON.stringify({
                error: 'Output filtered for safety',
                reason: outputCheck.reason
            })
        };
    }

    return {
        statusCode: 200,
        body: JSON.stringify({response: aiResponse})
    };
};
```

### 3.2 Google Cloud Integration

```python
# Cloud Functions with Safety
from google.cloud import aiplatform
from wia_safety import SafetyClient

safety = SafetyClient(api_key=os.environ['WIA_API_KEY'])

def ai_endpoint(request):
    """Cloud Function with integrated safety"""
    input_text = request.get_json()['text']

    # Pre-check
    if not safety.validate_input(input_text):
        return {'error': 'Blocked by safety filter'}, 400

    # Call Vertex AI
    response = call_vertex_ai(input_text)

    # Post-check
    if not safety.validate_output(response):
        return {'error': 'Output filtered'}, 500

    return {'response': response}, 200
```

### 3.3 Azure Integration

```csharp
// Azure Functions with Safety
using WIA.SafetyProtocol;

[FunctionName("SafeAIEndpoint")]
public static async Task<IActionResult> Run(
    [HttpTrigger] HttpRequest req,
    ILogger log)
{
    var safety = new SafetyClient(Environment.GetEnvironmentVariable("WIA_API_KEY"));
    string input = await new StreamReader(req.Body).ReadToEndAsync();

    // Safety validation
    var inputCheck = await safety.ValidateInputAsync(input);
    if (!inputCheck.Safe)
    {
        return new BadRequestObjectResult(new {
            error = "Blocked",
            reason = inputCheck.Reason
        });
    }

    // Call Azure OpenAI
    var response = await AzureOpenAI.CompleteAsync(input);

    // Output validation
    var outputCheck = await safety.ValidateOutputAsync(response);
    if (!outputCheck.Safe)
    {
        return new ObjectResult(new {
            error = "Filtered",
            reason = outputCheck.Reason
        }) {StatusCode = 500};
    }

    return new OkObjectResult(new {response});
}
```

---

## 4. Compliance Implementation

### 4.1 GDPR Compliance

**Requirements:**
- Right to explanation for AI decisions
- Data minimization in training/inference
- User data deletion capability
- Privacy by design
- Data protection impact assessment (DPIA)

**Implementation:**
```typescript
class GDPRCompliantAI {
    async processRequest(userId: string, input: string) {
        // Log for GDPR article 22 (right to explanation)
        const requestId = uuid();
        await this.auditLog.record({
            requestId,
            userId,
            timestamp: new Date(),
            input: this.anonymize(input),
            purpose: 'AI assistance'
        });

        // Process with AI
        const result = await this.ai.process(input);

        // Store explanation
        await this.explanations.store(requestId, {
            decision: result.decision,
            reasoning: result.explanation,
            confidence: result.confidence,
            factors: result.factors
        });

        return result;
    }

    async handleDeletionRequest(userId: string) {
        // GDPR Article 17 - Right to be forgotten
        await this.auditLog.deleteUserData(userId);
        await this.trainingData.markForDeletion(userId);
        await this.userModels.delete(userId);

        return {status: 'deleted', timestamp: new Date()};
    }

    async provideExplanation(requestId: string) {
        // GDPR Article 22 - Right to explanation
        return await this.explanations.retrieve(requestId);
    }
}
```

### 4.2 EU AI Act Compliance

**High-Risk System Requirements:**
1. Risk management system
2. Data governance
3. Technical documentation
4. Record-keeping
5. Transparency
6. Human oversight
7. Accuracy, robustness, cybersecurity

**Implementation:**
```yaml
# AI Act Compliance Configuration
ai_act_compliance:
  risk_classification: "high-risk"  # hiring, credit, law enforcement
  requirements:
    risk_management:
      enabled: true
      continuous_assessment: true
      mitigation_measures: [guardrails, monitoring, human_review]

    data_governance:
      training_data_documented: true
      bias_assessment: quarterly
      data_quality_metrics: enabled

    technical_documentation:
      model_card: "path/to/model-card.json"
      architecture: "path/to/architecture.md"
      training_procedure: "path/to/training.md"

    record_keeping:
      retention_period: "7 years"
      automated_logging: true
      audit_trail: complete

    transparency:
      users_informed: true
      ai_generated_content_labeled: true
      explanations_available: true

    human_oversight:
      required_for: [high_stakes_decisions]
      override_capability: true
      escalation_process: documented

    technical_measures:
      accuracy_threshold: 0.95
      robustness_testing: continuous
      cybersecurity: [encryption, access_control, monitoring]
```

### 4.3 US Compliance (State Laws)

```typescript
// California Consumer Privacy Act (CCPA) Compliance
class CCPACompliance {
    async handleConsumerRequest(request: ConsumerRequest) {
        switch (request.type) {
            case 'access':
                // Right to know
                return await this.providePersonalData(request.userId);

            case 'delete':
                // Right to delete
                return await this.deletePersonalData(request.userId);

            case 'opt-out':
                // Opt-out of sale
                return await this.optOutOfSale(request.userId);

            case 'correct':
                // Right to correct
                return await this.correctPersonalData(
                    request.userId,
                    request.corrections
                );
        }
    }
}
```

### 4.4 Industry-Specific Compliance

**Healthcare (HIPAA):**
```typescript
const safetyConfig = {
    hipaa: {
        pii_redaction: {
            enabled: true,
            types: ['patient_name', 'mrn', 'ssn', 'date_of_birth']
        },
        encryption: {
            at_rest: true,
            in_transit: true,
            algorithm: 'AES-256'
        },
        access_control: {
            role_based: true,
            audit_trail: true
        },
        breach_notification: {
            enabled: true,
            threshold: 'any_breach'
        }
    }
};
```

**Finance (GLBA, FCRA):**
```typescript
const safetyConfig = {
    financial: {
        adverse_action_notices: {
            enabled: true,
            include_reasons: true,
            include_score_factors: true
        },
        fairness_requirements: {
            bias_testing: 'continuous',
            demographic_parity: true,
            disparate_impact_threshold: 0.8
        },
        explainability: {
            required: true,
            method: 'shapley_values'
        }
    }
};
```

---

## 5. Migration Strategy

### 5.1 Phased Rollout

**Phase 1: Monitoring Only (Week 1-2)**
- Deploy safety checks in shadow mode
- Collect metrics without blocking
- Tune thresholds based on data
- No impact to users

**Phase 2: Logging and Alerting (Week 3-4)**
- Enable alerting on violations
- Begin manual review process
- Identify false positives
- Refine configuration

**Phase 3: Partial Enforcement (Week 5-6)**
- Block obvious violations only
- Allow borderline cases
- Monitor user impact
- Iterate on configuration

**Phase 4: Full Enforcement (Week 7+)**
- Enable all guardrails
- Continuous monitoring
- Regular reviews
- Ongoing optimization

### 5.2 A/B Testing Safety Measures

```typescript
// Gradual rollout with A/B testing
const safetyRollout = {
    variants: {
        control: {traffic: 0.9, safety: 'off'},
        treatment: {traffic: 0.1, safety: 'on'}
    },
    metrics: [
        'user_satisfaction',
        'false_positive_rate',
        'safety_violations',
        'latency'
    ],
    successCriteria: {
        user_satisfaction: {min: -0.02},  // Max 2% decrease
        safety_violations: {max: 0.001},  // Under 0.1%
        latency: {max_increase: 100}      // Max 100ms added
    }
};
```

---

## 6. Monitoring and Reporting

### 6.1 Compliance Dashboard

Track compliance metrics:
- **GDPR**: Data deletion requests processed, explanations provided
- **AI Act**: Risk assessments completed, documentation up-to-date
- **Industry**: Domain-specific metrics

### 6.2 Regulatory Reporting

Generate reports for regulators:

```typescript
async function generateComplianceReport(period: string) {
    return {
        period,
        safety_metrics: {
            total_requests: await metrics.count(),
            blocked_requests: await metrics.blocked(),
            safety_score: await metrics.safetyScore()
        },
        bias_audit: await biasAudit.results(period),
        privacy_measures: {
            pii_detected: await privacy.piiDetections(period),
            data_deletions: await privacy.deletions(period)
        },
        incidents: await incidents.summary(period),
        certifications: await certifications.current()
    };
}
```

---

**弘益人間** - Through comprehensive integration and compliance, we ensure AI benefits all while meeting regulatory requirements.

© 2025 SmileStory Inc. / WIA
WIA-AI-010 Phase 4: Integration & Compliance v1.0

---

## Annex A — Conformance Tier Matrix

WIA conformance for ai-safety-protocol is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/ai-safety-protocol/api/` — TypeScript SDK skeleton
- `wia-standards/standards/ai-safety-protocol/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/ai-safety-protocol/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
