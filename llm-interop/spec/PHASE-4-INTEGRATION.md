# WIA-LLM-INTEROP: Phase 4 - Integration Specification

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Philosophy**: 홍익인간 (弘益人間) - Benefit All Humanity

---

## 1. Overview

This document defines integration patterns for WIA-LLM-INTEROP with:

| Category | Integrations |
|----------|--------------|
| AI Providers | OpenAI, Anthropic, Google, Meta, Mistral, Local models |
| Platforms | AWS, Azure, GCP, Kubernetes |
| Frameworks | LangChain, LlamaIndex, AutoGen, CrewAI |
| Enterprise | SSO, Audit, Compliance |

---

## 2. AI Provider Integrations

### 2.1 Integration Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  WIA-LLM-INTEROP Gateway                │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐  │
│  │ OpenAI   │  │Anthropic │  │ Google   │  │ Local  │  │
│  │ Adapter  │  │ Adapter  │  │ Adapter  │  │Adapter │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └───┬────┘  │
│       │             │             │             │       │
└───────┼─────────────┼─────────────┼─────────────┼───────┘
        │             │             │             │
        ▼             ▼             ▼             ▼
   ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────┐
   │ OpenAI  │   │Anthropic│   │ Gemini  │   │ Ollama  │
   │   API   │   │   API   │   │   API   │   │         │
   └─────────┘   └─────────┘   └─────────┘   └─────────┘
```

### 2.2 OpenAI Integration

#### Adapter Configuration

```yaml
adapter:
  provider: openai
  api_base: https://api.openai.com/v1
  api_key: ${OPENAI_API_KEY}

  model_mapping:
    gpt-4-turbo:
      wia_model_id: "did:wia:llm:openai:gpt-4-turbo"
      capability_level: 4
      domains: [general, code, reasoning]
    gpt-4o:
      wia_model_id: "did:wia:llm:openai:gpt-4o"
      capability_level: 4
      domains: [general, code, reasoning, vision]
    gpt-3.5-turbo:
      wia_model_id: "did:wia:llm:openai:gpt-3.5-turbo"
      capability_level: 2
      domains: [general]

  limits:
    max_tokens: 128000
    rate_limit_rpm: 500
    rate_limit_tpm: 200000
```

#### Message Translation

```rust
// WIA Message → OpenAI Format
fn to_openai(msg: &WiaMessage) -> OpenAIRequest {
    OpenAIRequest {
        model: translate_model_id(&msg.to.model_id),
        messages: convert_context(&msg.payload.context),
        max_tokens: msg.payload.constraints.max_tokens,
        temperature: 0.7,
        stream: msg.payload.constraints.stream.unwrap_or(false),
    }
}

// OpenAI Response → WIA Format
fn from_openai(resp: &OpenAIResponse, trace_id: &str) -> WiaMessage {
    WiaMessage {
        wia_version: "1.0".to_string(),
        message_type: MessageType::Response,
        trace_id: trace_id.to_string(),
        payload: ResponsePayload {
            content: resp.choices[0].message.content.clone(),
            confidence: estimate_confidence(&resp),
            usage: UsageStats {
                input_tokens: resp.usage.prompt_tokens,
                output_tokens: resp.usage.completion_tokens,
                processing_time_ms: 0, // Calculate from request timing
            },
            ..Default::default()
        },
        ..Default::default()
    }
}
```

### 2.3 Anthropic Integration

#### Adapter Configuration

```yaml
adapter:
  provider: anthropic
  api_base: https://api.anthropic.com/v1
  api_key: ${ANTHROPIC_API_KEY}
  api_version: "2024-01-01"

  model_mapping:
    claude-opus-4:
      wia_model_id: "did:wia:llm:anthropic:claude-opus-4"
      capability_level: 4
      domains: [general, code, reasoning, writing]
    claude-sonnet-4:
      wia_model_id: "did:wia:llm:anthropic:claude-sonnet-4"
      capability_level: 4
      domains: [general, code, reasoning]
    claude-haiku-3.5:
      wia_model_id: "did:wia:llm:anthropic:claude-haiku-3.5"
      capability_level: 3
      domains: [general, code]

  limits:
    max_tokens: 200000
    rate_limit_rpm: 50
```

#### Message Translation

```rust
fn to_anthropic(msg: &WiaMessage) -> AnthropicRequest {
    AnthropicRequest {
        model: translate_model_id(&msg.to.model_id),
        max_tokens: msg.payload.constraints.max_tokens.unwrap_or(4096),
        messages: convert_context(&msg.payload.context),
        system: extract_system_message(&msg.payload.context),
        stream: msg.payload.constraints.stream.unwrap_or(false),
    }
}

fn from_anthropic(resp: &AnthropicResponse, trace_id: &str) -> WiaMessage {
    WiaMessage {
        wia_version: "1.0".to_string(),
        message_type: MessageType::Response,
        trace_id: trace_id.to_string(),
        payload: ResponsePayload {
            content: resp.content[0].text.clone(),
            confidence: 0.9, // Anthropic doesn't provide confidence
            usage: UsageStats {
                input_tokens: resp.usage.input_tokens,
                output_tokens: resp.usage.output_tokens,
                ..Default::default()
            },
            ..Default::default()
        },
        ..Default::default()
    }
}
```

### 2.4 Google (Gemini) Integration

#### Adapter Configuration

```yaml
adapter:
  provider: google
  api_base: https://generativelanguage.googleapis.com/v1beta
  api_key: ${GOOGLE_API_KEY}

  model_mapping:
    gemini-2.0-flash:
      wia_model_id: "did:wia:llm:google:gemini-2.0-flash"
      capability_level: 4
      domains: [general, code, reasoning, vision]
    gemini-1.5-pro:
      wia_model_id: "did:wia:llm:google:gemini-1.5-pro"
      capability_level: 4
      domains: [general, code, reasoning]

  limits:
    max_tokens: 1000000  # 1M context
    rate_limit_rpm: 60
```

### 2.5 Local Model Integration (Ollama)

#### Adapter Configuration

```yaml
adapter:
  provider: ollama
  api_base: http://localhost:11434/api

  model_mapping:
    llama3.2:
      wia_model_id: "did:wia:llm:local:llama3.2"
      capability_level: 3
      domains: [general, code]
    codellama:
      wia_model_id: "did:wia:llm:local:codellama"
      capability_level: 3
      domains: [code]
    mistral:
      wia_model_id: "did:wia:llm:local:mistral"
      capability_level: 3
      domains: [general]

  limits:
    max_tokens: 32000
    rate_limit_rpm: 1000  # Local = no rate limit
```

### 2.6 Multi-Provider Federation

```yaml
federation:
  name: "Multi-Provider Analysis"
  topology: star

  members:
    - model_id: "did:wia:llm:anthropic:claude-opus-4"
      role: orchestrator
      adapter: anthropic

    - model_id: "did:wia:llm:openai:gpt-4o"
      role: specialist
      domain: vision
      adapter: openai

    - model_id: "did:wia:llm:google:gemini-2.0-flash"
      role: specialist
      domain: code
      adapter: google

    - model_id: "did:wia:llm:local:codellama"
      role: specialist
      domain: code_review
      adapter: ollama
```

---

## 3. Platform Integrations

### 3.1 AWS Integration

#### Lambda Deployment

```yaml
# serverless.yml
service: wia-llm-interop

provider:
  name: aws
  runtime: provided.al2023
  architecture: arm64
  region: ap-northeast-2

  environment:
    WIA_CONFIG_PATH: /opt/config/wia.yaml
    OPENAI_API_KEY: ${ssm:/wia/openai-api-key}
    ANTHROPIC_API_KEY: ${ssm:/wia/anthropic-api-key}

functions:
  message:
    handler: bootstrap
    events:
      - http:
          path: /wia/llm-interop/v1/message
          method: post

  federation:
    handler: bootstrap
    events:
      - http:
          path: /wia/llm-interop/v1/federation/{proxy+}
          method: any

  websocket:
    handler: bootstrap
    events:
      - websocket:
          route: $connect
      - websocket:
          route: $disconnect
      - websocket:
          route: $default
```

#### API Gateway Configuration

```yaml
# api-gateway.yaml
openapi: "3.0.1"
info:
  title: "WIA LLM Interop API"
  version: "1.0.0"

paths:
  /wia/llm-interop/v1/message:
    post:
      x-amazon-apigateway-integration:
        type: aws_proxy
        httpMethod: POST
        uri: arn:aws:apigateway:${region}:lambda:path/2015-03-31/functions/${LambdaArn}/invocations

      x-amazon-apigateway-request-validator: all
      x-amazon-apigateway-auth:
        type: AWS_IAM
```

### 3.2 Azure Integration

#### Container Apps Deployment

```yaml
# azure-container-apps.yaml
apiVersion: apps/v1
kind: ContainerApp
metadata:
  name: wia-llm-interop
  namespace: wia

spec:
  template:
    containers:
      - name: api
        image: wia/llm-interop:1.0.0
        resources:
          cpu: 1.0
          memory: 2Gi
        env:
          - name: AZURE_OPENAI_ENDPOINT
            secretRef: azure-openai-endpoint
          - name: AZURE_OPENAI_KEY
            secretRef: azure-openai-key

  scale:
    minReplicas: 2
    maxReplicas: 10
    rules:
      - name: http-scaling
        http:
          metadata:
            concurrentRequests: "100"

  ingress:
    external: true
    targetPort: 8080
    transport: http2
```

#### Azure OpenAI Adapter

```yaml
adapter:
  provider: azure_openai
  api_base: https://${AZURE_RESOURCE}.openai.azure.com
  api_key: ${AZURE_OPENAI_KEY}
  api_version: "2024-02-15-preview"

  deployments:
    gpt-4-turbo:
      deployment_name: "gpt-4-turbo-deployment"
      wia_model_id: "did:wia:llm:azure:gpt-4-turbo"
```

### 3.3 Google Cloud Integration

#### Cloud Run Deployment

```yaml
# cloud-run.yaml
apiVersion: serving.knative.dev/v1
kind: Service
metadata:
  name: wia-llm-interop
  annotations:
    run.googleapis.com/ingress: all

spec:
  template:
    metadata:
      annotations:
        autoscaling.knative.dev/minScale: "1"
        autoscaling.knative.dev/maxScale: "100"
        run.googleapis.com/cpu-throttling: "false"

    spec:
      containerConcurrency: 100
      timeoutSeconds: 300

      containers:
        - image: gcr.io/wia-project/llm-interop:1.0.0
          ports:
            - containerPort: 8080
          resources:
            limits:
              cpu: "2"
              memory: 4Gi
          env:
            - name: GOOGLE_API_KEY
              valueFrom:
                secretKeyRef:
                  name: google-api-key
                  key: value
```

### 3.4 Kubernetes Deployment

```yaml
# kubernetes/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-llm-interop
  namespace: wia
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-llm-interop
  template:
    metadata:
      labels:
        app: wia-llm-interop
    spec:
      containers:
        - name: api
          image: wia/llm-interop:1.0.0
          ports:
            - containerPort: 8080
              name: http
            - containerPort: 50051
              name: grpc
          env:
            - name: WIA_CONFIG_PATH
              value: /etc/wia/config.yaml
          volumeMounts:
            - name: config
              mountPath: /etc/wia
          readinessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 5
          livenessProbe:
            httpGet:
              path: /health
              port: 8080
            initialDelaySeconds: 10
          resources:
            requests:
              cpu: 500m
              memory: 512Mi
            limits:
              cpu: 2000m
              memory: 2Gi
      volumes:
        - name: config
          configMap:
            name: wia-config

---
apiVersion: v1
kind: Service
metadata:
  name: wia-llm-interop
  namespace: wia
spec:
  selector:
    app: wia-llm-interop
  ports:
    - name: http
      port: 80
      targetPort: 8080
    - name: grpc
      port: 443
      targetPort: 50051
  type: ClusterIP

---
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: wia-llm-interop
  namespace: wia
  annotations:
    kubernetes.io/ingress.class: nginx
    cert-manager.io/cluster-issuer: letsencrypt-prod
spec:
  tls:
    - hosts:
        - api.wia.org
      secretName: wia-tls
  rules:
    - host: api.wia.org
      http:
        paths:
          - path: /wia/llm-interop
            pathType: Prefix
            backend:
              service:
                name: wia-llm-interop
                port:
                  number: 80
```

---

## 4. Framework Integrations

### 4.1 LangChain Integration

```python
from langchain.llms.base import LLM
from langchain.callbacks.manager import CallbackManagerForLLMRun
from typing import Optional, List, Any

class WiaLLM(LLM):
    """LangChain LLM wrapper for WIA-LLM-INTEROP."""

    wia_endpoint: str = "https://api.wia.org/wia/llm-interop/v1"
    wia_api_key: str
    target_model_id: str

    @property
    def _llm_type(self) -> str:
        return "wia-llm-interop"

    def _call(
        self,
        prompt: str,
        stop: Optional[List[str]] = None,
        run_manager: Optional[CallbackManagerForLLMRun] = None,
        **kwargs: Any,
    ) -> str:
        import requests

        headers = {
            "Authorization": f"Bearer {self.wia_api_key}",
            "Content-Type": "application/json",
            "X-WIA-Version": "1.0"
        }

        payload = {
            "wia_version": "1.0",
            "document_type": "message",
            "type": "query",
            "to": {
                "model_id": self.target_model_id
            },
            "payload": {
                "intent": "generation",
                "content": prompt,
                "constraints": {
                    "response_format": "text"
                }
            }
        }

        response = requests.post(
            f"{self.wia_endpoint}/message",
            headers=headers,
            json=payload
        )
        response.raise_for_status()

        result = response.json()
        return result["data"]["payload"]["content"]

# Usage
llm = WiaLLM(
    wia_api_key="wia_...",
    target_model_id="did:wia:llm:anthropic:claude-opus"
)

result = llm("Explain quantum computing in simple terms.")
```

### 4.2 LlamaIndex Integration

```python
from llama_index.core.llms import CustomLLM, LLMMetadata, CompletionResponse
from llama_index.core.callbacks import CallbackManager

class WiaLlamaIndexLLM(CustomLLM):
    """LlamaIndex LLM for WIA-LLM-INTEROP."""

    def __init__(
        self,
        wia_endpoint: str,
        wia_api_key: str,
        model_id: str,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.wia_endpoint = wia_endpoint
        self.wia_api_key = wia_api_key
        self.model_id = model_id

    @property
    def metadata(self) -> LLMMetadata:
        return LLMMetadata(
            context_window=200000,
            num_output=32000,
            model_name=self.model_id,
        )

    def complete(self, prompt: str, **kwargs) -> CompletionResponse:
        # Send WIA message
        response = self._send_wia_message(prompt)
        return CompletionResponse(text=response["content"])

    def stream_complete(self, prompt: str, **kwargs):
        # Stream WIA message
        for chunk in self._stream_wia_message(prompt):
            yield CompletionResponse(text=chunk, delta=chunk)

# Usage
from llama_index.core import VectorStoreIndex, SimpleDirectoryReader

llm = WiaLlamaIndexLLM(
    wia_endpoint="https://api.wia.org/wia/llm-interop/v1",
    wia_api_key="wia_...",
    model_id="did:wia:llm:anthropic:claude-opus"
)

documents = SimpleDirectoryReader("data").load_data()
index = VectorStoreIndex.from_documents(documents, llm=llm)
```

### 4.3 AutoGen Integration

```python
from autogen import AssistantAgent, UserProxyAgent, GroupChat, GroupChatManager

# WIA-compatible agent wrapper
class WiaAgent(AssistantAgent):
    """AutoGen agent using WIA-LLM-INTEROP."""

    def __init__(
        self,
        name: str,
        wia_model_id: str,
        wia_endpoint: str,
        wia_api_key: str,
        **kwargs
    ):
        # Configure LLM to use WIA adapter
        llm_config = {
            "config_list": [{
                "model": wia_model_id,
                "api_key": wia_api_key,
                "base_url": f"{wia_endpoint}/openai-compat",
            }],
            "timeout": 120,
        }

        super().__init__(
            name=name,
            llm_config=llm_config,
            **kwargs
        )

# Create WIA Federation as AutoGen GroupChat
medical_ai = WiaAgent(
    name="medical_expert",
    wia_model_id="did:wia:llm:specialized:medical",
    wia_endpoint="https://api.wia.org/wia/llm-interop/v1",
    wia_api_key="wia_...",
    system_message="You are a medical expert AI."
)

legal_ai = WiaAgent(
    name="legal_expert",
    wia_model_id="did:wia:llm:specialized:legal",
    wia_endpoint="https://api.wia.org/wia/llm-interop/v1",
    wia_api_key="wia_...",
    system_message="You are a legal expert AI."
)

user_proxy = UserProxyAgent(name="user")

group_chat = GroupChat(
    agents=[medical_ai, legal_ai, user_proxy],
    messages=[],
    max_round=10
)

manager = GroupChatManager(groupchat=group_chat)
```

---

## 5. Enterprise Integrations

### 5.1 SSO Integration (SAML/OIDC)

```yaml
# sso-config.yaml
sso:
  enabled: true

  providers:
    - type: oidc
      name: okta
      issuer: https://company.okta.com
      client_id: ${OKTA_CLIENT_ID}
      client_secret: ${OKTA_CLIENT_SECRET}
      scopes: ["openid", "profile", "email"]

    - type: saml
      name: azure_ad
      metadata_url: https://login.microsoftonline.com/.../metadata
      entity_id: https://api.wia.org
      acs_url: https://api.wia.org/auth/saml/callback

  role_mapping:
    "admin": ["wia:admin"]
    "developer": ["wia:message:send", "wia:federation:join"]
    "viewer": ["wia:capability:read"]
```

### 5.2 Audit Logging

```yaml
# audit-config.yaml
audit:
  enabled: true
  level: detailed  # minimal, standard, detailed

  storage:
    type: elasticsearch
    endpoint: https://es.company.com
    index_pattern: wia-audit-{date}

  events:
    - type: message.sent
      log_payload: false  # Don't log actual content
      log_metadata: true

    - type: federation.joined
      log_payload: true

    - type: auth.login
      log_payload: true

    - type: capability.queried
      log_payload: false

  retention:
    days: 365
    archive_after_days: 90
```

### 5.3 Compliance Configuration

```yaml
# compliance-config.yaml
compliance:
  gdpr:
    enabled: true
    data_residency: ["eu-west-1", "eu-central-1"]
    encryption_at_rest: true
    right_to_erasure: true
    consent_required: true

  hipaa:
    enabled: false  # Enable for healthcare
    audit_controls: true
    access_controls: true
    encryption_standard: "AES-256"

  sox:
    enabled: false  # Enable for financial
    change_management: true
    access_reviews: quarterly

  data_classification:
    levels:
      - name: public
        encryption: optional
        audit: minimal

      - name: internal
        encryption: required
        audit: standard

      - name: confidential
        encryption: required
        audit: detailed
        access_review: true

      - name: restricted
        encryption: required
        audit: detailed
        access_review: true
        dlp: true
```

---

## 6. SDK Reference

### 6.1 Rust SDK

```rust
// Cargo.toml
[dependencies]
wia-llm-interop = "1.0"

// Usage
use wia_llm_interop::{Client, Message, Federation};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Create client
    let client = Client::builder()
        .endpoint("https://api.wia.org/wia/llm-interop/v1")
        .api_key("wia_...")
        .build()?;

    // Send message
    let response = client
        .message()
        .to("did:wia:llm:anthropic:claude-opus")
        .content("Hello, world!")
        .send()
        .await?;

    println!("Response: {}", response.content);

    // Create federation
    let federation = client
        .federation()
        .name("My Federation")
        .topology(Topology::Star)
        .create()
        .await?;

    // Submit task
    let task = federation
        .task("Analyze this document")
        .priority(Priority::High)
        .submit()
        .await?;

    // Wait for result
    let result = task.await_result().await?;
    println!("Task result: {}", result.content);

    Ok(())
}
```

### 6.2 TypeScript SDK

```typescript
// npm install @wia/llm-interop

import { WiaClient, Federation, Message } from '@wia/llm-interop';

// Create client
const client = new WiaClient({
  endpoint: 'https://api.wia.org/wia/llm-interop/v1',
  apiKey: 'wia_...',
});

// Send message
const response = await client.message({
  to: 'did:wia:llm:anthropic:claude-opus',
  content: 'Hello, world!',
});

console.log('Response:', response.content);

// Create federation
const federation = await client.createFederation({
  name: 'My Federation',
  topology: 'star',
});

// Join with another AI
await federation.addMember({
  modelId: 'did:wia:llm:openai:gpt-4o',
  role: 'specialist',
  domain: 'vision',
});

// Submit task
const task = await federation.submitTask({
  query: 'Analyze this image and provide insights',
  attachments: [{ type: 'image', url: '...' }],
});

// Stream results
for await (const chunk of task.stream()) {
  process.stdout.write(chunk.content);
}
```

### 6.3 Python SDK

```python
# pip install wia-llm-interop

from wia_llm_interop import WiaClient, Federation
import asyncio

async def main():
    # Create client
    client = WiaClient(
        endpoint="https://api.wia.org/wia/llm-interop/v1",
        api_key="wia_..."
    )

    # Send message
    response = await client.message(
        to="did:wia:llm:anthropic:claude-opus",
        content="Hello, world!"
    )
    print(f"Response: {response.content}")

    # Create federation
    federation = await client.create_federation(
        name="My Federation",
        topology="star"
    )

    # Submit task with streaming
    async for chunk in federation.submit_task_stream(
        query="Explain quantum computing",
        constraints={"language": "ko"}
    ):
        print(chunk.content, end="", flush=True)

if __name__ == "__main__":
    asyncio.run(main())
```

---

## 7. Migration Guide

### 7.1 From OpenAI API

```python
# Before (OpenAI direct)
from openai import OpenAI
client = OpenAI()
response = client.chat.completions.create(
    model="gpt-4",
    messages=[{"role": "user", "content": "Hello"}]
)

# After (WIA-LLM-INTEROP)
from wia_llm_interop import WiaClient
client = WiaClient(api_key="wia_...")
response = await client.message(
    to="did:wia:llm:openai:gpt-4",
    content="Hello"
)
```

### 7.2 From LangChain

```python
# Before (LangChain with OpenAI)
from langchain_openai import ChatOpenAI
llm = ChatOpenAI(model="gpt-4")

# After (LangChain with WIA)
from wia_llm_interop.langchain import WiaLLM
llm = WiaLLM(
    wia_api_key="wia_...",
    target_model_id="did:wia:llm:openai:gpt-4"
)
# Rest of LangChain code unchanged
```

---

## 8. Monitoring & Observability

### 8.1 Metrics Export

```yaml
# metrics-config.yaml
metrics:
  enabled: true

  exporters:
    prometheus:
      enabled: true
      port: 9090
      path: /metrics

    opentelemetry:
      enabled: true
      endpoint: https://otel.company.com
      protocol: grpc

  labels:
    service: wia-llm-interop
    environment: production

  custom_metrics:
    - name: wia_message_latency_seconds
      type: histogram
      buckets: [0.1, 0.5, 1.0, 2.0, 5.0, 10.0]

    - name: wia_federation_task_total
      type: counter
      labels: [status, federation_id]
```

### 8.2 Distributed Tracing

```yaml
# tracing-config.yaml
tracing:
  enabled: true
  sampler: probabilistic
  sample_rate: 0.1  # 10%

  exporters:
    jaeger:
      enabled: true
      endpoint: https://jaeger.company.com/api/traces

    zipkin:
      enabled: false

  propagation:
    - tracecontext  # W3C Trace Context
    - baggage
```

---

**Document ID**: WIA-LLM-INTEROP-PHASE-4
**Version**: 1.0.0
**Date**: 2025-12-16
**Philosophy**: 홍익인간 (弘益人間) - 통합으로 AI들이 하나가 됩니다
