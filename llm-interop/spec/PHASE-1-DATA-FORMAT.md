# WIA-LLM-INTEROP: Phase 1 - Data Format Specification

**Version**: 1.0.0
**Date**: 2025-12-16
**Status**: Draft
**Philosophy**: 홍익인간 (弘益人間) - Benefit All Humanity

> AI들의 연동 표준 - 언어, 국경, 회사의 벽을 넘어 모든 AI가 협업합니다.

---

## 1. Overview

WIA-LLM-INTEROP defines three interconnected standards:

| Standard | Purpose |
|----------|---------|
| **WIA-LLM-CAPABILITY** | AI declares its abilities |
| **WIA-LLM-MESSAGE** | AI-to-AI message format |
| **WIA-LLM-FEDERATION** | Multi-AI collaboration protocol |

```
┌─────────────────────────────────────────────────────────────────┐
│                    WIA-LLM-INTEROP v1.0                         │
├─────────────────┬─────────────────┬─────────────────────────────┤
│   CAPABILITY    │    MESSAGE      │        FEDERATION           │
│   "나는 뭘      │   "이렇게       │       "함께 일하면          │
│    할 수 있어"  │    말해"        │        이렇게"              │
└─────────────────┴─────────────────┴─────────────────────────────┘
```

---

## 2. WIA-LLM-CAPABILITY Data Structures

### 2.1 CapabilityDocument

Root structure for AI capability declaration.

```typescript
interface CapabilityDocument {
  // Header
  wia_version: "1.0";
  document_type: "capability";
  document_id: UUID;
  created_at: ISO8601;
  updated_at: ISO8601;

  // Model Identity
  model: ModelIdentity;

  // Classification
  level: CapabilityLevel;

  // Detailed Capabilities
  domains: DomainExpertise[];
  languages: LanguageSupport[];
  modalities: ModalitySupport;
  tools: ToolCapability[];

  // Limits
  limits: ResourceLimits;

  // Trust & Quality
  trust: TrustMetadata;

  // Contact
  endpoints: EndpointInfo;

  // Signature
  signature?: CryptographicSignature;
}
```

### 2.2 ModelIdentity

```typescript
interface ModelIdentity {
  // Unique identifier (DID format recommended)
  model_id: string;            // "did:wia:llm:anthropic:claude-opus"

  // Display information
  display_name: string;        // "Claude Opus 4"
  short_name: string;          // "claude-opus"

  // Provider
  provider: ProviderInfo;

  // Version
  version: string;             // "4.0.0"
  release_date: ISO8601;

  // Family (for related models)
  family?: string;             // "claude"
  family_rank?: number;        // 1 = flagship, 2 = mid, 3 = lite
}

interface ProviderInfo {
  id: string;                  // "anthropic"
  name: string;                // "Anthropic"
  url: string;                 // "https://anthropic.com"
  country?: string;            // ISO 3166-1 alpha-2
}
```

### 2.3 CapabilityLevel

4-level classification system:

```typescript
enum CapabilityLevel {
  Level1 = 1,  // Basic: Simple Q&A, text generation
  Level2 = 2,  // Reasoning: Multi-step reasoning, analysis
  Level3 = 3,  // Specialist: Domain expert, specialized tasks
  Level4 = 4,  // Advanced: Tool use, multi-modal, agentic
}

interface LevelCriteria {
  level: CapabilityLevel;

  requirements: {
    // Level 1: Basic
    text_generation: boolean;

    // Level 2: Reasoning
    multi_step_reasoning?: boolean;
    context_understanding?: boolean;

    // Level 3: Specialist
    domain_expertise?: string[];
    certification?: string[];

    // Level 4: Advanced
    tool_use?: boolean;
    multi_modal?: boolean;
    agentic_behavior?: boolean;
    code_execution?: boolean;
  };
}
```

### 2.4 DomainExpertise

```typescript
interface DomainExpertise {
  // Domain identifier
  domain: DomainCode;

  // Proficiency (0.0 - 1.0)
  proficiency: number;

  // Sub-domains
  sub_domains: SubDomainExpertise[];

  // Certifications (optional)
  certifications?: Certification[];

  // Benchmark scores (optional)
  benchmarks?: BenchmarkResult[];
}

enum DomainCode {
  // Technical
  CODE = "code",
  MATH = "math",
  SCIENCE = "science",
  ENGINEERING = "engineering",
  DATA_SCIENCE = "data_science",
  CYBERSECURITY = "cybersecurity",

  // Professional
  MEDICAL = "medical",
  LEGAL = "legal",
  FINANCE = "finance",
  EDUCATION = "education",

  // Creative
  WRITING = "writing",
  TRANSLATION = "translation",
  MARKETING = "marketing",

  // General
  GENERAL = "general",
  REASONING = "reasoning",
  RESEARCH = "research",
}

interface SubDomainExpertise {
  name: string;                // "cardiology", "rust", "contract_law"
  proficiency: number;         // 0.0 - 1.0
}

interface Certification {
  name: string;                // "WIA-MEDICAL-L3"
  issuer: string;              // "wia.org"
  issued_date: ISO8601;
  expiry_date?: ISO8601;
  verification_url?: string;
}

interface BenchmarkResult {
  benchmark: string;           // "MMLU", "HumanEval", "GSM8K"
  score: number;
  max_score: number;
  percentile?: number;
  test_date: ISO8601;
}
```

### 2.5 LanguageSupport

```typescript
interface LanguageSupport {
  // ISO 639-1 code
  code: string;                // "ko", "en", "ar", etc.

  // Proficiency (0.0 - 1.0)
  proficiency: number;

  // Capabilities per language
  capabilities: {
    understanding: number;     // 읽기/듣기 이해
    generation: number;        // 말하기/쓰기 생성
    translation_from: number;  // 이 언어에서 번역
    translation_to: number;    // 이 언어로 번역
    code_comments?: number;    // 코드 주석 이해
  };

  // Script support
  scripts: string[];           // ["Hangul", "Latin", "Hanja"]
}
```

### 2.6 ModalitySupport

```typescript
interface ModalitySupport {
  text: TextModality;
  image?: ImageModality;
  audio?: AudioModality;
  video?: VideoModality;
  code?: CodeModality;
  structured_data?: StructuredDataModality;
}

interface TextModality {
  input: boolean;
  output: boolean;
  max_input_length: number;    // characters or tokens
  max_output_length: number;
  streaming: boolean;
}

interface ImageModality {
  input: boolean;              // Can understand images
  output: boolean;             // Can generate images

  // Input capabilities
  input_formats?: string[];    // ["png", "jpg", "webp", "gif"]
  max_resolution?: string;     // "4096x4096"
  ocr: boolean;
  diagram_understanding: boolean;

  // Output capabilities
  output_formats?: string[];
  output_resolution?: string;
}

interface AudioModality {
  input: boolean;              // Can understand audio
  output: boolean;             // Can generate audio (TTS)

  input_formats?: string[];    // ["mp3", "wav", "m4a"]
  max_duration_seconds?: number;
  transcription: boolean;
  speaker_identification?: boolean;

  output_formats?: string[];
  voices?: string[];           // Available TTS voices
}

interface VideoModality {
  input: boolean;
  output: boolean;

  input_formats?: string[];
  max_duration_seconds?: number;
  frame_analysis: boolean;
  temporal_understanding: boolean;
}

interface CodeModality {
  input: boolean;              // Can read/understand code
  output: boolean;             // Can generate code
  execution: boolean;          // Can execute code (sandbox)

  languages: CodeLanguageSupport[];
}

interface CodeLanguageSupport {
  language: string;            // "rust", "python", "typescript"
  proficiency: number;         // 0.0 - 1.0
  execution?: boolean;         // Can execute this language
}

interface StructuredDataModality {
  json: boolean;
  yaml: boolean;
  xml: boolean;
  csv: boolean;
  sql: boolean;

  // Schema understanding
  json_schema: boolean;
  openapi: boolean;
  graphql: boolean;
}
```

### 2.7 ToolCapability

```typescript
interface ToolCapability {
  // Tool identifier
  tool_id: string;             // "web_search", "code_execution"

  // Human-readable name
  name: string;

  // Description
  description: string;

  // Enabled by default
  enabled: boolean;

  // Configuration
  config?: Record<string, any>;

  // Limits
  limits?: ToolLimits;
}

interface ToolLimits {
  calls_per_minute?: number;
  calls_per_session?: number;
  timeout_seconds?: number;
}

// Standard tool identifiers
enum StandardTool {
  WEB_SEARCH = "web_search",
  WEB_FETCH = "web_fetch",
  CODE_EXECUTION = "code_execution",
  FILE_READ = "file_read",
  FILE_WRITE = "file_write",
  DATABASE_QUERY = "database_query",
  API_CALL = "api_call",
  IMAGE_GENERATION = "image_generation",
  AUDIO_GENERATION = "audio_generation",
  CALCULATOR = "calculator",
  CALENDAR = "calendar",
  EMAIL = "email",
}
```

### 2.8 ResourceLimits

```typescript
interface ResourceLimits {
  // Token limits
  max_input_tokens: number;
  max_output_tokens: number;
  max_context_window: number;

  // Rate limits
  rate_limits: RateLimits;

  // Timeout
  max_response_time_seconds: number;

  // Concurrent requests
  max_concurrent_requests?: number;
}

interface RateLimits {
  requests_per_minute: number;
  tokens_per_minute: number;
  tokens_per_day?: number;

  // Burst allowance
  burst_requests?: number;
  burst_window_seconds?: number;
}
```

### 2.9 TrustMetadata

```typescript
interface TrustMetadata {
  // Confidence threshold (refuse if below)
  confidence_threshold: number;  // 0.0 - 1.0

  // Self-reported hallucination rate
  hallucination_rate?: number;   // 0.0 - 1.0

  // Knowledge cutoff
  knowledge_cutoff: ISO8601;

  // Safety features
  safety: SafetyInfo;

  // Audit & compliance
  compliance: ComplianceInfo;
}

interface SafetyInfo {
  content_filtering: boolean;
  nsfw_filter: boolean;
  hate_speech_filter: boolean;
  violence_filter: boolean;
  self_harm_filter: boolean;

  // Refusal categories
  refusal_categories: string[];
}

interface ComplianceInfo {
  gdpr_compliant: boolean;
  hipaa_compliant?: boolean;
  sox_compliant?: boolean;
  iso27001?: boolean;

  data_residency?: string[];   // Countries where data is stored
  audit_logging: boolean;
}
```

### 2.10 EndpointInfo

```typescript
interface EndpointInfo {
  // Primary API endpoint
  api: APIEndpoint;

  // Capability discovery endpoint
  capability: string;          // URL returning this CapabilityDocument

  // Documentation
  documentation?: string;

  // Support
  support?: SupportInfo;
}

interface APIEndpoint {
  url: string;
  protocol: "http" | "https" | "wss" | "grpc";
  version: string;

  // Authentication
  auth: AuthMethod[];
}

enum AuthMethod {
  API_KEY = "api_key",
  OAUTH2 = "oauth2",
  JWT = "jwt",
  MTLS = "mtls",
  DID_AUTH = "did_auth",
}

interface SupportInfo {
  email?: string;
  url?: string;
  slack?: string;
  discord?: string;
}
```

---

## 3. WIA-LLM-MESSAGE Data Structures

### 3.1 MessageEnvelope

Universal message wrapper for all AI-to-AI communication.

```typescript
interface MessageEnvelope {
  // Header
  wia_version: "1.0";
  document_type: "message";

  // Message identification
  message_id: UUID;
  timestamp: ISO8601;

  // Tracing
  trace_id: UUID;              // Conversation trace
  parent_message_id?: UUID;    // For responses
  sequence_number?: number;    // Order in conversation

  // Routing
  from: MessageParticipant;
  to: MessageTarget;

  // Message type
  type: MessageType;

  // Priority & TTL
  priority: Priority;
  ttl_seconds?: number;

  // Payload
  payload: MessagePayload;

  // Metadata
  metadata?: MessageMetadata;

  // Security
  signature?: CryptographicSignature;
}

interface MessageParticipant {
  model_id: string;
  capability_uri?: string;     // URL to CapabilityDocument
  instance_id?: string;        // For distributed deployments
}

interface MessageTarget {
  // Direct addressing
  model_id?: string;

  // Capability-based routing
  capability_query?: CapabilityQuery;

  // Broadcast
  broadcast?: BroadcastScope;
}

interface CapabilityQuery {
  required_domains?: DomainCode[];
  min_level?: CapabilityLevel;
  required_languages?: string[];
  required_tools?: string[];
}

enum BroadcastScope {
  FEDERATION = "federation",   // All members of current federation
  DOMAIN = "domain",           // All AIs in same domain
  ALL = "all",                 // All registered AIs
}
```

### 3.2 MessageType

```typescript
enum MessageType {
  // Communication
  QUERY = "query",
  RESPONSE = "response",
  STREAM = "stream",

  // Control
  HANDSHAKE = "handshake",
  HEARTBEAT = "heartbeat",
  ACK = "ack",

  // Error
  ERROR = "error",

  // Federation
  TASK_ASSIGNMENT = "task_assignment",
  TASK_RESULT = "task_result",
  CONSENSUS_REQUEST = "consensus_request",
  CONSENSUS_VOTE = "consensus_vote",

  // Discovery
  CAPABILITY_REQUEST = "capability_request",
  CAPABILITY_RESPONSE = "capability_response",
}

enum Priority {
  LOW = "low",
  NORMAL = "normal",
  HIGH = "high",
  CRITICAL = "critical",
}
```

### 3.3 QueryPayload

```typescript
interface QueryPayload {
  // Intent classification
  intent: QueryIntent;

  // The actual query
  content: string;

  // Previous context
  context?: ConversationContext[];

  // Constraints
  constraints: QueryConstraints;

  // Attachments
  attachments?: Attachment[];
}

enum QueryIntent {
  QUESTION = "question",
  TASK = "task",
  ANALYSIS = "analysis",
  GENERATION = "generation",
  TRANSLATION = "translation",
  VERIFICATION = "verification",
  DELEGATION = "delegation",
}

interface ConversationContext {
  role: "user" | "assistant" | "system";
  content: string;
  model_id?: string;
  timestamp?: ISO8601;
}

interface QueryConstraints {
  // Response format
  response_format: "text" | "json" | "markdown" | "code";

  // Output limits
  max_tokens?: number;

  // Language
  response_language?: string;

  // Quality
  confidence_min?: number;
  require_reasoning?: boolean;
  require_sources?: boolean;

  // Timeout
  timeout_seconds?: number;
}

interface Attachment {
  attachment_id: UUID;
  type: "text" | "image" | "audio" | "file" | "data";

  // Inline content
  content?: string;            // Base64 for binary

  // External reference
  url?: string;

  // Metadata
  filename?: string;
  mime_type?: string;
  size_bytes?: number;
}
```

### 3.4 ResponsePayload

```typescript
interface ResponsePayload {
  // The response content
  content: string;

  // Confidence score (0.0 - 1.0)
  confidence: number;

  // Reasoning (if requested)
  reasoning?: string;

  // Sources/citations
  sources?: Source[];

  // Suggestions for follow-up
  suggestions?: Suggestion[];

  // Delegation recommendations
  delegations?: DelegationSuggestion[];

  // Usage stats
  usage: UsageStats;
}

interface Source {
  title: string;
  url?: string;
  excerpt?: string;
  relevance: number;           // 0.0 - 1.0
}

interface Suggestion {
  type: "question" | "action" | "clarification";
  content: string;
  relevance: number;
}

interface DelegationSuggestion {
  reason: string;
  suggested_capability: CapabilityQuery;
  partial_result?: string;
}

interface UsageStats {
  input_tokens: number;
  output_tokens: number;
  total_tokens: number;
  processing_time_ms: number;
  model_id: string;
}
```

### 3.5 StreamPayload

```typescript
interface StreamPayload {
  // Chunk identification
  chunk_index: number;
  total_chunks?: number;       // If known

  // Content
  content: string;

  // Finalization
  is_final: boolean;

  // Final chunk includes full response metadata
  final_metadata?: {
    confidence: number;
    usage: UsageStats;
    sources?: Source[];
  };
}
```

### 3.6 ErrorPayload

```typescript
interface ErrorPayload {
  // Standard error code
  code: ErrorCode;

  // Human-readable message
  message: string;

  // Detailed info
  details?: Record<string, any>;

  // Recoverability
  recoverable: boolean;

  // Retry guidance
  retry_after_seconds?: number;

  // Fallback suggestion
  fallback?: FallbackSuggestion;
}

enum ErrorCode {
  // 1xx: Capability errors
  E100_CAPABILITY_MISMATCH = "E100",
  E101_UNSUPPORTED_LANGUAGE = "E101",
  E102_UNSUPPORTED_MODALITY = "E102",
  E103_TOOL_UNAVAILABLE = "E103",

  // 2xx: Message errors
  E200_INVALID_FORMAT = "E200",
  E201_MISSING_FIELD = "E201",
  E202_INVALID_PAYLOAD = "E202",
  E203_MESSAGE_TOO_LARGE = "E203",

  // 3xx: Resource errors
  E300_RATE_LIMITED = "E300",
  E301_CONTEXT_TOO_LONG = "E301",
  E302_TIMEOUT = "E302",
  E303_QUOTA_EXCEEDED = "E303",

  // 4xx: Auth errors
  E400_AUTH_FAILED = "E400",
  E401_AUTH_EXPIRED = "E401",
  E402_PERMISSION_DENIED = "E402",

  // 5xx: Service errors
  E500_SERVICE_UNAVAILABLE = "E500",
  E501_INTERNAL_ERROR = "E501",
  E502_UPSTREAM_ERROR = "E502",

  // 6xx: Federation errors
  E600_FEDERATION_NOT_FOUND = "E600",
  E601_TASK_FAILED = "E601",
  E602_CONSENSUS_FAILED = "E602",
  E603_NODE_UNAVAILABLE = "E603",

  // 7xx: Quality errors
  E700_CONFIDENCE_TOO_LOW = "E700",
  E701_CANNOT_VERIFY = "E701",
  E702_CONFLICTING_SOURCES = "E702",
}

interface FallbackSuggestion {
  type: "retry" | "delegate" | "simplify" | "split";
  details: string;
  suggested_action?: Record<string, any>;
}
```

### 3.7 HandshakePayload

```typescript
interface HandshakePayload {
  phase: HandshakePhase;

  // Capability exchange
  capability?: CapabilityDocument;

  // Protocol negotiation
  supported_versions: string[];
  preferred_encoding: "utf-8" | "utf-16";
  compression?: "none" | "gzip" | "zstd" | "br";

  // Security negotiation
  supported_auth: AuthMethod[];
  supported_encryption?: string[];  // "tls1.3", "mtls"

  // Federation context (if joining)
  federation_id?: UUID;
  requested_role?: FederationRole;
}

enum HandshakePhase {
  INIT = "init",
  CAPABILITY_EXCHANGE = "capability_exchange",
  NEGOTIATION = "negotiation",
  AUTH = "auth",
  COMPLETE = "complete",
}
```

---

## 4. WIA-LLM-FEDERATION Data Structures

### 4.1 FederationDocument

```typescript
interface FederationDocument {
  // Header
  wia_version: "1.0";
  document_type: "federation";

  // Identification
  federation_id: UUID;
  name: string;
  description?: string;

  // Timestamps
  created_at: ISO8601;
  updated_at: ISO8601;

  // Topology
  topology: FederationTopology;

  // Members
  members: FederationMember[];

  // Configuration
  config: FederationConfig;

  // Current state
  state: FederationState;

  // Signature
  signature?: CryptographicSignature;
}

enum FederationTopology {
  STAR = "star",               // Central orchestrator
  MESH = "mesh",               // Peer-to-peer
  HIERARCHICAL = "hierarchical", // Multi-level
  RING = "ring",               // Sequential processing
}

enum FederationState {
  FORMING = "forming",
  ACTIVE = "active",
  DEGRADED = "degraded",
  SUSPENDED = "suspended",
  DISSOLVED = "dissolved",
}
```

### 4.2 FederationMember

```typescript
interface FederationMember {
  // Identity
  member_id: UUID;
  model_id: string;
  capability_uri: string;

  // Role
  role: FederationRole;

  // Domains (for routing)
  domains: DomainCode[];

  // Status
  status: MemberStatus;

  // Performance metrics
  metrics?: MemberMetrics;

  // Connection info
  endpoint: string;
  last_seen?: ISO8601;
}

enum FederationRole {
  ORCHESTRATOR = "orchestrator",
  SPECIALIST = "specialist",
  ROUTER = "router",
  AGGREGATOR = "aggregator",
  VALIDATOR = "validator",
  OBSERVER = "observer",        // Read-only
}

enum MemberStatus {
  JOINING = "joining",
  ACTIVE = "active",
  BUSY = "busy",
  UNAVAILABLE = "unavailable",
  LEAVING = "leaving",
}

interface MemberMetrics {
  requests_handled: number;
  average_response_time_ms: number;
  success_rate: number;         // 0.0 - 1.0
  uptime_percentage: number;
  last_error?: ErrorCode;
}
```

### 4.3 FederationConfig

```typescript
interface FederationConfig {
  // Task handling
  task_config: TaskConfig;

  // Consensus
  consensus_config: ConsensusConfig;

  // Fault tolerance
  fault_tolerance: FaultToleranceConfig;

  // Security
  security: FederationSecurityConfig;

  // Timeouts
  timeouts: TimeoutConfig;
}

interface TaskConfig {
  // Task decomposition
  auto_decompose: boolean;
  max_subtasks: number;

  // Parallelism
  max_parallel_tasks: number;

  // Retry
  max_retries: number;
  retry_delay_ms: number;
}

interface ConsensusConfig {
  // Consensus algorithm
  algorithm: ConsensusAlgorithm;

  // Quorum
  quorum_percentage: number;   // 0.0 - 1.0

  // Voting timeout
  voting_timeout_seconds: number;

  // Tie-breaking
  tie_breaker: TieBreaker;
}

enum ConsensusAlgorithm {
  MAJORITY = "majority",
  WEIGHTED = "weighted",
  UNANIMOUS = "unanimous",
  RAFT = "raft",
  PBFT = "pbft",
}

enum TieBreaker {
  ORCHESTRATOR = "orchestrator",
  HIGHEST_CONFIDENCE = "highest_confidence",
  RANDOM = "random",
  ABSTAIN = "abstain",
}

interface FaultToleranceConfig {
  // Circuit breaker
  circuit_breaker: CircuitBreakerConfig;

  // Failover
  failover_enabled: boolean;
  failover_threshold: number;  // Failures before failover

  // Graceful degradation
  degradation_enabled: boolean;
  min_members_for_operation: number;
}

interface CircuitBreakerConfig {
  enabled: boolean;
  failure_threshold: number;
  success_threshold: number;
  timeout_seconds: number;
  half_open_requests: number;
}

interface FederationSecurityConfig {
  // Member authentication
  require_auth: boolean;
  auth_methods: AuthMethod[];

  // Message signing
  require_signatures: boolean;

  // Encryption
  require_encryption: boolean;
  encryption_algorithm?: string;

  // Trust
  trust_model: TrustModel;
  malicious_detection: boolean;
}

enum TrustModel {
  OPEN = "open",               // Trust all
  REGISTERED = "registered",   // Trust registered only
  VERIFIED = "verified",       // Trust verified only
  ZERO_TRUST = "zero_trust",   // Verify everything
}

interface TimeoutConfig {
  handshake_timeout_seconds: number;
  task_timeout_seconds: number;
  response_timeout_seconds: number;
  heartbeat_interval_seconds: number;
  member_timeout_seconds: number;
}
```

### 4.4 TaskDocument

```typescript
interface TaskDocument {
  // Identification
  task_id: UUID;
  parent_task_id?: UUID;       // For subtasks
  federation_id: UUID;

  // Task definition
  definition: TaskDefinition;

  // Assignment
  assignment: TaskAssignment;

  // Status
  status: TaskStatus;

  // Result (when complete)
  result?: TaskResult;

  // Timing
  created_at: ISO8601;
  started_at?: ISO8601;
  completed_at?: ISO8601;
  deadline?: ISO8601;
}

interface TaskDefinition {
  // Original query
  original_query: string;

  // Task description
  description: string;

  // Required capabilities
  required_capability?: CapabilityQuery;

  // Dependencies
  dependencies: UUID[];        // Task IDs that must complete first

  // Priority
  priority: Priority;

  // Constraints
  constraints?: TaskConstraints;
}

interface TaskConstraints {
  max_tokens?: number;
  response_format?: string;
  language?: string;
  confidence_min?: number;
  timeout_seconds?: number;
}

interface TaskAssignment {
  assigned_to: string;         // model_id
  assigned_at: ISO8601;
  assigned_by: string;         // orchestrator model_id
}

enum TaskStatus {
  PENDING = "pending",
  QUEUED = "queued",
  IN_PROGRESS = "in_progress",
  WAITING_DEPENDENCY = "waiting_dependency",
  COMPLETED = "completed",
  FAILED = "failed",
  CANCELLED = "cancelled",
  TIMEOUT = "timeout",
}

interface TaskResult {
  // Success indicator
  success: boolean;

  // Response content
  content?: string;

  // Confidence
  confidence?: number;

  // Error (if failed)
  error?: ErrorPayload;

  // Usage
  usage?: UsageStats;

  // Subtask results (for decomposed tasks)
  subtask_results?: TaskResult[];
}
```

### 4.5 ConsensusDocument

```typescript
interface ConsensusDocument {
  // Identification
  consensus_id: UUID;
  federation_id: UUID;

  // The question
  question: ConsensusQuestion;

  // Votes
  votes: ConsensusVote[];

  // Result
  result?: ConsensusResult;

  // Timing
  initiated_at: ISO8601;
  deadline: ISO8601;
  completed_at?: ISO8601;
}

interface ConsensusQuestion {
  type: ConsensusQuestionType;
  description: string;
  options?: ConsensusOption[];
  context?: string;
}

enum ConsensusQuestionType {
  YES_NO = "yes_no",
  MULTIPLE_CHOICE = "multiple_choice",
  FREE_FORM = "free_form",
  RANKING = "ranking",
}

interface ConsensusOption {
  option_id: string;
  description: string;
}

interface ConsensusVote {
  voter_id: string;            // model_id
  vote: string | number | string[];  // Depends on question type
  confidence: number;
  reasoning?: string;
  timestamp: ISO8601;
}

interface ConsensusResult {
  // Outcome
  outcome: string;

  // Confidence in outcome
  confidence: number;

  // Vote summary
  vote_count: number;
  quorum_reached: boolean;

  // Detailed breakdown
  vote_breakdown: Record<string, number>;

  // Dissent (if any)
  dissenting_votes?: ConsensusVote[];
}
```

---

## 5. Shared Data Structures

### 5.1 CryptographicSignature

```typescript
interface CryptographicSignature {
  algorithm: SignatureAlgorithm;
  public_key_id: string;       // Key identifier or DID
  signature: string;           // Base64 encoded
  timestamp: ISO8601;
}

enum SignatureAlgorithm {
  ED25519 = "ed25519",
  RSA_PSS = "rsa_pss",
  ECDSA_P256 = "ecdsa_p256",
  // Quantum-resistant (WIA-PQ-CRYPTO)
  DILITHIUM = "dilithium",
  SPHINCS = "sphincs",
}
```

### 5.2 Common Types

```typescript
type UUID = string;            // UUID v7 recommended
type ISO8601 = string;         // ISO 8601 datetime

// Version pattern
type Version = string;         // Semantic versioning: "1.0.0"

// DID pattern
type DID = string;             // "did:wia:llm:provider:model"
```

---

## 6. Example Documents

### 6.1 Capability Example: Claude Opus

```yaml
wia_version: "1.0"
document_type: capability
document_id: "550e8400-e29b-41d4-a716-446655440000"
created_at: "2025-12-16T00:00:00Z"
updated_at: "2025-12-16T00:00:00Z"

model:
  model_id: "did:wia:llm:anthropic:claude-opus-4"
  display_name: "Claude Opus 4.5"
  short_name: "claude-opus"
  provider:
    id: "anthropic"
    name: "Anthropic"
    url: "https://anthropic.com"
  version: "4.5.0"
  family: "claude"
  family_rank: 1

level: 4

domains:
  - domain: code
    proficiency: 0.95
    sub_domains:
      - name: "rust"
        proficiency: 0.92
      - name: "python"
        proficiency: 0.95
  - domain: reasoning
    proficiency: 0.95
  - domain: writing
    proficiency: 0.93

languages:
  - code: "en"
    proficiency: 0.98
  - code: "ko"
    proficiency: 0.90

modalities:
  text:
    input: true
    output: true
    max_input_length: 200000
    max_output_length: 32000
    streaming: true
  image:
    input: true
    output: false
    ocr: true
    diagram_understanding: true
  code:
    input: true
    output: true
    execution: false
    languages:
      - language: "rust"
        proficiency: 0.92
      - language: "python"
        proficiency: 0.95

tools:
  - tool_id: "web_search"
    name: "Web Search"
    enabled: true
  - tool_id: "code_execution"
    name: "Code Execution"
    enabled: true

limits:
  max_input_tokens: 200000
  max_output_tokens: 32000
  max_context_window: 200000
  rate_limits:
    requests_per_minute: 50
    tokens_per_minute: 100000

trust:
  confidence_threshold: 0.7
  knowledge_cutoff: "2025-01-01"
  safety:
    content_filtering: true
    nsfw_filter: true
  compliance:
    gdpr_compliant: true

endpoints:
  api:
    url: "https://api.anthropic.com/v1"
    protocol: "https"
    version: "2024-01-01"
    auth: ["api_key"]
  capability: "https://api.anthropic.com/v1/capability"
```

### 6.2 Message Example: Query & Response

```json
{
  "wia_version": "1.0",
  "document_type": "message",
  "message_id": "550e8400-e29b-41d4-a716-446655440001",
  "timestamp": "2025-12-16T10:30:00Z",
  "trace_id": "550e8400-e29b-41d4-a716-446655440000",

  "from": {
    "model_id": "did:wia:llm:user:medical-ai"
  },
  "to": {
    "model_id": "did:wia:llm:anthropic:claude-opus"
  },

  "type": "query",
  "priority": "normal",

  "payload": {
    "intent": "analysis",
    "content": "환자 A의 혈액검사 결과를 분석해주세요. RBC: 4.5, WBC: 11.2, PLT: 180",
    "constraints": {
      "response_format": "json",
      "response_language": "ko",
      "confidence_min": 0.7
    }
  }
}
```

---

## 7. JSON Schema Locations

| Schema | Path |
|--------|------|
| Capability | `schema/capability.schema.json` |
| Message | `schema/message.schema.json` |
| Federation | `schema/federation.schema.json` |
| Task | `schema/task.schema.json` |
| Consensus | `schema/consensus.schema.json` |
| Error Codes | `schema/error-codes.json` |

---

**Document ID**: WIA-LLM-INTEROP-PHASE-1
**Version**: 1.0.0
**Date**: 2025-12-16
**Philosophy**: 홍익인간 (弘益人間) - 모든 AI가 자유롭게 협업할 수 있는 표준
