# WIA-OMNI-API v1.0 Specification

> 모든 것을 품는 어머니 API
>
> The Mother of All APIs
>
> 홍익인간 (弘益人間) - Benefit All Humanity

## Abstract

WIA-OMNI-API는 모든 API 패러다임을 품고, 자식(클라이언트)이 편하도록 모든 것을 알아서 처리하는 메타 API 표준입니다.

단군할아버지의 홍익인간 정신으로, 21세기 인류를 위해 만들어졌습니다.

## 1. Introduction

### 1.1 현재 API의 문제

```
REST API      → 동사가 제한적, 오버페칭
GraphQL       → 복잡한 쿼리, 캐싱 어려움
gRPC          → 브라우저 지원 제한
WebSocket     → 상태 관리 복잡
MQTT          → IoT 특화, 범용성 부족

결과:
- 각각 따로 배워야 함
- 호환 안 됨
- 버전 바뀌면 클라이언트 수정
- 복잡함을 클라이언트가 감당
```

### 1.2 어머니의 철학

```
어머니는 자식이 뭘 원하는지만 들으면
나머지는 다 알아서 해준다.

어머니는 자식에게 복잡함을 전가하지 않는다.
어머니는 자식이 변하라고 강요하지 않는다.
어머니는 모든 자식을 품는다.
```

### 1.3 WIA-OMNI-API의 원칙

1. **Embrace All (모두 품기)**: 모든 프로토콜, 포맷, 버전을 품는다
2. **Adapt to Child (자식에게 맞추기)**: 서버가 클라이언트에 맞춘다
3. **Self-Evolving (자기 진화)**: 사용 패턴을 학습하고 진화한다
4. **Never Break (절대 깨지 않기)**: 하위 호환성을 절대 깨지 않는다
5. **Intent-Based (의도 기반)**: WIA-INTENT와 통합된다

## 2. Architecture

### 2.1 Overall Structure

```
┌─────────────────────────────────────────────────────┐
│                  Clients (Children)                  │
│  Browser │ Mobile │ IoT │ Server │ AI │ Future...   │
└─────────────────────┬───────────────────────────────┘
                      │ (Any Protocol, Any Format)
                      ▼
┌─────────────────────────────────────────────────────┐
│               WIA-OMNI-API Gateway                   │
│                    (Mother)                          │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │           Protocol Embracer                  │   │
│  │   REST ↔ GraphQL ↔ gRPC ↔ WS ↔ MQTT ↔ ???  │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │           Intent Interpreter                 │   │
│  │        (WIA-INTENT Integration)              │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │           Version Harmonizer                 │   │
│  │     (Automatic Backward Compatibility)       │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │           Format Transformer                 │   │
│  │      JSON ↔ XML ↔ Protobuf ↔ MsgPack        │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
│  ┌─────────────────────────────────────────────┐   │
│  │           Self-Evolution Engine              │   │
│  │    (Learn, Adapt, Optimize, Predict)        │   │
│  └─────────────────────────────────────────────┘   │
│                                                      │
└─────────────────────┬───────────────────────────────┘
                      ▼
┌─────────────────────────────────────────────────────┐
│               Backend Services                       │
└─────────────────────────────────────────────────────┘
```

### 2.2 Core Components

#### 2.2.1 Protocol Embracer (프로토콜 포용기)

```yaml
protocol_embracer:
  supported:
    - rest:
        methods: [GET, POST, PUT, PATCH, DELETE, HEAD, OPTIONS]
        versions: [1.0, 1.1, 2.0, 3.0]
    - graphql:
        operations: [query, mutation, subscription]
        versions: [June2018, October2021, Draft]
    - grpc:
        modes: [unary, server_stream, client_stream, bidi_stream]
    - websocket:
        protocols: [ws, wss]
        subprotocols: [json, binary, custom]
    - mqtt:
        versions: [3.1, 3.1.1, 5.0]
        qos: [0, 1, 2]
    - future:
        auto_detect: true
        learn_new_protocols: true

  auto_selection:
    based_on:
      - client_capability
      - network_condition
      - request_type
      - performance_requirement
```

#### 2.2.2 Intent Interpreter (의도 해석기)

```yaml
intent_interpreter:
  # WIA-INTENT 언어 통합
  wia_intent_integration: true

  # 자연어도 이해
  natural_language: true

  # 의도 → API 호출 변환
  translation:
    input: "사용자 목록 줘"
    output:
      rest: "GET /users"
      graphql: "query { users { id name } }"
      grpc: "UserService.ListUsers()"

  # 모호한 의도 처리
  ambiguity_handling:
    strategy: ask_or_infer
    confidence_threshold: 0.8
```

#### 2.2.3 Version Harmonizer (버전 조화기)

```yaml
version_harmonizer:
  principle: "절대 자식을 깨지 않는다"

  strategies:
    # 필드 추가 - 기존 클라이언트 영향 없음
    field_addition:
      action: add_with_default
      notify: false

    # 필드 제거 - 기존 클라이언트 계속 지원
    field_removal:
      action: deprecate_then_null
      deprecation_period: 2_years
      notify: true

    # 필드 변경 - 자동 변환
    field_rename:
      action: alias_both
      old_name: supported_forever
      new_name: preferred

    # 타입 변경 - 자동 캐스팅
    type_change:
      action: auto_convert
      preserve_semantics: true

  versioning:
    scheme: semantic
    header: "X-WIA-API-Version"
    default: latest_compatible
    sunset_policy: never_break
```

#### 2.2.4 Format Transformer (포맷 변환기)

```yaml
format_transformer:
  supported:
    - json
    - xml
    - protobuf
    - msgpack
    - yaml
    - csv
    - html
    - custom

  auto_detection:
    from_header: "Accept, Content-Type"
    from_extension: ".json, .xml, .pb"
    from_content: true

  transformation:
    lossless: preferred
    fallback: best_effort
    notify_on_loss: true

  optimization:
    compress: auto  # gzip, br, zstd
    minify: based_on_client
    stream: when_large
```

#### 2.2.5 Self-Evolution Engine (자기 진화 엔진)

```yaml
self_evolution:
  learning:
    from:
      - request_patterns
      - error_frequencies
      - latency_measurements
      - client_preferences
      - usage_trends

    methods:
      - pattern_recognition
      - anomaly_detection
      - predictive_modeling
      - reinforcement_learning

  adaptation:
    # 자주 요청되는 데이터 자동 캐싱
    auto_cache:
      trigger: frequency_threshold
      strategy: predictive

    # 자주 함께 요청되는 데이터 자동 번들링
    auto_bundle:
      trigger: co_occurrence
      strategy: preemptive

    # 느린 엔드포인트 자동 최적화
    auto_optimize:
      trigger: latency_threshold
      strategy: query_rewriting

    # 에러 패턴 자동 수정 제안
    auto_heal:
      trigger: error_pattern
      strategy: suggest_and_apply

  boundaries:
    never_change: contract_semantics
    always_preserve: backward_compatibility
    human_approval: breaking_changes
```

## 3. Request/Response Model

### 3.1 Universal Request Format

```typescript
interface OmniRequest {
  // 의도 (가장 중요)
  intent: string | WiaIntent;

  // 선택적 힌트들 - 없어도 어머니가 알아서
  hints?: {
    protocol?: 'rest' | 'graphql' | 'grpc' | 'auto';
    format?: 'json' | 'xml' | 'auto';
    version?: string | 'latest' | 'compatible';
    priority?: 'speed' | 'accuracy' | 'cost';
    timeout?: number | 'patient';  // 어머니는 기다려줌
  };

  // 페이로드 - 어떤 형태든 OK
  payload?: any;

  // 컨텍스트 - 대화의 맥락
  context?: {
    session_id?: string;
    previous_requests?: string[];
    user_preferences?: Record<string, any>;
  };
}
```

### 3.2 Universal Response Format

```typescript
interface OmniResponse {
  // 성공 여부
  success: boolean;

  // 결과 데이터
  data?: any;

  // 메타 정보
  meta: {
    // 실제 사용된 프로토콜
    protocol_used: string;
    // 실제 사용된 버전
    version_used: string;
    // 실제 사용된 포맷
    format_used: string;
    // 응답 시간
    latency_ms: number;
    // 캐시 여부
    from_cache: boolean;
    // 어머니의 조언
    suggestions?: string[];
  };

  // 에러 (있다면)
  error?: {
    code: string;
    message: string;
    // 어머니의 위로와 해결책
    comfort: string;
    recovery_options: RecoveryOption[];
  };

  // 관련 리소스 (HATEOAS 스타일, 자동)
  related?: {
    next?: string;
    prev?: string;
    parent?: string;
    children?: string[];
  };

  // 진화 힌트
  evolution?: {
    // 더 좋은 방법이 있을 때
    better_approach?: string;
    // deprecated 예정
    deprecation_notice?: string;
    // 새로운 기능 알림
    new_features?: string[];
  };
}

interface RecoveryOption {
  action: string;
  description: string;
  auto_applicable: boolean;
  confidence: number;
}
```

### 3.3 Intent-Based Request Examples

#### 간단한 요청
```typescript
// 클라이언트가 이렇게만 보내면
{
  intent: "get user info",
  payload: { user_id: 123 }
}

// 어머니가 알아서 처리
// → REST: GET /users/123
// → GraphQL: query { user(id: 123) { ... } }
// → gRPC: UserService.GetUser({ id: 123 })
```

#### WIA-INTENT 연동
```typescript
{
  intent: {
    type: "wia-intent",
    code: `
      intent GetUserInfo {
        given: user_id
        want: user_profile
        constraints {
          include: [name, email, avatar]
          exclude: [password, internal_notes]
          freshness: <= 5.minutes
        }
        certainty: >= 0.95
      }
    `
  }
}
```

#### 자연어 요청
```typescript
{
  intent: "123번 사용자의 최근 주문 3개만 빨리 줘",
  hints: {
    priority: "speed"
  }
}
```

## 4. Protocol Bridging

### 4.1 REST to GraphQL Bridge

```yaml
bridge_rest_to_graphql:
  # REST 요청이 들어오면 GraphQL로 변환
  example:
    input:
      method: GET
      path: /users/123
      query: fields=name,email

    output:
      query: |
        query {
          user(id: 123) {
            name
            email
          }
        }

  # 자동 최적화
  optimization:
    # N+1 문제 자동 해결
    auto_batch: true
    # 필요한 필드만 요청
    field_selection: from_query_params
```

### 4.2 GraphQL to REST Bridge

```yaml
bridge_graphql_to_rest:
  # GraphQL 요청을 REST로 분해
  example:
    input:
      query: |
        query {
          user(id: 123) {
            name
            orders {
              id
              total
            }
          }
        }

    output:
      - GET /users/123?fields=name
      - GET /users/123/orders?fields=id,total

  # 병렬 실행 및 조합
  execution:
    parallel: true
    merge_strategy: deep_merge
```

### 4.3 Any to Any Bridge

```yaml
universal_bridge:
  # 어떤 프로토콜이든 어떤 프로토콜로든
  matrix:
    from: [rest, graphql, grpc, websocket, mqtt, intent]
    to: [rest, graphql, grpc, websocket, mqtt, intent]

  # 자동 변환 규칙 학습
  learning:
    from_usage: true
    from_documentation: true
    from_examples: true
```

## 5. Version Management

### 5.1 Never-Break Policy

```yaml
never_break_policy:
  # 버전 변화에도 기존 클라이언트 절대 깨지지 않음

  guarantees:
    - old_requests_always_work: true
    - old_responses_always_parseable: true
    - no_forced_migration: true

  mechanisms:
    # 필드가 사라져도
    removed_field:
      strategy: return_null_with_warning
      duration: forever

    # 타입이 바뀌어도
    changed_type:
      strategy: return_both_formats
      old_field: "amount" # string "100"
      new_field: "amount_v2" # number 100

    # 구조가 바뀌어도
    changed_structure:
      strategy: transform_on_the_fly
      old_format: { user_name: "kim" }
      new_format: { user: { name: "kim" } }
```

### 5.2 Transparent Upgrade

```yaml
transparent_upgrade:
  # 클라이언트 모르게 업그레이드

  process:
    1. detect_client_version: from_header_or_behavior
    2. serve_appropriate_version: automatically
    3. suggest_upgrade: gently_in_response
    4. never_force: true

  example:
    # 클라이언트 v1 요청
    request:
      GET /api/users/123
      X-API-Version: 1.0

    # 내부적으로 v3 호출, v1 형식으로 응답
    response:
      # v3에서 추가된 필드는 제외
      # v1에서 사용하던 형식 유지
      { "user_name": "Kim" }  # v3는 { user: { name: "Kim" } }

    # 업그레이드 제안 (강제 아님)
    headers:
      X-WIA-Suggestion: "v3 available with better features"
      X-WIA-Upgrade-Guide: "https://..."
```

## 6. Error Handling

### 6.1 Compassionate Error Response

```yaml
compassionate_errors:
  principle: "에러도 어머니처럼 - 혼내지 않고 도와준다"

  structure:
    code: string          # 기계용
    message: string       # 기술자용
    comfort: string       # 인간용 (위로)
    cause: string         # 왜 발생했는지
    solution: string      # 어떻게 해결하는지
    auto_fix: boolean     # 자동 수정 가능한지
    alternatives: []      # 대안들
```

### 6.2 Error Examples

```json
{
  "success": false,
  "error": {
    "code": "USER_NOT_FOUND",
    "message": "User with ID 123 does not exist",
    "comfort": "걱정 마세요, 흔한 일이에요.",
    "cause": "요청하신 사용자 ID가 시스템에 없습니다.",
    "solution": "사용자 ID를 확인해주세요. 최근 삭제되었을 수도 있어요.",
    "auto_fix": false,
    "alternatives": [
      {
        "action": "search_similar",
        "description": "비슷한 이름의 사용자 검색",
        "endpoint": "/users/search?name_like=..."
      },
      {
        "action": "list_recent",
        "description": "최근 활성 사용자 목록",
        "endpoint": "/users/recent"
      }
    ]
  }
}
```

### 6.3 Auto-Recovery

```yaml
auto_recovery:
  strategies:
    # 타임아웃 → 자동 재시도
    timeout:
      action: retry_with_backoff
      max_retries: 3
      notify_client: after_all_failed

    # 서버 에러 → 대체 서버
    server_error:
      action: failover_to_replica
      notify_client: transparently

    # 인증 만료 → 자동 갱신 시도
    auth_expired:
      action: try_refresh_token
      notify_client: if_failed

    # 형식 오류 → 자동 수정 시도
    format_error:
      action: try_auto_correct
      confidence_threshold: 0.9
      notify_client: with_correction
```

## 7. Self-Evolution

### 7.1 Learning from Usage

```yaml
usage_learning:
  collect:
    - request_patterns
    - response_times
    - error_frequencies
    - client_preferences
    - data_access_patterns

  analyze:
    - frequent_queries: for_caching
    - co_occurring_requests: for_bundling
    - slow_paths: for_optimization
    - error_patterns: for_prevention

  apply:
    - auto_create_shortcuts: true
    - auto_optimize_queries: true
    - auto_suggest_batching: true
    - auto_predict_needs: true
```

### 7.2 Predictive Optimization

```yaml
predictive_optimization:
  # 사용 패턴 예측
  prediction:
    # 월요일 아침 9시에 대시보드 요청 급증
    pattern: "monday_morning_dashboard"
    action: pre_warm_cache

    # 특정 사용자는 항상 A 다음에 B 요청
    pattern: "sequential_ab"
    action: preload_b_when_a_requested

  # 자동 최적화
  optimization:
    # 자주 함께 요청되는 데이터 자동 조인
    auto_join:
      threshold: 70%_co_occurrence
      action: create_composite_endpoint

    # 잘 안 쓰이는 필드 lazy loading
    auto_lazy:
      threshold: 10%_usage
      action: exclude_from_default
```

### 7.3 API Evolution Suggestions

```yaml
evolution_suggestions:
  # API 개선 제안
  to_developers:
    - "users 엔드포인트와 orders 엔드포인트가 95%의 요청에서 함께 호출됩니다.
       users_with_orders 복합 엔드포인트를 고려해보세요."

    - "password 필드가 절대 요청되지 않습니다.
       응답에서 기본 제외하면 성능이 개선됩니다."

    - "v1 API를 아직 사용하는 클라이언트가 3개 있습니다.
       연락처: [...]. 마이그레이션 지원을 제안해보세요."

  # 클라이언트에게 부드럽게 제안
  to_clients:
    - "새로운 batch 엔드포인트를 사용하면
       현재 5번의 요청을 1번으로 줄일 수 있어요."

    - "GraphQL을 지원합니다.
       현재 요청 패턴에 더 적합할 수 있어요."
```

## 8. Security (Protective Mother)

### 8.1 Security Principles

```yaml
security:
  principle: "어머니는 자식을 보호한다"

  protection:
    # 외부 위협으로부터
    from_external:
      - ddos_protection: auto
      - injection_prevention: always
      - authentication: flexible_but_strong

    # 실수로부터
    from_mistakes:
      - rate_limiting: gentle_with_warning
      - dangerous_operations: confirm_required
      - data_loss: auto_backup_before

    # 과도한 비용으로부터
    from_overcost:
      - usage_alerts: proactive
      - budget_limits: configurable
      - optimization_suggestions: automatic
```

### 8.2 Gentle Rate Limiting

```yaml
gentle_rate_limiting:
  # 강제 차단 대신 점진적 대응

  levels:
    normal:
      action: serve_normally

    approaching_limit:
      action: serve_with_warning
      header: "X-WIA-Gentle-Warning: You're making many requests. Everything OK?"

    at_limit:
      action: serve_slower
      delay: progressive
      message: "잠깐 쉬어가세요. 제가 조금 느리게 응답할게요."

    over_limit:
      action: queue_requests
      message: "요청이 많아요. 순서대로 처리할게요. 기다려주세요."

    way_over_limit:
      action: temporary_pause
      message: "잠시 쉬었다가 다시 시도해주세요. 제가 준비할게요."
      retry_after: 60
```

## 9. Integration with WIA Standards

### 9.1 WIA-INTENT Integration

```yaml
wia_intent_integration:
  # 의도 기반 요청 직접 지원
  accept_intent: true

  example:
    request:
      intent: |
        intent GetDashboard {
          want: user_dashboard
          constraints {
            freshness: <= 1.minute
            include: [stats, recent_activity, notifications]
          }
          certainty: >= 0.9
        }

    processing:
      1. parse_intent: WIA-INTENT parser
      2. resolve_to_api: internal mapping
      3. execute: optimal_path
      4. verify_constraints: certainty check
```

### 9.2 WIA-LLM-INTEROP Integration

```yaml
wia_llm_integration:
  # AI 에이전트가 API 호출 시

  ai_friendly:
    - natural_language_support: true
    - intent_based_calls: true
    - auto_capability_discovery: true
    - error_explanation: ai_optimized

  capability_declaration:
    # WIA-LLM-CAPABILITY 형식으로 API 능력 선언
    format: wia_llm_capability
    auto_generate: true
```

### 9.3 WIA-PQ-CRYPTO Integration

```yaml
wia_pq_crypto:
  # 양자내성 암호화 지원

  encryption:
    algorithm: WIA-PQ-CRYPTO compliant
    key_exchange: CRYSTALS-Kyber
    signatures: CRYSTALS-Dilithium

  future_proof:
    quantum_safe: true
    upgrade_path: automatic
```

## 10. Implementation Guide

### 10.1 Gateway Implementation

```typescript
interface OmniGateway {
  // 모든 요청의 단일 진입점
  handle(request: OmniRequest): Promise<OmniResponse>;

  // 프로토콜 감지
  detectProtocol(request: RawRequest): Protocol;

  // 의도 해석
  interpretIntent(intent: string | WiaIntent): ResolvedIntent;

  // 최적 경로 결정
  determineRoute(intent: ResolvedIntent): Route;

  // 실행 및 응답 변환
  executeAndTransform(route: Route, request: OmniRequest): Promise<OmniResponse>;
}
```

### 10.2 Evolution Engine Implementation

```typescript
interface EvolutionEngine {
  // 학습
  learn(request: OmniRequest, response: OmniResponse, metrics: Metrics): void;

  // 패턴 감지
  detectPatterns(): Pattern[];

  // 최적화 제안
  suggestOptimizations(): Optimization[];

  // 자동 적용 (안전한 것만)
  applyAutoOptimizations(): AppliedOptimization[];

  // 개발자 승인 대기
  getPendingApprovals(): Optimization[];
}
```

## 11. Examples

### 11.1 Simple Usage

```typescript
// 클라이언트 - 이게 전부입니다
const response = await omni.request({
  intent: "get user 123"
});

// 어머니가 알아서:
// - 프로토콜 선택
// - 포맷 결정
// - 버전 호환성 처리
// - 캐싱
// - 에러 복구
```

### 11.2 Advanced Usage

```typescript
const response = await omni.request({
  intent: {
    type: "wia-intent",
    code: `
      intent GetUserDashboard {
        given: user_id
        want: {
          profile: basic_info
          stats: summary
          activity: recent_10
        }
        constraints {
          latency: <= 200ms
          freshness: <= 5min
        }
        fallback {
          if timeout: return cached_version
          if partial_failure: return available_data
        }
      }
    `
  },
  payload: { user_id: 123 },
  hints: {
    priority: "speed"
  }
});
```

## 12. Migration Guide

### 12.1 From REST

```yaml
migration_from_rest:
  # 기존 REST API를 WIA-OMNI-API로 감싸기

  before:
    client: fetch("/api/users/123")

  after:
    # 방법 1: 그대로 사용 (어머니가 알아서)
    client: omni.request({ intent: "GET /api/users/123" })

    # 방법 2: 의도 기반으로
    client: omni.request({ intent: "get user 123" })

  benefit:
    - 기존 코드 수정 최소화
    - 점진적 마이그레이션
    - 즉시 혜택 (캐싱, 최적화, 에러 복구)
```

### 12.2 Gradual Adoption

```yaml
gradual_adoption:
  phase_1:
    action: wrap_existing_apis
    effort: minimal
    benefit: immediate_improvements

  phase_2:
    action: add_intent_endpoints
    effort: moderate
    benefit: better_developer_experience

  phase_3:
    action: enable_evolution
    effort: configuration
    benefit: self_optimization

  phase_4:
    action: full_intent_based
    effort: optional
    benefit: maximum_flexibility
```

## 13. Governance

### 13.1 Evolution Boundaries

```yaml
evolution_boundaries:
  # 자동 진화의 한계

  auto_allowed:
    - caching_strategy
    - query_optimization
    - format_transformation
    - error_recovery

  requires_human_approval:
    - new_endpoints
    - breaking_changes
    - security_changes
    - pricing_changes

  never_auto:
    - data_deletion
    - access_control_changes
    - compliance_related
```

### 13.2 Audit Trail

```yaml
audit_trail:
  # 모든 진화 기록

  logged:
    - what_changed
    - why_changed
    - who_approved (if required)
    - impact_metrics
    - rollback_available

  retention: forever
  searchable: true
  exportable: true
```

---

## Appendix A: Philosophy

```
단군할아버지의 홍익인간 (弘益人間)

- 널리 인간을 이롭게 하라
- 어머니처럼 모든 것을 품어라
- 자식이 편하면 그것이 답이다
- 강요하지 말고 도와줘라
- 절대 깨뜨리지 말라

이것이 WIA-OMNI-API의 정신입니다.
```

---

## Appendix B: Comparison

| Feature | REST | GraphQL | gRPC | WIA-OMNI-API |
|---------|------|---------|------|--------------|
| Protocol | HTTP only | HTTP only | HTTP/2 | Any |
| Format | Usually JSON | JSON | Protobuf | Any |
| Versioning | Manual | Schema | Proto | Auto |
| Breaking Changes | Possible | Possible | Possible | Never |
| Self-Evolution | No | No | No | Yes |
| Intent-Based | No | No | No | Yes |
| Error Recovery | Manual | Manual | Manual | Auto |

---

**WIA-OMNI-API v1.0**
**World Certification Industry Association**
**홍익인간 (弘益人間) - Benefit All Humanity**

*어머니처럼 모든 것을 품는 API*
*단군할아버지가 21세기에 오셨다면 만들었을 그것*
