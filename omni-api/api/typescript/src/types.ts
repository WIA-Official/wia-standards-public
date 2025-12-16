/**
 * WIA-OMNI-API Type Definitions
 *
 * 모든 것을 품는 어머니 API
 */

// ============================================================
// Request Types
// ============================================================

export interface OmniRequest {
  /** 의도 - 가장 중요 */
  intent: string | WiaIntent;

  /** 선택적 힌트들 */
  hints?: RequestHints;

  /** 페이로드 */
  payload?: any;

  /** 컨텍스트 */
  context?: RequestContext;
}

export interface WiaIntent {
  type: 'wia-intent';
  code: string;
}

export interface RequestHints {
  /** 선호 프로토콜 */
  protocol?: Protocol | 'auto';
  /** 선호 포맷 */
  format?: Format | 'auto';
  /** API 버전 */
  version?: string | 'latest' | 'compatible';
  /** 우선순위 */
  priority?: 'speed' | 'accuracy' | 'cost';
  /** 타임아웃 */
  timeout?: number | 'patient';
}

export interface RequestContext {
  /** 세션 ID */
  session_id?: string;
  /** 이전 요청들 */
  previous_requests?: string[];
  /** 사용자 선호도 */
  user_preferences?: Record<string, any>;
}

// ============================================================
// Response Types
// ============================================================

export interface OmniResponse<T = any> {
  /** 성공 여부 */
  success: boolean;
  /** 결과 데이터 */
  data?: T;
  /** 메타 정보 */
  meta: ResponseMeta;
  /** 에러 */
  error?: OmniError;
  /** 관련 리소스 */
  related?: RelatedResources;
  /** 진화 힌트 */
  evolution?: EvolutionHints;
}

export interface ResponseMeta {
  /** 실제 사용된 프로토콜 */
  protocol_used: Protocol;
  /** 실제 사용된 버전 */
  version_used: string;
  /** 실제 사용된 포맷 */
  format_used: Format;
  /** 응답 시간 (ms) */
  latency_ms: number;
  /** 캐시 여부 */
  from_cache: boolean;
  /** 어머니의 조언 */
  suggestions?: string[];
}

export interface OmniError {
  /** 에러 코드 */
  code: string;
  /** 기술적 메시지 */
  message: string;
  /** 위로 메시지 */
  comfort: string;
  /** 원인 */
  cause?: string;
  /** 해결책 */
  solution?: string;
  /** 복구 옵션들 */
  recovery_options: RecoveryOption[];
}

export interface RecoveryOption {
  action: string;
  description: string;
  auto_applicable: boolean;
  confidence: number;
  endpoint?: string;
}

export interface RelatedResources {
  next?: string;
  prev?: string;
  parent?: string;
  children?: string[];
}

export interface EvolutionHints {
  better_approach?: string;
  deprecation_notice?: string;
  new_features?: string[];
}

// ============================================================
// Protocol & Format Types
// ============================================================

export type Protocol =
  | 'rest'
  | 'graphql'
  | 'grpc'
  | 'websocket'
  | 'mqtt'
  | 'intent';

export type Format =
  | 'json'
  | 'xml'
  | 'protobuf'
  | 'msgpack'
  | 'yaml';

export type HttpMethod =
  | 'GET'
  | 'POST'
  | 'PUT'
  | 'PATCH'
  | 'DELETE'
  | 'HEAD'
  | 'OPTIONS';

// ============================================================
// Gateway Types
// ============================================================

export interface GatewayConfig {
  /** 기본 URL */
  baseUrl: string;
  /** 기본 프로토콜 */
  defaultProtocol: Protocol;
  /** 기본 포맷 */
  defaultFormat: Format;
  /** 기본 타임아웃 (ms) */
  defaultTimeout: number;
  /** 자동 재시도 */
  autoRetry: boolean;
  /** 최대 재시도 횟수 */
  maxRetries: number;
  /** 캐시 활성화 */
  cacheEnabled: boolean;
  /** 진화 엔진 활성화 */
  evolutionEnabled: boolean;
}

export interface ResolvedIntent {
  /** 원본 의도 */
  original: string | WiaIntent;
  /** 해석된 액션 */
  action: string;
  /** 추출된 파라미터 */
  params: Record<string, any>;
  /** 해석 확신도 */
  confidence: number;
  /** 가능한 해석들 */
  alternatives?: ResolvedIntent[];
}

export interface Route {
  /** 프로토콜 */
  protocol: Protocol;
  /** 엔드포인트 */
  endpoint: string;
  /** HTTP 메서드 (REST용) */
  method?: HttpMethod;
  /** 헤더 */
  headers?: Record<string, string>;
  /** 본문 */
  body?: any;
}

// ============================================================
// Evolution Types
// ============================================================

export interface EvolutionEngine {
  /** 요청/응답에서 학습 */
  learn(request: OmniRequest, response: OmniResponse, metrics: Metrics): void;
  /** 패턴 감지 */
  detectPatterns(): Pattern[];
  /** 최적화 제안 */
  suggestOptimizations(): Optimization[];
  /** 자동 최적화 적용 */
  applyAutoOptimizations(): AppliedOptimization[];
}

export interface Metrics {
  latency: number;
  success: boolean;
  cacheHit: boolean;
  errorCode?: string;
  timestamp: Date;
}

export interface Pattern {
  type: 'frequency' | 'sequence' | 'co_occurrence' | 'temporal';
  description: string;
  confidence: number;
  data: any;
}

export interface Optimization {
  type: 'cache' | 'bundle' | 'preload' | 'rewrite';
  description: string;
  impact: 'low' | 'medium' | 'high';
  autoApplicable: boolean;
  apply: () => void;
}

export interface AppliedOptimization extends Optimization {
  appliedAt: Date;
  result: 'success' | 'failed' | 'pending';
}

// ============================================================
// Version Management Types
// ============================================================

export interface VersionConfig {
  /** 현재 버전 */
  current: string;
  /** 지원 버전들 */
  supported: string[];
  /** 버전별 변환기 */
  transformers: Map<string, VersionTransformer>;
  /** 절대 깨지 않기 정책 */
  neverBreakPolicy: boolean;
}

export interface VersionTransformer {
  /** 요청 변환 */
  transformRequest(request: any, fromVersion: string, toVersion: string): any;
  /** 응답 변환 */
  transformResponse(response: any, fromVersion: string, toVersion: string): any;
}

// ============================================================
// Error Types
// ============================================================

export class OmniApiError extends Error {
  constructor(
    message: string,
    public code: string,
    public comfort: string,
    public recoveryOptions: RecoveryOption[] = []
  ) {
    super(message);
    this.name = 'OmniApiError';
  }
}

export class IntentParseError extends OmniApiError {
  constructor(message: string) {
    super(
      message,
      'INTENT_PARSE_ERROR',
      '의도를 이해하기 어려웠어요. 다시 한번 말씀해주시겠어요?',
      [
        {
          action: 'simplify',
          description: '더 간단하게 표현해보세요',
          auto_applicable: false,
          confidence: 0.8,
        },
      ]
    );
  }
}

export class ProtocolError extends OmniApiError {
  constructor(protocol: string, message: string) {
    super(
      message,
      'PROTOCOL_ERROR',
      `${protocol} 연결에 문제가 있어요. 제가 다른 방법을 찾아볼게요.`,
      [
        {
          action: 'fallback',
          description: '다른 프로토콜로 시도',
          auto_applicable: true,
          confidence: 0.9,
        },
      ]
    );
  }
}

export class VersionError extends OmniApiError {
  constructor(requested: string, available: string[]) {
    super(
      `Version ${requested} not found`,
      'VERSION_ERROR',
      '요청하신 버전을 찾을 수 없어요. 비슷한 버전으로 안내해드릴까요?',
      available.map((v) => ({
        action: 'use_version',
        description: `버전 ${v} 사용`,
        auto_applicable: true,
        confidence: 0.95,
        endpoint: `?version=${v}`,
      }))
    );
  }
}

// ============================================================
// Utility Types
// ============================================================

export type DeepPartial<T> = {
  [P in keyof T]?: T[P] extends object ? DeepPartial<T[P]> : T[P];
};

export interface CacheEntry<T> {
  data: T;
  timestamp: Date;
  ttl: number;
  hits: number;
}

export interface RateLimitInfo {
  remaining: number;
  total: number;
  resetAt: Date;
  level: 'normal' | 'warning' | 'slow' | 'queued' | 'paused';
}
