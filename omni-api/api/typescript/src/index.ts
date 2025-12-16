/**
 * WIA-OMNI-API SDK
 *
 * 모든 것을 품는 어머니 API
 * The Mother of All APIs
 *
 * 홍익인간 (弘益人間) - Benefit All Humanity
 *
 * @version 1.0.0
 * @author World Certification Industry Association (WIA)
 */

export * from './types';

import {
  OmniRequest,
  OmniResponse,
  OmniError,
  GatewayConfig,
  Protocol,
  Format,
  ResolvedIntent,
  Route,
  HttpMethod,
  Metrics,
  Pattern,
  Optimization,
  RecoveryOption,
  WiaIntent,
  IntentParseError,
  ProtocolError,
  CacheEntry,
  RateLimitInfo,
} from './types';

// ============================================================
// Default Configuration
// ============================================================

const DEFAULT_CONFIG: GatewayConfig = {
  baseUrl: '',
  defaultProtocol: 'rest',
  defaultFormat: 'json',
  defaultTimeout: 30000,
  autoRetry: true,
  maxRetries: 3,
  cacheEnabled: true,
  evolutionEnabled: true,
};

// ============================================================
// Intent Interpreter
// ============================================================

class IntentInterpreter {
  private patterns: Map<RegExp, (match: RegExpMatchArray) => ResolvedIntent> = new Map();

  constructor() {
    this.initializePatterns();
  }

  private initializePatterns(): void {
    // "get user 123" -> GET /users/123
    this.patterns.set(
      /^get\s+(\w+)\s+(\d+)$/i,
      (match) => ({
        original: match[0],
        action: 'get_by_id',
        params: { resource: match[1], id: match[2] },
        confidence: 0.95,
      })
    );

    // "list users" -> GET /users
    this.patterns.set(
      /^list\s+(\w+)s?$/i,
      (match) => ({
        original: match[0],
        action: 'list',
        params: { resource: match[1] },
        confidence: 0.9,
      })
    );

    // "create user" -> POST /users
    this.patterns.set(
      /^create\s+(\w+)$/i,
      (match) => ({
        original: match[0],
        action: 'create',
        params: { resource: match[1] },
        confidence: 0.9,
      })
    );

    // "update user 123" -> PUT /users/123
    this.patterns.set(
      /^update\s+(\w+)\s+(\d+)$/i,
      (match) => ({
        original: match[0],
        action: 'update',
        params: { resource: match[1], id: match[2] },
        confidence: 0.9,
      })
    );

    // "delete user 123" -> DELETE /users/123
    this.patterns.set(
      /^delete\s+(\w+)\s+(\d+)$/i,
      (match) => ({
        original: match[0],
        action: 'delete',
        params: { resource: match[1], id: match[2] },
        confidence: 0.9,
      })
    );

    // Korean support: "123번 사용자 정보"
    this.patterns.set(
      /^(\d+)번?\s*(\w+)\s*(정보|목록|데이터)?/,
      (match) => ({
        original: match[0],
        action: 'get_by_id',
        params: { id: match[1], resource: match[2] },
        confidence: 0.85,
      })
    );

    // Natural language: "사용자 목록 줘"
    this.patterns.set(
      /^(\w+)\s*(목록|리스트)\s*(줘|주세요|보여줘)?$/,
      (match) => ({
        original: match[0],
        action: 'list',
        params: { resource: match[1] },
        confidence: 0.85,
      })
    );
  }

  interpret(intent: string | WiaIntent): ResolvedIntent {
    // WIA-INTENT 코드인 경우
    if (typeof intent === 'object' && intent.type === 'wia-intent') {
      return this.interpretWiaIntent(intent);
    }

    // 문자열 의도 해석
    const intentStr = intent.toString().trim().toLowerCase();

    for (const [pattern, resolver] of this.patterns) {
      const match = intentStr.match(pattern);
      if (match) {
        return resolver(match);
      }
    }

    // 해석 실패 시
    return {
      original: intent,
      action: 'unknown',
      params: { raw: intent },
      confidence: 0.3,
    };
  }

  private interpretWiaIntent(intent: WiaIntent): ResolvedIntent {
    // 간단한 WIA-INTENT 파싱
    const code = intent.code;
    const intentMatch = code.match(/intent\s+(\w+)/);
    const wantMatch = code.match(/want:\s*(\w+)/);
    const givenMatch = code.match(/given:\s*(\w+)/);

    return {
      original: intent,
      action: intentMatch ? intentMatch[1] : 'unknown',
      params: {
        want: wantMatch ? wantMatch[1] : undefined,
        given: givenMatch ? givenMatch[1] : undefined,
      },
      confidence: 0.9,
    };
  }
}

// ============================================================
// Protocol Embracer
// ============================================================

class ProtocolEmbracer {
  async execute(route: Route, payload?: any): Promise<any> {
    switch (route.protocol) {
      case 'rest':
        return this.executeRest(route, payload);
      case 'graphql':
        return this.executeGraphQL(route, payload);
      default:
        return this.executeRest(route, payload);
    }
  }

  private async executeRest(route: Route, payload?: any): Promise<any> {
    const options: RequestInit = {
      method: route.method || 'GET',
      headers: {
        'Content-Type': 'application/json',
        ...route.headers,
      },
    };

    if (payload && !['GET', 'HEAD'].includes(route.method || 'GET')) {
      options.body = JSON.stringify(payload);
    }

    const response = await fetch(route.endpoint, options);
    return response.json();
  }

  private async executeGraphQL(route: Route, payload?: any): Promise<any> {
    const options: RequestInit = {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...route.headers,
      },
      body: JSON.stringify({
        query: payload?.query,
        variables: payload?.variables,
      }),
    };

    const response = await fetch(route.endpoint, options);
    return response.json();
  }

  selectBestProtocol(intent: ResolvedIntent, hints?: OmniRequest['hints']): Protocol {
    // 힌트가 있으면 존중
    if (hints?.protocol && hints.protocol !== 'auto') {
      return hints.protocol;
    }

    // 의도에 따른 최적 프로토콜 선택
    if (intent.action === 'subscribe' || intent.action === 'stream') {
      return 'websocket';
    }

    if (intent.action.includes('batch')) {
      return 'graphql';
    }

    return 'rest';
  }
}

// ============================================================
// Version Harmonizer
// ============================================================

class VersionHarmonizer {
  private currentVersion = '3.0';
  private supportedVersions = ['1.0', '2.0', '3.0'];

  harmonize(
    request: OmniRequest,
    requestedVersion?: string
  ): { version: string; transformed: OmniRequest } {
    const version = requestedVersion || this.currentVersion;

    // 지원되는 버전인지 확인
    if (!this.supportedVersions.includes(version)) {
      // 가장 가까운 버전 찾기
      const closest = this.findClosestVersion(version);
      return {
        version: closest,
        transformed: this.transformRequest(request, version, closest),
      };
    }

    return { version, transformed: request };
  }

  private findClosestVersion(requested: string): string {
    // 간단한 구현: 최신 호환 버전 반환
    return this.currentVersion;
  }

  private transformRequest(
    request: OmniRequest,
    fromVersion: string,
    toVersion: string
  ): OmniRequest {
    // 버전 변환 로직 (실제로는 더 복잡함)
    return request;
  }

  transformResponse(response: any, fromVersion: string, toVersion: string): any {
    // 응답 변환 로직
    return response;
  }
}

// ============================================================
// Cache Manager
// ============================================================

class CacheManager {
  private cache: Map<string, CacheEntry<any>> = new Map();
  private defaultTTL = 60000; // 1분

  get<T>(key: string): T | undefined {
    const entry = this.cache.get(key);
    if (!entry) return undefined;

    // TTL 확인
    if (Date.now() - entry.timestamp.getTime() > entry.ttl) {
      this.cache.delete(key);
      return undefined;
    }

    entry.hits++;
    return entry.data as T;
  }

  set<T>(key: string, data: T, ttl?: number): void {
    this.cache.set(key, {
      data,
      timestamp: new Date(),
      ttl: ttl || this.defaultTTL,
      hits: 0,
    });
  }

  generateKey(request: OmniRequest): string {
    return JSON.stringify({
      intent: request.intent,
      payload: request.payload,
    });
  }

  clear(): void {
    this.cache.clear();
  }

  getStats(): { size: number; totalHits: number } {
    let totalHits = 0;
    this.cache.forEach((entry) => (totalHits += entry.hits));
    return { size: this.cache.size, totalHits };
  }
}

// ============================================================
// Evolution Engine (Simple Version)
// ============================================================

class SimpleEvolutionEngine {
  private patterns: Pattern[] = [];
  private requestHistory: Array<{ request: OmniRequest; metrics: Metrics }> = [];

  learn(request: OmniRequest, response: OmniResponse, metrics: Metrics): void {
    this.requestHistory.push({ request, metrics });

    // 최근 100개만 유지
    if (this.requestHistory.length > 100) {
      this.requestHistory.shift();
    }

    // 패턴 분석
    this.analyzePatterns();
  }

  private analyzePatterns(): void {
    // 빈도 분석
    const intentFrequency = new Map<string, number>();

    for (const { request } of this.requestHistory) {
      const key =
        typeof request.intent === 'string'
          ? request.intent
          : JSON.stringify(request.intent);
      intentFrequency.set(key, (intentFrequency.get(key) || 0) + 1);
    }

    // 고빈도 패턴 감지
    for (const [intent, count] of intentFrequency) {
      if (count > 10) {
        this.patterns.push({
          type: 'frequency',
          description: `Intent "${intent}" is frequently used (${count} times)`,
          confidence: Math.min(count / 20, 1),
          data: { intent, count },
        });
      }
    }
  }

  detectPatterns(): Pattern[] {
    return this.patterns;
  }

  suggestOptimizations(): Optimization[] {
    const suggestions: Optimization[] = [];

    for (const pattern of this.patterns) {
      if (pattern.type === 'frequency' && pattern.confidence > 0.7) {
        suggestions.push({
          type: 'cache',
          description: `Consider caching "${pattern.data.intent}" - high frequency`,
          impact: 'high',
          autoApplicable: true,
          apply: () => {
            console.log('Auto-caching enabled for:', pattern.data.intent);
          },
        });
      }
    }

    return suggestions;
  }
}

// ============================================================
// Main Gateway Class
// ============================================================

export class OmniGateway {
  private config: GatewayConfig;
  private intentInterpreter: IntentInterpreter;
  private protocolEmbracer: ProtocolEmbracer;
  private versionHarmonizer: VersionHarmonizer;
  private cacheManager: CacheManager;
  private evolutionEngine: SimpleEvolutionEngine;

  constructor(config: Partial<GatewayConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.intentInterpreter = new IntentInterpreter();
    this.protocolEmbracer = new ProtocolEmbracer();
    this.versionHarmonizer = new VersionHarmonizer();
    this.cacheManager = new CacheManager();
    this.evolutionEngine = new SimpleEvolutionEngine();
  }

  /**
   * 어머니에게 요청하기
   */
  async request<T = any>(request: OmniRequest): Promise<OmniResponse<T>> {
    const startTime = Date.now();

    try {
      // 1. 캐시 확인
      if (this.config.cacheEnabled) {
        const cacheKey = this.cacheManager.generateKey(request);
        const cached = this.cacheManager.get<T>(cacheKey);
        if (cached) {
          return this.buildResponse(cached, {
            protocol_used: 'rest',
            version_used: '3.0',
            format_used: 'json',
            latency_ms: Date.now() - startTime,
            from_cache: true,
          });
        }
      }

      // 2. 의도 해석
      const resolved = this.intentInterpreter.interpret(request.intent);
      if (resolved.confidence < 0.5) {
        throw new IntentParseError(
          `Could not understand intent: ${request.intent}`
        );
      }

      // 3. 버전 조화
      const { version, transformed } = this.versionHarmonizer.harmonize(
        request,
        request.hints?.version as string
      );

      // 4. 최적 프로토콜 선택
      const protocol = this.protocolEmbracer.selectBestProtocol(
        resolved,
        request.hints
      );

      // 5. 라우트 결정
      const route = this.determineRoute(resolved, protocol);

      // 6. 실행
      const data = await this.executeWithRetry(route, request.payload);

      // 7. 응답 구성
      const response = this.buildResponse<T>(data, {
        protocol_used: protocol,
        version_used: version,
        format_used: this.config.defaultFormat,
        latency_ms: Date.now() - startTime,
        from_cache: false,
      });

      // 8. 캐시 저장
      if (this.config.cacheEnabled && resolved.action.startsWith('get')) {
        const cacheKey = this.cacheManager.generateKey(request);
        this.cacheManager.set(cacheKey, data);
      }

      // 9. 진화 엔진 학습
      if (this.config.evolutionEnabled) {
        this.evolutionEngine.learn(request, response, {
          latency: response.meta.latency_ms,
          success: true,
          cacheHit: false,
          timestamp: new Date(),
        });
      }

      return response;
    } catch (error) {
      return this.handleError(error, Date.now() - startTime);
    }
  }

  private determineRoute(resolved: ResolvedIntent, protocol: Protocol): Route {
    const { action, params } = resolved;
    let endpoint = this.config.baseUrl;
    let method: HttpMethod = 'GET';

    switch (action) {
      case 'get_by_id':
        endpoint += `/${params.resource}s/${params.id}`;
        method = 'GET';
        break;
      case 'list':
        endpoint += `/${params.resource}s`;
        method = 'GET';
        break;
      case 'create':
        endpoint += `/${params.resource}s`;
        method = 'POST';
        break;
      case 'update':
        endpoint += `/${params.resource}s/${params.id}`;
        method = 'PUT';
        break;
      case 'delete':
        endpoint += `/${params.resource}s/${params.id}`;
        method = 'DELETE';
        break;
      default:
        endpoint += '/api';
    }

    return { protocol, endpoint, method };
  }

  private async executeWithRetry(route: Route, payload?: any): Promise<any> {
    let lastError: Error | undefined;

    for (let attempt = 0; attempt <= this.config.maxRetries; attempt++) {
      try {
        return await this.protocolEmbracer.execute(route, payload);
      } catch (error) {
        lastError = error as Error;
        if (attempt < this.config.maxRetries) {
          // 지수 백오프
          await this.sleep(Math.pow(2, attempt) * 1000);
        }
      }
    }

    throw lastError;
  }

  private buildResponse<T>(
    data: T,
    meta: OmniResponse['meta']
  ): OmniResponse<T> {
    return {
      success: true,
      data,
      meta,
    };
  }

  private handleError(error: any, latency: number): OmniResponse {
    const omniError: OmniError = {
      code: error.code || 'UNKNOWN_ERROR',
      message: error.message || 'An unexpected error occurred',
      comfort: error.comfort || '걱정 마세요. 제가 도와드릴게요.',
      cause: error.cause,
      solution: error.solution,
      recovery_options: error.recoveryOptions || [
        {
          action: 'retry',
          description: '다시 시도해보세요',
          auto_applicable: true,
          confidence: 0.8,
        },
      ],
    };

    return {
      success: false,
      meta: {
        protocol_used: 'rest',
        version_used: '3.0',
        format_used: 'json',
        latency_ms: latency,
        from_cache: false,
      },
      error: omniError,
    };
  }

  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  // ============================================================
  // Public Utilities
  // ============================================================

  /** 캐시 통계 */
  getCacheStats() {
    return this.cacheManager.getStats();
  }

  /** 캐시 초기화 */
  clearCache() {
    this.cacheManager.clear();
  }

  /** 감지된 패턴 */
  getDetectedPatterns(): Pattern[] {
    return this.evolutionEngine.detectPatterns();
  }

  /** 최적화 제안 */
  getOptimizationSuggestions(): Optimization[] {
    return this.evolutionEngine.suggestOptimizations();
  }
}

// ============================================================
// Convenience Functions
// ============================================================

let defaultGateway: OmniGateway | null = null;

/**
 * 기본 게이트웨이로 요청
 */
export async function omniRequest<T = any>(
  request: OmniRequest
): Promise<OmniResponse<T>> {
  if (!defaultGateway) {
    defaultGateway = new OmniGateway();
  }
  return defaultGateway.request<T>(request);
}

/**
 * 간단한 의도 기반 요청
 */
export async function ask<T = any>(intent: string, payload?: any): Promise<T | undefined> {
  const response = await omniRequest<T>({ intent, payload });
  return response.data;
}

/**
 * 기본 게이트웨이 설정
 */
export function configure(config: Partial<GatewayConfig>): void {
  defaultGateway = new OmniGateway(config);
}

// ============================================================
// Version & Export
// ============================================================

export const VERSION = '1.0.0';
export const WIA_OMNI_API_VERSION = '1.0';

export default OmniGateway;
