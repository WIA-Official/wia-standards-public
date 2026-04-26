/**
 * WIA-COMM-014: CDN - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Types
// ============================================================================

/**
 * CDN providers
 */
export type CDNProvider =
  | 'cloudflare'
  | 'akamai'
  | 'fastly'
  | 'cloudfront'
  | 'azure'
  | 'google'
  | 'bunny'
  | 'custom';

/**
 * HTTP protocols
 */
export type HTTPProtocol = 'http/1.1' | 'http/2' | 'http/3' | 'quic';

/**
 * Cache actions
 */
export type CacheAction = 'cache' | 'bypass' | 'ignore' | 'revalidate';

/**
 * Geographic regions
 */
export type Region =
  | 'NA'
  | 'SA'
  | 'EU'
  | 'APAC'
  | 'ME'
  | 'AF'
  | 'global';

// ============================================================================
// CDN Configuration
// ============================================================================

/**
 * CDN configuration
 */
export interface CDNConfig {
  /** CDN provider */
  provider: CDNProvider;

  /** Edge locations to enable */
  edgeLocations: Region[];

  /** Cache strategy */
  cacheStrategy: CacheStrategy;

  /** Origin shield configuration */
  originShield?: OriginShieldConfig;

  /** TLS/SSL configuration */
  tls?: TLSConfig;

  /** DDoS protection */
  ddosProtection?: DDoSConfig;

  /** Load balancing */
  loadBalancing?: LoadBalancingConfig;

  /** Analytics */
  analytics?: AnalyticsConfig;
}

/**
 * Cache strategy configuration
 */
export interface CacheStrategy {
  /** Static assets caching */
  staticAssets?: {
    ttl: number;
    staleWhileRevalidate?: number;
    immutable?: boolean;
  };

  /** Dynamic content caching */
  dynamicContent?: {
    ttl: number;
    edgeCache: boolean;
    varyHeaders?: string[];
  };

  /** Video streaming caching */
  videoStreaming?: {
    segmentDuration: number;
    adaptiveBitrate: boolean;
    ttl?: number;
  };

  /** API response caching */
  apiResponses?: {
    ttl: number;
    varyByQuery?: boolean;
    varyByCookie?: boolean;
  };
}

/**
 * Origin shield configuration
 */
export interface OriginShieldConfig {
  /** Enable origin shield */
  enabled: boolean;

  /** Shield location (region) */
  location: Region;

  /** Cache TTL at shield */
  ttl: number;

  /** Backup shield location */
  backupLocation?: Region;

  /** Grace period for stale content */
  gracePeriod?: number;
}

/**
 * TLS/SSL configuration
 */
export interface TLSConfig {
  /** Minimum TLS version */
  minVersion: '1.2' | '1.3';

  /** Certificate type */
  certificateType: 'shared' | 'dedicated' | 'custom';

  /** Custom certificate (if applicable) */
  customCertificate?: {
    certificate: string;
    privateKey: string;
    bundle?: string;
  };

  /** HSTS configuration */
  hsts?: {
    enabled: boolean;
    maxAge: number;
    includeSubDomains: boolean;
    preload: boolean;
  };

  /** Cipher suites */
  cipherSuites?: string[];
}

/**
 * DDoS protection configuration
 */
export interface DDoSConfig {
  /** Enable DDoS protection */
  enabled: boolean;

  /** Challenge threshold (requests/min) */
  challengeThreshold: number;

  /** Block threshold (requests/min) */
  blockThreshold: number;

  /** WAF rules */
  wafRules?: WAFRule[];

  /** Rate limiting */
  rateLimiting?: RateLimitConfig[];

  /** Geo-blocking */
  geoBlocking?: {
    blockedCountries?: string[];
    allowedCountries?: string[];
  };
}

/**
 * WAF rule
 */
export interface WAFRule {
  /** Rule ID */
  id: string;

  /** Rule name */
  name: string;

  /** Rule type */
  type: 'sql-injection' | 'xss' | 'csrf' | 'file-inclusion' | 'custom';

  /** Action to take */
  action: 'block' | 'challenge' | 'log';

  /** Pattern to match */
  pattern?: string;

  /** Enabled */
  enabled: boolean;
}

/**
 * Rate limiting configuration
 */
export interface RateLimitConfig {
  /** Rule name */
  name: string;

  /** URL path pattern */
  path?: string;

  /** Request threshold */
  threshold: number;

  /** Time period (seconds) */
  period: number;

  /** Action to take */
  action: 'block' | 'challenge' | 'rate_limit';

  /** Scope */
  scope: 'global' | 'per-ip' | 'per-session';
}

/**
 * Load balancing configuration
 */
export interface LoadBalancingConfig {
  /** Algorithm */
  algorithm:
    | 'geographic'
    | 'round-robin'
    | 'least-connections'
    | 'weighted'
    | 'latency-based';

  /** Origin servers */
  origins: OriginServer[];

  /** Health check */
  healthCheck?: HealthCheckConfig;

  /** Failover */
  failover?: FailoverConfig;
}

/**
 * Origin server
 */
export interface OriginServer {
  /** Server ID */
  id: string;

  /** Hostname or IP */
  host: string;

  /** Port */
  port: number;

  /** Protocol */
  protocol: 'http' | 'https';

  /** Weight (for weighted load balancing) */
  weight?: number;

  /** Backup server */
  isBackup?: boolean;

  /** Health status */
  healthy?: boolean;
}

/**
 * Health check configuration
 */
export interface HealthCheckConfig {
  /** Enable health checks */
  enabled: boolean;

  /** Check interval (seconds) */
  interval: number;

  /** Timeout (seconds) */
  timeout: number;

  /** Unhealthy threshold */
  unhealthyThreshold: number;

  /** Healthy threshold */
  healthyThreshold: number;

  /** Check path */
  path: string;

  /** Expected status code */
  expectedStatus: number;

  /** Expected body (optional) */
  expectedBody?: string;
}

/**
 * Failover configuration
 */
export interface FailoverConfig {
  /** Strategy */
  strategy: 'active-passive' | 'active-active';

  /** Failover time (seconds) */
  failoverTime: number;

  /** Auto-failback */
  autoFailback: boolean;
}

/**
 * Analytics configuration
 */
export interface AnalyticsConfig {
  /** Enable analytics */
  enabled: boolean;

  /** Metrics to track */
  metrics: AnalyticsMetric[];

  /** Sampling rate (0-100%) */
  samplingRate?: number;

  /** Log retention (days) */
  retentionDays?: number;
}

/**
 * Analytics metrics
 */
export type AnalyticsMetric =
  | 'requests'
  | 'bandwidth'
  | 'cache-hit-ratio'
  | 'ttfb'
  | 'error-rate'
  | 'geo-distribution'
  | 'device-type'
  | 'protocol-version';

// ============================================================================
// Cache Operations
// ============================================================================

/**
 * Cache rule
 */
export interface CacheRule {
  /** Rule ID */
  id: string;

  /** URL pattern */
  pattern: string;

  /** TTL (seconds) */
  ttl: number;

  /** Cache action */
  action: CacheAction;

  /** Edge cache enabled */
  edgeCache?: boolean;

  /** Vary headers */
  varyHeaders?: string[];

  /** Cache key components */
  cacheKey?: CacheKeyConfig;

  /** Stale-while-revalidate (seconds) */
  staleWhileRevalidate?: number;

  /** Stale-if-error (seconds) */
  staleIfError?: number;
}

/**
 * Cache key configuration
 */
export interface CacheKeyConfig {
  /** Include query string */
  includeQuery?: boolean;

  /** Include specific headers */
  includeHeaders?: string[];

  /** Include cookies */
  includeCookies?: boolean;

  /** Include device type */
  includeDeviceType?: boolean;

  /** Include geo location */
  includeGeo?: boolean;

  /** Custom key components */
  custom?: string[];
}

/**
 * Cache purge request
 */
export interface CachePurgeRequest {
  /** Purge type */
  type: 'url' | 'tag' | 'host' | 'full';

  /** URLs to purge (if type is 'url') */
  urls?: string[];

  /** Tags to purge (if type is 'tag') */
  tags?: string[];

  /** Hosts to purge (if type is 'host') */
  hosts?: string[];

  /** Purge method */
  method?: 'hard' | 'soft';
}

/**
 * Cache purge response
 */
export interface CachePurgeResponse {
  /** Purge ID */
  id: string;

  /** Status */
  status: 'pending' | 'in-progress' | 'completed' | 'failed';

  /** Progress (0-100%) */
  progress: number;

  /** Created timestamp */
  createdAt: Date;

  /** Completed timestamp */
  completedAt?: Date;

  /** Error message (if failed) */
  error?: string;
}

/**
 * Cache performance metrics
 */
export interface CachePerformance {
  /** Total requests */
  totalRequests: number;

  /** Cache hits */
  cacheHits: number;

  /** Cache misses */
  cacheMisses: number;

  /** Cache hit ratio (%) */
  hitRatio: number;

  /** Bandwidth served from cache */
  bandwidthCached: number;

  /** Bandwidth served from origin */
  bandwidthOrigin: number;

  /** Bandwidth saved (%) */
  bandwidthSaved: number;

  /** Average TTFB (ms) */
  avgTTFB: number;

  /** Origin requests */
  originRequests: number;

  /** Origin load reduction (%) */
  originLoadReduction: number;
}

// ============================================================================
// Edge Operations
// ============================================================================

/**
 * Edge location
 */
export interface EdgeLocation {
  /** Location code */
  code: string;

  /** City */
  city: string;

  /** Country */
  country: string;

  /** Region */
  region: Region;

  /** Latitude */
  latitude: number;

  /** Longitude */
  longitude: number;

  /** PoP count */
  popCount: number;

  /** Capacity (Gbps) */
  capacity: number;

  /** Status */
  status: 'active' | 'maintenance' | 'offline';
}

/**
 * Edge test result
 */
export interface EdgeTestResult {
  /** Location */
  location: EdgeLocation;

  /** Latency (ms) */
  latency: number;

  /** TTFB (ms) */
  ttfb: number;

  /** Cache status */
  cacheStatus: 'hit' | 'miss' | 'expired' | 'bypass';

  /** Protocol used */
  protocol: HTTPProtocol;

  /** TLS version */
  tlsVersion?: string;

  /** Server ID */
  serverId?: string;

  /** Response time (ms) */
  responseTime: number;
}

// ============================================================================
// Video Streaming
// ============================================================================

/**
 * Video streaming configuration
 */
export interface VideoStreamingConfig {
  /** Streaming protocol */
  protocol: 'hls' | 'dash' | 'hds' | 'mss';

  /** Segment duration (seconds) */
  segmentDuration: number;

  /** Adaptive bitrate enabled */
  adaptiveBitrate: boolean;

  /** Bitrate ladder */
  bitrateLadder?: BitrateLevel[];

  /** Low-latency mode */
  lowLatency?: boolean;

  /** DVR window (seconds) */
  dvrWindow?: number;

  /** Encryption */
  encryption?: VideoEncryptionConfig;
}

/**
 * Bitrate level
 */
export interface BitrateLevel {
  /** Resolution */
  resolution: string;

  /** Bitrate (bps) */
  bitrate: number;

  /** Codec */
  codec: string;

  /** Frame rate */
  frameRate?: number;
}

/**
 * Video encryption configuration
 */
export interface VideoEncryptionConfig {
  /** Encryption method */
  method: 'aes-128' | 'aes-256' | 'widevine' | 'fairplay' | 'playready';

  /** Key rotation interval (seconds) */
  keyRotationInterval?: number;

  /** DRM provider */
  drmProvider?: string;
}

// ============================================================================
// Multi-CDN
// ============================================================================

/**
 * Multi-CDN configuration
 */
export interface MultiCDNConfig {
  /** Primary CDN */
  primary: CDNProvider;

  /** Secondary CDNs */
  secondary: CDNProvider[];

  /** Traffic distribution */
  distribution: TrafficDistribution[];

  /** Failover enabled */
  failoverEnabled: boolean;

  /** Performance monitoring */
  performanceMonitoring: boolean;
}

/**
 * Traffic distribution
 */
export interface TrafficDistribution {
  /** CDN provider */
  provider: CDNProvider;

  /** Traffic percentage (0-100) */
  percentage: number;

  /** Regions (if region-specific) */
  regions?: Region[];

  /** Priority */
  priority: number;
}

// ============================================================================
// Analytics
// ============================================================================

/**
 * CDN analytics
 */
export interface CDNAnalytics {
  /** Time period */
  period: {
    start: Date;
    end: Date;
  };

  /** Request statistics */
  requests: {
    total: number;
    cached: number;
    uncached: number;
    errors: number;
  };

  /** Bandwidth statistics */
  bandwidth: {
    total: number;
    cached: number;
    uncached: number;
    saved: number;
  };

  /** Performance metrics */
  performance: {
    avgTTFB: number;
    p50TTFB: number;
    p95TTFB: number;
    p99TTFB: number;
    cacheHitRatio: number;
    errorRate: number;
  };

  /** Geographic distribution */
  geoDistribution?: {
    region: Region;
    requests: number;
    bandwidth: number;
  }[];

  /** Protocol distribution */
  protocolDistribution?: {
    protocol: HTTPProtocol;
    percentage: number;
  }[];

  /** Device distribution */
  deviceDistribution?: {
    type: 'mobile' | 'desktop' | 'tablet' | 'bot';
    percentage: number;
  }[];

  /** Top URLs */
  topUrls?: {
    url: string;
    requests: number;
    bandwidth: number;
  }[];

  /** Security events */
  securityEvents?: {
    ddosBlocks: number;
    wafBlocks: number;
    rateLimits: number;
    botBlocks: number;
  };
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * CDN error codes
 */
export enum CDNErrorCode {
  INVALID_CONFIG = 'CDN001',
  CACHE_PURGE_FAILED = 'CDN002',
  ORIGIN_UNREACHABLE = 'CDN003',
  SSL_ERROR = 'CDN004',
  RATE_LIMIT_EXCEEDED = 'CDN005',
  AUTHENTICATION_FAILED = 'CDN006',
  INVALID_CACHE_KEY = 'CDN007',
  EDGE_LOCATION_UNAVAILABLE = 'CDN008',
  BANDWIDTH_LIMIT_EXCEEDED = 'CDN009',
  API_ERROR = 'CDN010',
}

/**
 * CDN error
 */
export class CDNError extends Error {
  constructor(
    public code: CDNErrorCode,
    message: string,
    public details?: Record<string, unknown>
  ) {
    super(message);
    this.name = 'CDNError';
  }
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * Result type for operations that can fail
 */
export type Result<T, E = Error> =
  | { success: true; data: T }
  | { success: false; error: E };

/**
 * Async result type
 */
export type AsyncResult<T, E = Error> = Promise<Result<T, E>>;

// ============================================================================
// Constants
// ============================================================================

/**
 * CDN constants
 */
export const CDN_CONSTANTS = {
  /** Default cache TTLs (seconds) */
  DEFAULT_TTLS: {
    STATIC_ASSETS: 86400, // 1 day
    DYNAMIC_HTML: 60, // 1 minute
    API_RESPONSE: 300, // 5 minutes
    VIDEO_SEGMENT: 604800, // 1 week
  },

  /** HTTP status codes */
  HTTP_STATUS: {
    OK: 200,
    NOT_MODIFIED: 304,
    BAD_REQUEST: 400,
    UNAUTHORIZED: 401,
    FORBIDDEN: 403,
    NOT_FOUND: 404,
    TOO_MANY_REQUESTS: 429,
    INTERNAL_ERROR: 500,
    BAD_GATEWAY: 502,
    SERVICE_UNAVAILABLE: 503,
    GATEWAY_TIMEOUT: 504,
  },

  /** Cache headers */
  CACHE_HEADERS: {
    CACHE_CONTROL: 'Cache-Control',
    EXPIRES: 'Expires',
    ETAG: 'ETag',
    LAST_MODIFIED: 'Last-Modified',
    VARY: 'Vary',
    CDN_CACHE_STATUS: 'X-Cache',
    CDN_CACHE_CONTROL: 'CDN-Cache-Control',
  },

  /** Security headers */
  SECURITY_HEADERS: {
    HSTS: 'Strict-Transport-Security',
    CSP: 'Content-Security-Policy',
    X_FRAME_OPTIONS: 'X-Frame-Options',
    X_CONTENT_TYPE_OPTIONS: 'X-Content-Type-Options',
    REFERRER_POLICY: 'Referrer-Policy',
  },
} as const;

// ============================================================================
// Export All
// ============================================================================

export type {
  CDNProvider,
  HTTPProtocol,
  CacheAction,
  Region,
  CDNConfig,
  CacheStrategy,
  OriginShieldConfig,
  TLSConfig,
  DDoSConfig,
  WAFRule,
  RateLimitConfig,
  LoadBalancingConfig,
  OriginServer,
  HealthCheckConfig,
  FailoverConfig,
  AnalyticsConfig,
  AnalyticsMetric,
  CacheRule,
  CacheKeyConfig,
  CachePurgeRequest,
  CachePurgeResponse,
  CachePerformance,
  EdgeLocation,
  EdgeTestResult,
  VideoStreamingConfig,
  BitrateLevel,
  VideoEncryptionConfig,
  MultiCDNConfig,
  TrafficDistribution,
  CDNAnalytics,
};

export { CDN_CONSTANTS, CDNErrorCode, CDNError };
