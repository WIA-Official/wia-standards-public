/**
 * WIA-COMM-014: CDN SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Communication Research Group
 *
 * 弘익人間 (Benefit All Humanity)
 *
 * This SDK provides tools for CDN management including:
 * - Edge caching configuration
 * - Cache purge operations
 * - Performance analytics
 * - Multi-CDN orchestration
 * - DDoS protection
 */

import {
  CDNConfig,
  CacheRule,
  CachePurgeRequest,
  CachePurgeResponse,
  CachePerformance,
  EdgeLocation,
  EdgeTestResult,
  CDNAnalytics,
  MultiCDNConfig,
  CDNErrorCode,
  CDNError,
  CDN_CONSTANTS,
  Region,
  CDNProvider,
  HTTPProtocol,
} from './types';

// ============================================================================
// Main SDK Class
// ============================================================================

/**
 * WIA-COMM-014 CDN SDK
 */
export class CDNManager {
  private version = '1.0.0';
  private config: CDNConfig;

  constructor(config: CDNConfig) {
    this.config = config;
    this.validateConfig(config);
  }

  /**
   * Get SDK version
   */
  getVersion(): string {
    return this.version;
  }

  /**
   * Validate CDN configuration
   */
  private validateConfig(config: CDNConfig): void {
    if (!config.provider) {
      throw new CDNError(
        CDNErrorCode.INVALID_CONFIG,
        'CDN provider is required'
      );
    }

    if (!config.edgeLocations || config.edgeLocations.length === 0) {
      throw new CDNError(
        CDNErrorCode.INVALID_CONFIG,
        'At least one edge location is required'
      );
    }
  }

  /**
   * Create CDN configuration
   */
  static createConfig(params: Partial<CDNConfig>): CDNConfig {
    return {
      provider: params.provider || 'cloudflare',
      edgeLocations: params.edgeLocations || ['global'],
      cacheStrategy: params.cacheStrategy || {
        staticAssets: {
          ttl: CDN_CONSTANTS.DEFAULT_TTLS.STATIC_ASSETS,
          staleWhileRevalidate: 3600,
          immutable: true,
        },
        dynamicContent: {
          ttl: CDN_CONSTANTS.DEFAULT_TTLS.DYNAMIC_HTML,
          edgeCache: true,
          varyHeaders: ['Accept-Encoding', 'Accept-Language'],
        },
      },
      originShield: params.originShield,
      tls: params.tls,
      ddosProtection: params.ddosProtection,
      loadBalancing: params.loadBalancing,
      analytics: params.analytics,
    };
  }

  /**
   * Calculate cache hit ratio
   */
  calculateCacheHitRatio(params: {
    totalRequests: number;
    cacheHits: number;
    cacheMisses: number;
  }): CachePerformance {
    const { totalRequests, cacheHits, cacheMisses } = params;

    if (totalRequests === 0) {
      throw new CDNError(
        CDNErrorCode.INVALID_CONFIG,
        'Total requests must be greater than 0'
      );
    }

    const hitRatio = (cacheHits / totalRequests) * 100;

    // Estimate bandwidth savings (assume average response size 100 KB)
    const avgResponseSize = 100 * 1024; // 100 KB in bytes
    const bandwidthCached = cacheHits * avgResponseSize;
    const bandwidthOrigin = cacheMisses * avgResponseSize;
    const bandwidthSaved =
      (bandwidthCached / (bandwidthCached + bandwidthOrigin)) * 100;

    // Estimate origin load reduction
    const originRequests = cacheMisses;
    const originLoadReduction = (cacheHits / totalRequests) * 100;

    // Estimate TTFB improvement
    const avgTTFBCached = 50; // ms (edge)
    const avgTTFBOrigin = 500; // ms (origin)
    const avgTTFB =
      (cacheHits * avgTTFBCached + cacheMisses * avgTTFBOrigin) /
      totalRequests;

    return {
      totalRequests,
      cacheHits,
      cacheMisses,
      hitRatio: parseFloat(hitRatio.toFixed(2)),
      bandwidthCached,
      bandwidthOrigin,
      bandwidthSaved: parseFloat(bandwidthSaved.toFixed(2)),
      avgTTFB: parseFloat(avgTTFB.toFixed(2)),
      originRequests,
      originLoadReduction: parseFloat(originLoadReduction.toFixed(2)),
    };
  }

  /**
   * Validate cache strategy
   */
  validateCacheStrategy(rule: CacheRule): {
    isValid: boolean;
    errors: string[];
    warnings: string[];
  } {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Validate TTL
    if (rule.ttl < 0) {
      errors.push('TTL cannot be negative');
    }

    if (rule.ttl > 31536000) {
      // 1 year
      warnings.push('TTL exceeds 1 year, consider reducing');
    }

    // Validate pattern
    if (!rule.pattern || rule.pattern.trim() === '') {
      errors.push('Cache rule pattern is required');
    }

    // Validate stale-while-revalidate
    if (
      rule.staleWhileRevalidate &&
      rule.staleWhileRevalidate > rule.ttl * 10
    ) {
      warnings.push(
        'Stale-while-revalidate is significantly larger than TTL'
      );
    }

    // Validate cache key
    if (rule.cacheKey) {
      if (
        rule.cacheKey.includeHeaders &&
        rule.cacheKey.includeHeaders.length > 10
      ) {
        warnings.push(
          'Too many vary headers may reduce cache efficiency'
        );
      }
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings,
    };
  }

  /**
   * Purge cache
   */
  async purgeCache(request: CachePurgeRequest): Promise<CachePurgeResponse> {
    // Validate purge request
    if (request.type === 'url' && (!request.urls || request.urls.length === 0)) {
      throw new CDNError(
        CDNErrorCode.CACHE_PURGE_FAILED,
        'URLs are required for URL-based purge'
      );
    }

    if (request.type === 'tag' && (!request.tags || request.tags.length === 0)) {
      throw new CDNError(
        CDNErrorCode.CACHE_PURGE_FAILED,
        'Tags are required for tag-based purge'
      );
    }

    // Generate purge ID
    const purgeId = `purge-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Simulate purge operation (in real implementation, would call CDN API)
    const response: CachePurgeResponse = {
      id: purgeId,
      status: 'pending',
      progress: 0,
      createdAt: new Date(),
    };

    // Simulate async purge completion
    setTimeout(() => {
      response.status = 'completed';
      response.progress = 100;
      response.completedAt = new Date();
    }, 5000);

    return response;
  }

  /**
   * Get edge locations
   */
  getEdgeLocations(region?: Region): EdgeLocation[] {
    // Sample edge locations (in real implementation, would fetch from CDN API)
    const locations: EdgeLocation[] = [
      {
        code: 'LAX',
        city: 'Los Angeles',
        country: 'USA',
        region: 'NA',
        latitude: 34.0522,
        longitude: -118.2437,
        popCount: 5,
        capacity: 100000,
        status: 'active',
      },
      {
        code: 'LHR',
        city: 'London',
        country: 'UK',
        region: 'EU',
        latitude: 51.5074,
        longitude: -0.1278,
        popCount: 8,
        capacity: 120000,
        status: 'active',
      },
      {
        code: 'SIN',
        city: 'Singapore',
        country: 'Singapore',
        region: 'APAC',
        latitude: 1.3521,
        longitude: 103.8198,
        popCount: 6,
        capacity: 80000,
        status: 'active',
      },
      {
        code: 'GRU',
        city: 'São Paulo',
        country: 'Brazil',
        region: 'SA',
        latitude: -23.5505,
        longitude: -46.6333,
        popCount: 3,
        capacity: 50000,
        status: 'active',
      },
    ];

    if (region) {
      return locations.filter((loc) => loc.region === region);
    }

    return locations;
  }

  /**
   * Test edge location performance
   */
  async testEdgeLocation(
    url: string,
    location: EdgeLocation
  ): Promise<EdgeTestResult> {
    // Simulate edge test (in real implementation, would make actual HTTP request)
    const latency = Math.random() * 100 + 10; // 10-110 ms
    const ttfb = Math.random() * 50 + 20; // 20-70 ms
    const responseTime = ttfb + Math.random() * 100; // Total response time

    const protocols: HTTPProtocol[] = ['http/1.1', 'http/2', 'http/3'];
    const cacheStatuses: ('hit' | 'miss' | 'expired' | 'bypass')[] = [
      'hit',
      'miss',
      'expired',
      'bypass',
    ];

    return {
      location,
      latency: parseFloat(latency.toFixed(2)),
      ttfb: parseFloat(ttfb.toFixed(2)),
      cacheStatus: cacheStatuses[Math.floor(Math.random() * cacheStatuses.length)],
      protocol: protocols[Math.floor(Math.random() * protocols.length)],
      tlsVersion: 'TLS 1.3',
      serverId: `edge-${location.code}-${Math.floor(Math.random() * 100)}`,
      responseTime: parseFloat(responseTime.toFixed(2)),
    };
  }

  /**
   * Optimize origin shield
   */
  optimizeOriginShield(params: {
    originRegion: Region;
    trafficPatterns: { region: Region; requests: number }[];
  }): {
    primaryShield: Region;
    backupShield: Region;
    expectedCacheHitImprovement: number;
  } {
    const { originRegion, trafficPatterns } = params;

    // Sort traffic by requests
    const sortedTraffic = [...trafficPatterns].sort(
      (a, b) => b.requests - a.requests
    );

    // Primary shield should be same as origin region for lowest latency
    const primaryShield = originRegion;

    // Backup shield should be in highest traffic region (if different from origin)
    const backupShield =
      sortedTraffic[0].region !== originRegion
        ? sortedTraffic[0].region
        : sortedTraffic[1]?.region || originRegion;

    // Estimate cache hit improvement (typically 10-20% with shield)
    const expectedCacheHitImprovement = 15;

    return {
      primaryShield,
      backupShield,
      expectedCacheHitImprovement,
    };
  }

  /**
   * Analyze CDN performance
   */
  analyzeCDNPerformance(params: {
    timeframe: '1h' | '24h' | '7d' | '30d';
    metrics: string[];
  }): CDNAnalytics {
    const now = new Date();
    const start = new Date(now);

    // Calculate start time based on timeframe
    switch (params.timeframe) {
      case '1h':
        start.setHours(start.getHours() - 1);
        break;
      case '24h':
        start.setHours(start.getHours() - 24);
        break;
      case '7d':
        start.setDate(start.getDate() - 7);
        break;
      case '30d':
        start.setDate(start.getDate() - 30);
        break;
    }

    // Generate sample analytics (in real implementation, would fetch from CDN API)
    const totalRequests = Math.floor(Math.random() * 1000000) + 100000;
    const cacheHitRatio = Math.random() * 20 + 80; // 80-100%
    const cachedRequests = Math.floor(totalRequests * (cacheHitRatio / 100));
    const uncachedRequests = totalRequests - cachedRequests;

    return {
      period: {
        start,
        end: now,
      },
      requests: {
        total: totalRequests,
        cached: cachedRequests,
        uncached: uncachedRequests,
        errors: Math.floor(totalRequests * 0.001), // 0.1% error rate
      },
      bandwidth: {
        total: totalRequests * 100 * 1024, // Assume 100 KB average
        cached: cachedRequests * 100 * 1024,
        uncached: uncachedRequests * 100 * 1024,
        saved: cachedRequests * 100 * 1024,
      },
      performance: {
        avgTTFB: Math.random() * 30 + 20, // 20-50 ms
        p50TTFB: Math.random() * 20 + 15, // 15-35 ms
        p95TTFB: Math.random() * 50 + 50, // 50-100 ms
        p99TTFB: Math.random() * 100 + 100, // 100-200 ms
        cacheHitRatio: parseFloat(cacheHitRatio.toFixed(2)),
        errorRate: 0.1,
      },
      geoDistribution: [
        { region: 'NA', requests: Math.floor(totalRequests * 0.4), bandwidth: 0 },
        { region: 'EU', requests: Math.floor(totalRequests * 0.3), bandwidth: 0 },
        { region: 'APAC', requests: Math.floor(totalRequests * 0.25), bandwidth: 0 },
        { region: 'SA', requests: Math.floor(totalRequests * 0.05), bandwidth: 0 },
      ],
      protocolDistribution: [
        { protocol: 'http/2', percentage: 60 },
        { protocol: 'http/3', percentage: 30 },
        { protocol: 'http/1.1', percentage: 10 },
      ],
      deviceDistribution: [
        { type: 'mobile', percentage: 55 },
        { type: 'desktop', percentage: 40 },
        { type: 'tablet', percentage: 4 },
        { type: 'bot', percentage: 1 },
      ],
      securityEvents: {
        ddosBlocks: Math.floor(Math.random() * 100),
        wafBlocks: Math.floor(Math.random() * 1000),
        rateLimits: Math.floor(Math.random() * 500),
        botBlocks: Math.floor(Math.random() * 2000),
      },
    };
  }

  /**
   * Configure multi-CDN
   */
  configureMultiCDN(config: MultiCDNConfig): {
    isValid: boolean;
    warnings: string[];
    recommendations: string[];
  } {
    const warnings: string[] = [];
    const recommendations: string[] = [];

    // Validate traffic distribution sums to 100%
    const totalPercentage = config.distribution.reduce(
      (sum, dist) => sum + dist.percentage,
      0
    );

    if (Math.abs(totalPercentage - 100) > 0.01) {
      warnings.push(
        `Traffic distribution sums to ${totalPercentage}%, should be 100%`
      );
    }

    // Check if primary CDN has highest percentage
    const primaryDist = config.distribution.find(
      (d) => d.provider === config.primary
    );
    const maxPercentage = Math.max(
      ...config.distribution.map((d) => d.percentage)
    );

    if (primaryDist && primaryDist.percentage < maxPercentage) {
      recommendations.push('Consider making the CDN with highest traffic primary');
    }

    // Recommend at least 2 CDNs for redundancy
    if (config.secondary.length === 0) {
      recommendations.push('Add at least one secondary CDN for redundancy');
    }

    // Check failover configuration
    if (!config.failoverEnabled) {
      recommendations.push('Enable failover for better reliability');
    }

    return {
      isValid: warnings.length === 0,
      warnings,
      recommendations,
    };
  }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/**
 * Create CDN configuration (standalone)
 */
export function createCDNConfig(params: Partial<CDNConfig>): CDNConfig {
  return CDNManager.createConfig(params);
}

/**
 * Calculate cache hit ratio (standalone)
 */
export function calculateCacheHitRatio(params: {
  totalRequests: number;
  cacheHits: number;
  cacheMisses: number;
}): CachePerformance {
  const manager = new CDNManager(createCDNConfig({ provider: 'cloudflare' }));
  return manager.calculateCacheHitRatio(params);
}

/**
 * Validate cache strategy (standalone)
 */
export function validateCacheStrategy(rule: CacheRule): {
  isValid: boolean;
  errors: string[];
  warnings: string[];
} {
  const manager = new CDNManager(createCDNConfig({ provider: 'cloudflare' }));
  return manager.validateCacheStrategy(rule);
}

/**
 * Optimize origin shield (standalone)
 */
export function optimizeOriginShield(params: {
  originRegion: Region;
  trafficPatterns: { region: Region; requests: number }[];
}): {
  primaryShield: Region;
  backupShield: Region;
  expectedCacheHitImprovement: number;
} {
  const manager = new CDNManager(createCDNConfig({ provider: 'cloudflare' }));
  return manager.optimizeOriginShield(params);
}

/**
 * Analyze CDN performance (standalone)
 */
export function analyzeCDNPerformance(params: {
  timeframe: '1h' | '24h' | '7d' | '30d';
  metrics: string[];
}): CDNAnalytics {
  const manager = new CDNManager(createCDNConfig({ provider: 'cloudflare' }));
  return manager.analyzeCDNPerformance(params);
}

// ============================================================================
// Export All
// ============================================================================

export * from './types';
export { CDNManager };
