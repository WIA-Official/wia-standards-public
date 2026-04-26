/**
 * WIA-SOCIAL SDK
 * 조카의 TypeScript 구현 - "나 통해서 다 연결돼!" 🌐
 *
 * @packageDocumentation
 */

import {
  Platform,
  PlatformCapabilities,
  PlatformConnection,
  UniversalIdentity,
  UserProfile,
  PrivacySettings,
  PrivacyLevel,
  MediaRef,
  UniversalPost,
  PostContent,
  PostTarget,
  PlatformPost,
  PostMetadata,
  UnifiedReactions,
  UnifiedComments,
  UnifiedFeed,
  FeedSource,
  FeedFilters,
  FeedSorting,
  Connection,
  Relationship,
  FriendSuggestion,
  Conversation,
  Message,
  MessageContent,
  ExportPackage,
  ImportResult,
  PrivacyScanResult,
  PrivacyRisk,
  WIASocialConfig,
  SocialEvent,
  SocialEventType,
  EventCallback,
  PostCallback,
  Unsubscribe,
  PaginationOptions,
  FeedOptions,
  SearchOptions,
  CrossPostResult,
  SyncResult,
  WIAId,
  Timestamp,
  PostId,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Default Configuration
// ============================================================================

const DEFAULT_CONFIG: WIASocialConfig = {
  platforms: [],
  privacy: {
    defaultVisibility: 'public',
    dataCollection: {
      allowAnalytics: false,
      allowTargetedAds: false,
      allowThirdPartySharing: false,
      allowLocationTracking: false,
    },
    airShield: {
      enabled: true,
      protectionMode: 'balanced',
      autoScan: true,
    },
  },
  autoSync: true,
  syncInterval: 5,
  enableAirShield: true,
  enableAnalytics: false,
};

// ============================================================================
// Platform Adapter (Base)
// ============================================================================

abstract class PlatformAdapter {
  abstract platform: Platform;
  abstract capabilities: PlatformCapabilities;

  protected connected: boolean = false;
  protected token?: string;

  abstract connect(token: string): Promise<void>;
  abstract disconnect(): Promise<void>;

  abstract createPost(content: PostContent): Promise<PlatformPost>;
  abstract deletePost(postId: string): Promise<void>;
  abstract getFeed(options: FeedOptions): Promise<UniversalPost[]>;

  abstract getFollowers(options?: PaginationOptions): Promise<Connection[]>;
  abstract getFollowing(options?: PaginationOptions): Promise<Connection[]>;
  abstract follow(userId: string): Promise<void>;
  abstract unfollow(userId: string): Promise<void>;

  abstract exportData(): Promise<ExportPackage>;

  isConnected(): boolean {
    return this.connected;
  }
}

// ============================================================================
// Mock Platform Adapters (For demonstration)
// ============================================================================

class InstagramAdapter extends PlatformAdapter {
  platform: Platform = 'instagram';
  capabilities: PlatformCapabilities = {
    posting: true,
    stories: true,
    reels: true,
    liveStreaming: true,
    messaging: true,
    groups: false,
    events: false,
    polls: true,
    scheduling: false,
    analytics: true,
    export: true,
    import: false,
  };

  async connect(token: string): Promise<void> {
    this.token = token;
    this.connected = true;
    console.log('📸 Instagram 연결됨');
  }

  async disconnect(): Promise<void> {
    this.token = undefined;
    this.connected = false;
    console.log('📸 Instagram 연결 해제');
  }

  async createPost(content: PostContent): Promise<PlatformPost> {
    return {
      platform: 'instagram',
      platformPostId: `ig_${Date.now()}`,
      url: `https://instagram.com/p/${Date.now()}`,
      postedAt: Date.now(),
      status: 'posted',
    };
  }

  async deletePost(postId: string): Promise<void> {
    console.log(`📸 Instagram 포스트 삭제: ${postId}`);
  }

  async getFeed(options: FeedOptions): Promise<UniversalPost[]> {
    return [];
  }

  async getFollowers(): Promise<Connection[]> {
    return [];
  }

  async getFollowing(): Promise<Connection[]> {
    return [];
  }

  async follow(userId: string): Promise<void> {
    console.log(`📸 Instagram 팔로우: ${userId}`);
  }

  async unfollow(userId: string): Promise<void> {
    console.log(`📸 Instagram 언팔로우: ${userId}`);
  }

  async exportData(): Promise<ExportPackage> {
    return {
      format: 'wia-social-v1',
      exportedAt: Date.now(),
      identity: {} as UniversalIdentity,
      posts: [],
      connections: [],
      messages: [],
      media: { totalSize: 0, items: [] },
      metadata: {
        platforms: ['instagram'],
        dateRange: { start: 0, end: Date.now() },
        postCount: 0,
        connectionCount: 0,
        messageCount: 0,
        mediaCount: 0,
      },
    };
  }
}

class TwitterAdapter extends PlatformAdapter {
  platform: Platform = 'twitter';
  capabilities: PlatformCapabilities = {
    posting: true,
    stories: false,
    reels: false,
    liveStreaming: false,
    messaging: true,
    groups: false,
    events: false,
    polls: true,
    scheduling: true,
    analytics: true,
    export: true,
    import: false,
  };

  async connect(token: string): Promise<void> {
    this.token = token;
    this.connected = true;
    console.log('🐦 Twitter 연결됨');
  }

  async disconnect(): Promise<void> {
    this.token = undefined;
    this.connected = false;
    console.log('🐦 Twitter 연결 해제');
  }

  async createPost(content: PostContent): Promise<PlatformPost> {
    // Twitter 280자 제한 처리
    let text = content.text || '';
    if (text.length > 280) {
      text = text.substring(0, 277) + '...';
    }

    return {
      platform: 'twitter',
      platformPostId: `tw_${Date.now()}`,
      url: `https://twitter.com/status/${Date.now()}`,
      postedAt: Date.now(),
      status: 'posted',
    };
  }

  async deletePost(postId: string): Promise<void> {
    console.log(`🐦 Twitter 포스트 삭제: ${postId}`);
  }

  async getFeed(options: FeedOptions): Promise<UniversalPost[]> {
    return [];
  }

  async getFollowers(): Promise<Connection[]> {
    return [];
  }

  async getFollowing(): Promise<Connection[]> {
    return [];
  }

  async follow(userId: string): Promise<void> {
    console.log(`🐦 Twitter 팔로우: ${userId}`);
  }

  async unfollow(userId: string): Promise<void> {
    console.log(`🐦 Twitter 언팔로우: ${userId}`);
  }

  async exportData(): Promise<ExportPackage> {
    return {
      format: 'wia-social-v1',
      exportedAt: Date.now(),
      identity: {} as UniversalIdentity,
      posts: [],
      connections: [],
      messages: [],
      media: { totalSize: 0, items: [] },
      metadata: {
        platforms: ['twitter'],
        dateRange: { start: 0, end: Date.now() },
        postCount: 0,
        connectionCount: 0,
        messageCount: 0,
        mediaCount: 0,
      },
    };
  }
}

class TikTokAdapter extends PlatformAdapter {
  platform: Platform = 'tiktok';
  capabilities: PlatformCapabilities = {
    posting: true,
    stories: false,
    reels: true,
    liveStreaming: true,
    messaging: true,
    groups: false,
    events: false,
    polls: false,
    scheduling: false,
    analytics: true,
    export: true,
    import: false,
  };

  async connect(token: string): Promise<void> {
    this.token = token;
    this.connected = true;
    console.log('🎵 TikTok 연결됨');
  }

  async disconnect(): Promise<void> {
    this.token = undefined;
    this.connected = false;
    console.log('🎵 TikTok 연결 해제');
  }

  async createPost(content: PostContent): Promise<PlatformPost> {
    return {
      platform: 'tiktok',
      platformPostId: `tt_${Date.now()}`,
      url: `https://tiktok.com/@user/video/${Date.now()}`,
      postedAt: Date.now(),
      status: 'posted',
    };
  }

  async deletePost(postId: string): Promise<void> {
    console.log(`🎵 TikTok 포스트 삭제: ${postId}`);
  }

  async getFeed(options: FeedOptions): Promise<UniversalPost[]> {
    return [];
  }

  async getFollowers(): Promise<Connection[]> {
    return [];
  }

  async getFollowing(): Promise<Connection[]> {
    return [];
  }

  async follow(userId: string): Promise<void> {
    console.log(`🎵 TikTok 팔로우: ${userId}`);
  }

  async unfollow(userId: string): Promise<void> {
    console.log(`🎵 TikTok 언팔로우: ${userId}`);
  }

  async exportData(): Promise<ExportPackage> {
    return {
      format: 'wia-social-v1',
      exportedAt: Date.now(),
      identity: {} as UniversalIdentity,
      posts: [],
      connections: [],
      messages: [],
      media: { totalSize: 0, items: [] },
      metadata: {
        platforms: ['tiktok'],
        dateRange: { start: 0, end: Date.now() },
        postCount: 0,
        connectionCount: 0,
        messageCount: 0,
        mediaCount: 0,
      },
    };
  }
}

// ============================================================================
// Privacy Guard (이모 연동)
// ============================================================================

class SocialPrivacyGuard {
  private enabled: boolean;
  private mode: 'balanced' | 'fortress' | 'paranoid';

  constructor(enabled: boolean = true, mode: 'balanced' | 'fortress' | 'paranoid' = 'balanced') {
    this.enabled = enabled;
    this.mode = mode;
  }

  async scanBeforePost(content: PostContent): Promise<PrivacyScanResult> {
    if (!this.enabled) {
      return { safe: true, risks: [], recommendation: 'proceed' };
    }

    const risks: PrivacyRisk[] = [];

    // 위치 정보 체크
    if (content.location) {
      risks.push({
        type: 'location_exposure',
        severity: this.mode === 'paranoid' ? 'high' : 'medium',
        description: '위치 정보가 포함되어 있습니다',
        recommendation: '정확한 위치 대신 대략적 지역명 사용을 권장합니다',
      });
    }

    // 민감 정보 패턴 체크
    if (content.text) {
      const sensitivePatterns = [
        { pattern: /\d{3}-\d{4}-\d{4}/, type: 'phone' },
        { pattern: /\d{6}-\d{7}/, type: 'id_number' },
        { pattern: /[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}/, type: 'email' },
      ];

      for (const { pattern, type } of sensitivePatterns) {
        if (pattern.test(content.text)) {
          risks.push({
            type: 'sensitive_info',
            severity: 'high',
            description: `${type} 정보가 감지되었습니다`,
            recommendation: '개인정보를 제거해주세요',
          });
        }
      }
    }

    // 미디어 메타데이터 체크
    if (content.media && content.media.length > 0) {
      for (const media of content.media) {
        if (media.metadata && !media.metadata.stripped) {
          if (media.metadata.location) {
            risks.push({
              type: 'metadata_leak',
              severity: 'medium',
              description: '미디어에 위치 메타데이터가 포함되어 있습니다',
              recommendation: '메타데이터를 제거하시겠습니까?',
            });
          }
        }
      }
    }

    const hasHighRisk = risks.some((r) => r.severity === 'high');
    const hasMediumRisk = risks.some((r) => r.severity === 'medium');

    return {
      safe: !hasHighRisk,
      risks,
      recommendation: hasHighRisk ? 'block' : hasMediumRisk ? 'review' : 'proceed',
    };
  }

  stripMetadata(media: MediaRef): MediaRef {
    return {
      ...media,
      metadata: {
        ...media.metadata,
        location: undefined,
        stripped: true,
      },
    };
  }

  fuzzyLocation(lat: number, lon: number, radiusKm: number = 1): { lat: number; lon: number } {
    const earthRadius = 6371; // km
    const randomAngle = Math.random() * 2 * Math.PI;
    const randomRadius = Math.random() * radiusKm;

    const deltaLat = (randomRadius / earthRadius) * (180 / Math.PI);
    const deltaLon =
      (randomRadius / (earthRadius * Math.cos((lat * Math.PI) / 180))) * (180 / Math.PI);

    return {
      lat: lat + deltaLat * Math.cos(randomAngle),
      lon: lon + deltaLon * Math.sin(randomAngle),
    };
  }
}

// ============================================================================
// Feed Manager
// ============================================================================

class FeedManager {
  private adapters: Map<Platform, PlatformAdapter>;
  private cache: Map<string, UniversalPost[]> = new Map();
  private lastFetch: Map<Platform, number> = new Map();

  constructor(adapters: Map<Platform, PlatformAdapter>) {
    this.adapters = adapters;
  }

  async getUnifiedFeed(options: FeedOptions = {}): Promise<UnifiedFeed> {
    const posts: UniversalPost[] = [];
    const sources: FeedSource[] = [];
    const platforms = options.filters?.platforms || Array.from(this.adapters.keys());

    for (const platform of platforms) {
      const adapter = this.adapters.get(platform);
      if (!adapter || !adapter.isConnected()) {
        sources.push({
          platform,
          enabled: false,
          lastFetched: 0,
          error: 'Not connected',
        });
        continue;
      }

      try {
        const platformPosts = await adapter.getFeed(options);
        posts.push(...platformPosts);
        this.lastFetch.set(platform, Date.now());
        sources.push({
          platform,
          enabled: true,
          lastFetched: Date.now(),
        });
      } catch (error) {
        sources.push({
          platform,
          enabled: true,
          lastFetched: this.lastFetch.get(platform) || 0,
          error: String(error),
        });
      }
    }

    // Sort posts
    const sorting = options.sorting || 'reverse_chronological';
    if (sorting === 'chronological') {
      posts.sort((a, b) => a.metadata.createdAt - b.metadata.createdAt);
    } else if (sorting === 'reverse_chronological') {
      posts.sort((a, b) => b.metadata.createdAt - a.metadata.createdAt);
    }

    // Apply limit
    const limit = options.limit || 50;
    const limitedPosts = posts.slice(0, limit);

    return {
      posts: limitedPosts,
      sources,
      filters: options.filters || {},
      sorting,
      hasMore: posts.length > limit,
    };
  }

  subscribe(callback: PostCallback): Unsubscribe {
    // In real implementation, set up WebSocket connections
    const intervalId = setInterval(async () => {
      const feed = await this.getUnifiedFeed({ limit: 10 });
      feed.posts.forEach(callback);
    }, 30000);

    return () => clearInterval(intervalId);
  }
}

// ============================================================================
// Social Graph Manager
// ============================================================================

class SocialGraphManager {
  private adapters: Map<Platform, PlatformAdapter>;
  private connectionCache: Map<string, Connection[]> = new Map();

  constructor(adapters: Map<Platform, PlatformAdapter>) {
    this.adapters = adapters;
  }

  async getFollowers(options?: PaginationOptions): Promise<Connection[]> {
    const allFollowers: Connection[] = [];

    for (const [platform, adapter] of this.adapters) {
      if (adapter.isConnected()) {
        const followers = await adapter.getFollowers(options);
        allFollowers.push(...followers);
      }
    }

    // Deduplicate by WIA ID
    const unique = new Map<string, Connection>();
    for (const conn of allFollowers) {
      const key = conn.identity.wia_id;
      if (!unique.has(key)) {
        unique.set(key, conn);
      } else {
        // Merge platforms
        const existing = unique.get(key)!;
        existing.platforms = [...new Set([...existing.platforms, ...conn.platforms])];
      }
    }

    return Array.from(unique.values());
  }

  async getFollowing(options?: PaginationOptions): Promise<Connection[]> {
    const allFollowing: Connection[] = [];

    for (const [platform, adapter] of this.adapters) {
      if (adapter.isConnected()) {
        const following = await adapter.getFollowing(options);
        allFollowing.push(...following);
      }
    }

    // Deduplicate
    const unique = new Map<string, Connection>();
    for (const conn of allFollowing) {
      const key = conn.identity.wia_id;
      if (!unique.has(key)) {
        unique.set(key, conn);
      } else {
        const existing = unique.get(key)!;
        existing.platforms = [...new Set([...existing.platforms, ...conn.platforms])];
      }
    }

    return Array.from(unique.values());
  }

  async getMutuals(): Promise<Connection[]> {
    const followers = await this.getFollowers();
    const following = await this.getFollowing();

    const followingIds = new Set(following.map((c) => c.identity.wia_id));
    return followers.filter((f) => followingIds.has(f.identity.wia_id));
  }

  async follow(identity: UniversalIdentity): Promise<void> {
    for (const platform of identity.connectedPlatforms) {
      const adapter = this.adapters.get(platform.platform);
      if (adapter && adapter.isConnected()) {
        await adapter.follow(platform.userId);
      }
    }
    console.log(`🌐 ${identity.displayName}님을 모든 플랫폼에서 팔로우했습니다`);
  }

  async unfollow(identity: UniversalIdentity): Promise<void> {
    for (const platform of identity.connectedPlatforms) {
      const adapter = this.adapters.get(platform.platform);
      if (adapter && adapter.isConnected()) {
        await adapter.unfollow(platform.userId);
      }
    }
    console.log(`🌐 ${identity.displayName}님을 모든 플랫폼에서 언팔로우했습니다`);
  }

  async findAcrossPlatforms(query: string): Promise<UniversalIdentity[]> {
    // In real implementation, search across all platforms
    return [];
  }

  async getSuggestions(): Promise<FriendSuggestion[]> {
    // In real implementation, use ML to suggest friends
    return [];
  }
}

// ============================================================================
// Data Portability Manager
// ============================================================================

class DataPortabilityManager {
  private adapters: Map<Platform, PlatformAdapter>;

  constructor(adapters: Map<Platform, PlatformAdapter>) {
    this.adapters = adapters;
  }

  async exportAll(): Promise<ExportPackage> {
    const packages: ExportPackage[] = [];

    for (const [platform, adapter] of this.adapters) {
      if (adapter.isConnected()) {
        const pkg = await adapter.exportData();
        packages.push(pkg);
      }
    }

    // Merge all packages
    return this.mergePackages(packages);
  }

  async exportPlatform(platform: Platform): Promise<ExportPackage> {
    const adapter = this.adapters.get(platform);
    if (!adapter || !adapter.isConnected()) {
      throw new Error(`${platform} is not connected`);
    }
    return adapter.exportData();
  }

  private mergePackages(packages: ExportPackage[]): ExportPackage {
    const merged: ExportPackage = {
      format: 'wia-social-v1',
      exportedAt: Date.now(),
      identity: packages[0]?.identity || ({} as UniversalIdentity),
      posts: [],
      connections: [],
      messages: [],
      media: { totalSize: 0, items: [] },
      metadata: {
        platforms: [],
        dateRange: { start: Infinity, end: 0 },
        postCount: 0,
        connectionCount: 0,
        messageCount: 0,
        mediaCount: 0,
      },
    };

    for (const pkg of packages) {
      merged.posts.push(...pkg.posts);
      merged.connections.push(...pkg.connections);
      merged.messages.push(...pkg.messages);
      merged.media.items.push(...pkg.media.items);
      merged.media.totalSize += pkg.media.totalSize;
      merged.metadata.platforms.push(...pkg.metadata.platforms);
      merged.metadata.postCount += pkg.metadata.postCount;
      merged.metadata.connectionCount += pkg.metadata.connectionCount;
      merged.metadata.messageCount += pkg.metadata.messageCount;
      merged.metadata.mediaCount += pkg.metadata.mediaCount;

      if (pkg.metadata.dateRange.start < merged.metadata.dateRange.start) {
        merged.metadata.dateRange.start = pkg.metadata.dateRange.start;
      }
      if (pkg.metadata.dateRange.end > merged.metadata.dateRange.end) {
        merged.metadata.dateRange.end = pkg.metadata.dateRange.end;
      }
    }

    return merged;
  }

  async deleteFromPlatform(platform: Platform): Promise<void> {
    // In real implementation, use platform's data deletion API
    console.log(`🗑️ ${platform}에서 데이터 삭제 요청됨`);
  }

  async deleteAll(): Promise<void> {
    for (const platform of this.adapters.keys()) {
      await this.deleteFromPlatform(platform);
    }
    console.log('🗑️ 모든 플랫폼에서 데이터 삭제 요청됨');
  }
}

// ============================================================================
// Main WIASocial Class
// ============================================================================

/**
 * WIA-SOCIAL 메인 클래스
 * 조카의 연결 - "나 통해서 다 연결돼!" 🌐
 */
export class WIASocial {
  private config: WIASocialConfig;
  private identity?: UniversalIdentity;
  private adapters: Map<Platform, PlatformAdapter> = new Map();
  private connected: boolean = false;

  // Managers
  private feedManager!: FeedManager;
  private socialGraph!: SocialGraphManager;
  private portability!: DataPortabilityManager;
  private privacyGuard: SocialPrivacyGuard;

  // Event handlers
  private eventHandlers: Map<SocialEventType, Set<EventCallback>> = new Map();

  constructor(config: Partial<WIASocialConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.identity = config.identity;

    // Initialize privacy guard (이모 연동)
    this.privacyGuard = new SocialPrivacyGuard(
      this.config.enableAirShield,
      this.config.privacy.airShield?.protectionMode || 'balanced'
    );

    // Initialize adapters for requested platforms
    this.initializeAdapters();

    console.log('🌐 조카 준비 완료: "나 통해서 다 연결돼!"');
  }

  private initializeAdapters(): void {
    for (const platform of this.config.platforms) {
      switch (platform) {
        case 'instagram':
          this.adapters.set('instagram', new InstagramAdapter());
          break;
        case 'twitter':
          this.adapters.set('twitter', new TwitterAdapter());
          break;
        case 'tiktok':
          this.adapters.set('tiktok', new TikTokAdapter());
          break;
        // Add more adapters as needed
      }
    }

    // Initialize managers
    this.feedManager = new FeedManager(this.adapters);
    this.socialGraph = new SocialGraphManager(this.adapters);
    this.portability = new DataPortabilityManager(this.adapters);
  }

  // ============================================================================
  // Connection
  // ============================================================================

  async connect(): Promise<void> {
    console.log('🌐 모든 플랫폼 연결 중...');
    this.connected = true;
  }

  async connectPlatform(platform: Platform, token: string): Promise<void> {
    let adapter = this.adapters.get(platform);

    if (!adapter) {
      // Create adapter if not exists
      switch (platform) {
        case 'instagram':
          adapter = new InstagramAdapter();
          break;
        case 'twitter':
          adapter = new TwitterAdapter();
          break;
        case 'tiktok':
          adapter = new TikTokAdapter();
          break;
        default:
          throw new Error(`Unsupported platform: ${platform}`);
      }
      this.adapters.set(platform, adapter);
    }

    await adapter.connect(token);
    this.emit('platform_connected', { platform });
  }

  async disconnectPlatform(platform: Platform): Promise<void> {
    const adapter = this.adapters.get(platform);
    if (adapter) {
      await adapter.disconnect();
      this.emit('platform_disconnected', { platform });
    }
  }

  getConnectedPlatforms(): Platform[] {
    return Array.from(this.adapters.entries())
      .filter(([_, adapter]) => adapter.isConnected())
      .map(([platform]) => platform);
  }

  // ============================================================================
  // Identity
  // ============================================================================

  static async createIdentity(params: {
    displayName: string;
    email: string;
    username?: string;
  }): Promise<UniversalIdentity> {
    const wiaId: WIAId = `wia:${params.username || params.displayName.toLowerCase().replace(/\s/g, '.')}.${Date.now()}`;

    return {
      wia_id: wiaId,
      displayName: params.displayName,
      username: params.username || params.displayName.toLowerCase().replace(/\s/g, '_'),
      profile: {
        bio: '',
        links: [],
        badges: [],
      },
      connectedPlatforms: [],
      privacy: {
        defaultVisibility: 'public',
        platformOverrides: {},
        dataCollection: {
          allowAnalytics: false,
          allowTargetedAds: false,
          allowThirdPartySharing: false,
          allowLocationTracking: false,
        },
        airShield: {
          enabled: true,
          protectionMode: 'balanced',
          autoScan: true,
        },
      },
      verification: {
        level: 'basic',
        email: true,
        phone: false,
        identity: false,
        platformCount: 0,
      },
      createdAt: Date.now(),
      updatedAt: Date.now(),
    };
  }

  getIdentity(): UniversalIdentity | undefined {
    return this.identity;
  }

  async updateProfile(profile: Partial<UserProfile>): Promise<void> {
    if (this.identity) {
      this.identity.profile = { ...this.identity.profile, ...profile };
      this.identity.updatedAt = Date.now();
    }
  }

  // ============================================================================
  // Posting
  // ============================================================================

  async post(params: {
    content: PostContent;
    targets?: Platform[];
    privacy?: PrivacyLevel;
    schedule?: Timestamp;
  }): Promise<CrossPostResult> {
    const { content, targets, privacy, schedule } = params;
    const targetPlatforms = targets || this.getConnectedPlatforms();

    // Privacy scan (이모 연동)
    if (this.config.enableAirShield) {
      const scanResult = await this.privacyGuard.scanBeforePost(content);
      if (scanResult.recommendation === 'block') {
        console.log('🛡️ 이모: 위험한 정보가 있어요! 포스팅을 막았어요.');
        this.emit('privacy_alert', { risks: scanResult.risks });
        return {
          success: false,
          results: targetPlatforms.map((p) => ({
            platform: p,
            success: false,
            error: 'Privacy check failed',
          })),
        };
      } else if (scanResult.recommendation === 'review') {
        console.log('🛡️ 이모: 확인이 필요한 내용이 있어요:', scanResult.risks);
      }
    }

    const results: CrossPostResult['results'] = [];

    for (const platform of targetPlatforms) {
      const adapter = this.adapters.get(platform);
      if (!adapter || !adapter.isConnected()) {
        results.push({
          platform,
          success: false,
          error: 'Not connected',
        });
        continue;
      }

      try {
        const platformPost = await adapter.createPost(content);
        results.push({
          platform,
          success: true,
          postId: platformPost.platformPostId,
          url: platformPost.url,
        });
      } catch (error) {
        results.push({
          platform,
          success: false,
          error: String(error),
        });
      }
    }

    const success = results.some((r) => r.success);
    if (success) {
      console.log(`🌐 ${results.filter((r) => r.success).length}개 플랫폼에 포스팅 완료!`);
    }

    return { success, results };
  }

  async deletePost(postId: PostId, platforms?: Platform[]): Promise<void> {
    const targetPlatforms = platforms || this.getConnectedPlatforms();

    for (const platform of targetPlatforms) {
      const adapter = this.adapters.get(platform);
      if (adapter && adapter.isConnected()) {
        await adapter.deletePost(postId);
      }
    }

    console.log(`🌐 포스트 삭제 완료: ${postId}`);
  }

  // ============================================================================
  // Feed
  // ============================================================================

  async getFeed(options?: FeedOptions): Promise<UnifiedFeed> {
    return this.feedManager.getUnifiedFeed(options);
  }

  subscribeFeed(callback: PostCallback): Unsubscribe {
    return this.feedManager.subscribe(callback);
  }

  // ============================================================================
  // Social Graph
  // ============================================================================

  async getFollowers(options?: PaginationOptions): Promise<Connection[]> {
    return this.socialGraph.getFollowers(options);
  }

  async getFollowing(options?: PaginationOptions): Promise<Connection[]> {
    return this.socialGraph.getFollowing(options);
  }

  async getMutuals(): Promise<Connection[]> {
    return this.socialGraph.getMutuals();
  }

  async follow(identity: UniversalIdentity): Promise<void> {
    return this.socialGraph.follow(identity);
  }

  async unfollow(identity: UniversalIdentity): Promise<void> {
    return this.socialGraph.unfollow(identity);
  }

  async findFriendAcrossPlatforms(query: string): Promise<UniversalIdentity[]> {
    return this.socialGraph.findAcrossPlatforms(query);
  }

  async getSuggestions(): Promise<FriendSuggestion[]> {
    return this.socialGraph.getSuggestions();
  }

  // ============================================================================
  // Data Portability
  // ============================================================================

  async exportAll(): Promise<ExportPackage> {
    return this.portability.exportAll();
  }

  async exportPlatform(platform: Platform): Promise<ExportPackage> {
    return this.portability.exportPlatform(platform);
  }

  async deleteData(platform?: Platform): Promise<void> {
    if (platform) {
      await this.portability.deleteFromPlatform(platform);
    } else {
      await this.portability.deleteAll();
    }
  }

  // ============================================================================
  // Search
  // ============================================================================

  async search(query: string, options?: SearchOptions): Promise<UniversalIdentity[]> {
    return this.socialGraph.findAcrossPlatforms(query);
  }

  // ============================================================================
  // Privacy (이모 연동)
  // ============================================================================

  async scanContent(content: PostContent): Promise<PrivacyScanResult> {
    return this.privacyGuard.scanBeforePost(content);
  }

  stripMediaMetadata(media: MediaRef): MediaRef {
    return this.privacyGuard.stripMetadata(media);
  }

  // ============================================================================
  // Events
  // ============================================================================

  on(event: SocialEventType, callback: EventCallback): Unsubscribe {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(callback);
    return () => this.eventHandlers.get(event)?.delete(callback);
  }

  private emit(event: SocialEventType, data: unknown): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      const socialEvent: SocialEvent = {
        type: event,
        timestamp: Date.now(),
        data,
      };
      handlers.forEach((handler) => handler(socialEvent));
    }
  }

  // ============================================================================
  // Stats
  // ============================================================================

  getStats(): {
    connectedPlatforms: number;
    totalFollowers: number;
    totalFollowing: number;
  } {
    return {
      connectedPlatforms: this.getConnectedPlatforms().length,
      totalFollowers: 0, // Would be calculated from actual data
      totalFollowing: 0,
    };
  }
}

// ============================================================================
// Convenience Functions
// ============================================================================

let globalSocial: WIASocial | null = null;

/**
 * 전역 소셜 초기화
 */
export function initSocial(config?: Partial<WIASocialConfig>): WIASocial {
  globalSocial = new WIASocial(config);
  return globalSocial;
}

/**
 * 전역 소셜 가져오기
 */
export function getSocial(): WIASocial | null {
  return globalSocial;
}

// ============================================================================
// Family Integration Helper
// ============================================================================

/**
 * WIA 가족 통합 헬퍼
 */
export const familyIntegration = {
  /**
   * 의도 기반 포스팅 (아버지 연동)
   */
  postWithIntent: async (
    social: WIASocial,
    intent: string
  ): Promise<CrossPostResult> => {
    // In real implementation, parse intent using WIA-INTENT
    console.log(`👨 아버지(INTENT)가 의도 해석 중: "${intent}"`);

    // Mock parsing
    const content: PostContent = {
      text: intent,
    };

    return social.post({ content });
  },

  /**
   * 보안 포스팅 (이모 연동)
   */
  securePost: async (
    social: WIASocial,
    content: PostContent,
    targets: Platform[]
  ): Promise<CrossPostResult> => {
    console.log('🛡️ 이모(AIR-SHIELD)가 콘텐츠 검사 중...');
    const scanResult = await social.scanContent(content);

    if (!scanResult.safe) {
      console.log('🛡️ 이모: 위험해요! 수정이 필요해요.');
      throw new Error('Privacy check failed');
    }

    console.log('🛡️ 이모: 안전해요! 포스팅 진행해도 돼요~');
    return social.post({ content, targets });
  },
};

// ============================================================================
// Export Default
// ============================================================================

export default WIASocial;
