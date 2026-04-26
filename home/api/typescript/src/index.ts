/**
 * WIA-HOME SDK
 * 가족의 집 TypeScript 구현 - "건물주 없는 내 집" 🏠
 *
 * @packageDocumentation
 */

import {
  WIAHomeConfig,
  WIADomain,
  ServerConfig,
  ServerStatus,
  ServerResources,
  VisitorStats,
  BandwidthStats,
  Peer,
  PeerId,
  NATType,
  RelayNode,
  SiteAnalysis,
  CloneOptions,
  Theme,
  ColorScheme,
  Page,
  PageData,
  PageId,
  BlogPost,
  PostId,
  MediaRef,
  MediaId,
  Product,
  ProductId,
  Order,
  OrderResult,
  BookingAvailability,
  BookingService,
  Booking,
  BookingNotifications,
  AnalyticsData,
  RealtimeData,
  SecurityConfig,
  PrivacyConfig,
  LowSpecConfig,
  PlatformIntegration,
  PaymentIntegration,
  MigrationConfig,
  MigrationResult,
  QuickStartOptions,
  IndustryTemplate,
  TemplateConfig,
  HomeEvent,
  HomeEventType,
  EventCallback,
  Unsubscribe,
  Timestamp,
  Bytes,
  Percentage,
} from './types';

// Re-export all types
export * from './types';

// ============================================================================
// Default Configuration
// ============================================================================

const DEFAULT_SERVER_CONFIG: ServerConfig = {
  port: 'auto',
  maxConnections: 100,
  autoStart: false,
  sleepMode: true,
  sleepAfter: 10,
  allowRelay: true,
};

const DEFAULT_CONFIG: WIAHomeConfig = {
  name: 'My Home',
  server: DEFAULT_SERVER_CONFIG,
  features: {
    shop: false,
    blog: false,
    booking: false,
  },
};

// ============================================================================
// Site Analyzer
// ============================================================================

class SiteAnalyzer {
  async analyze(url: string): Promise<SiteAnalysis> {
    console.log(`🔍 사이트 분석 중: ${url}`);

    // In real implementation, crawl and analyze the site
    // This is a simplified mock

    const analysis: SiteAnalysis = {
      url,
      analyzedAt: Date.now(),
      structure: {
        pages: [
          { url: '/', title: 'Home', type: 'home', hasForm: false, hasMedia: true },
          { url: '/about', title: 'About', type: 'about', hasForm: false, hasMedia: true },
          { url: '/contact', title: 'Contact', type: 'contact', hasForm: true, hasMedia: false },
        ],
        navigation: {
          type: 'horizontal',
          items: [
            { label: 'Home', href: '/' },
            { label: 'About', href: '/about' },
            { label: 'Contact', href: '/contact' },
          ],
          depth: 1,
        },
        layout: 'single-column',
      },
      features: {
        hasShop: false,
        hasBlog: false,
        hasContact: true,
        hasBooking: false,
        hasSocialLinks: true,
        hasNewsletter: false,
        hasSearch: false,
        hasLogin: false,
      },
      tech: {
        framework: 'unknown',
      },
      performance: {
        loadTime: 2500,
        size: 1500000,
        score: 75,
        issues: [],
      },
      recommendations: [
        {
          type: 'improvement',
          title: '이미지 최적화',
          description: '이미지를 WebP 형식으로 변환하면 로딩 속도가 향상됩니다',
          priority: 'medium',
        },
      ],
    };

    console.log('✅ 분석 완료');
    return analysis;
  }

  async clone(url: string, options: CloneOptions): Promise<WIAHome> {
    console.log(`🔄 사이트 클론 중: ${url}`);

    const analysis = await this.analyze(url);

    // Create new home based on analysis
    const home = new WIAHome({
      name: options.customize.name,
      features: {
        shop: options.features?.shop ?? analysis.features.hasShop,
        blog: options.features?.blog ?? analysis.features.hasBlog,
        booking: options.features?.booking ?? analysis.features.hasBooking,
      },
    });

    // Apply customization
    if (options.customize.colors) {
      home.setTheme({ colors: options.customize.colors as ColorScheme });
    }

    console.log('✅ 클론 완료');
    return home;
  }
}

// ============================================================================
// P2P Network Manager
// ============================================================================

class P2PNetworkManager {
  private peerId?: PeerId;
  private peers: Map<PeerId, Peer> = new Map();
  private natType: NATType = 'unknown';
  private connected: boolean = false;

  async join(): Promise<PeerId> {
    this.peerId = `peer_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    this.connected = true;

    // Detect NAT type
    this.natType = await this.detectNATType();

    console.log(`🌐 P2P 네트워크 참여: ${this.peerId}`);
    console.log(`📡 NAT 타입: ${this.natType}`);

    return this.peerId;
  }

  async leave(): Promise<void> {
    this.connected = false;
    this.peers.clear();
    console.log('🌐 P2P 네트워크 탈퇴');
  }

  private async detectNATType(): Promise<NATType> {
    // In real implementation, perform STUN checks
    return 'full_cone';
  }

  getPeers(): Peer[] {
    return Array.from(this.peers.values());
  }

  getConnectedCount(): number {
    return this.peers.size;
  }

  getNATType(): NATType {
    return this.natType;
  }

  isConnected(): boolean {
    return this.connected;
  }

  async getRelayNodes(): Promise<RelayNode[]> {
    // In real implementation, fetch from relay network
    return [
      {
        id: 'relay_kr_1',
        address: 'kr1.relay.wia.home',
        region: 'Korea',
        load: 30,
        available: true,
      },
      {
        id: 'relay_jp_1',
        address: 'jp1.relay.wia.home',
        region: 'Japan',
        load: 45,
        available: true,
      },
    ];
  }
}

// ============================================================================
// WIA DNS Manager
// ============================================================================

class WIADNSManager {
  private registeredDomains: Map<string, string> = new Map();

  async register(name: string, type: 'home' | 'shop' | 'blog' | 'page' = 'home'): Promise<WIADomain> {
    const domain = `${name}.wia.${type}` as WIADomain;

    // In real implementation, register with WIA DNS network
    this.registeredDomains.set(name, domain);

    console.log(`🌐 도메인 등록: ${domain}`);
    return domain;
  }

  async resolve(domain: WIADomain): Promise<PeerId | null> {
    // In real implementation, resolve from WIA DNS network
    return `peer_${domain.split('.')[0]}`;
  }

  async linkCustomDomain(wiaDomain: WIADomain, customDomain: string): Promise<void> {
    console.log(`🔗 커스텀 도메인 연결: ${customDomain} → ${wiaDomain}`);
  }
}

// ============================================================================
// Personal Server
// ============================================================================

class PersonalServer {
  private config: ServerConfig;
  private _status: ServerStatus = 'stopped';
  private startTime?: Timestamp;
  private requestCount: number = 0;

  constructor(config: ServerConfig) {
    this.config = config;
  }

  async start(): Promise<void> {
    this._status = 'starting';
    console.log('🚀 서버 시작 중...');

    // In real implementation, start HTTP server
    await new Promise((resolve) => setTimeout(resolve, 500));

    this._status = 'running';
    this.startTime = Date.now();

    const port = this.config.port === 'auto' ? 8080 : this.config.port;
    console.log(`✅ 서버 실행 중: http://localhost:${port}`);
  }

  async stop(): Promise<void> {
    this._status = 'stopping';
    console.log('🛑 서버 중지 중...');

    await new Promise((resolve) => setTimeout(resolve, 200));

    this._status = 'stopped';
    this.startTime = undefined;
    console.log('✅ 서버 중지됨');
  }

  get status(): ServerStatus {
    return this._status;
  }

  get uptime(): number {
    if (!this.startTime) return 0;
    return Date.now() - this.startTime;
  }

  getResources(): ServerResources {
    return {
      cpu: 5,
      memory: 128 * 1024 * 1024, // 128MB
      storage: 500 * 1024 * 1024, // 500MB
      bandwidth: {
        inbound: 1024,
        outbound: 2048,
        totalIn: this.requestCount * 5000,
        totalOut: this.requestCount * 50000,
      },
    };
  }

  getVisitorStats(): VisitorStats {
    return {
      today: Math.floor(Math.random() * 100),
      thisWeek: Math.floor(Math.random() * 500),
      thisMonth: Math.floor(Math.random() * 2000),
      total: Math.floor(Math.random() * 10000),
      activeNow: Math.floor(Math.random() * 10),
    };
  }
}

// ============================================================================
// Content Manager
// ============================================================================

class ContentManager {
  private _pages: Map<PageId, Page> = new Map();
  private _posts: Map<PostId, BlogPost> = new Map();
  private _media: Map<MediaId, MediaRef> = new Map();
  private _products: Map<ProductId, Product> = new Map();

  // Pages
  pages = {
    create: async (data: PageData): Promise<Page> => {
      const page: Page = {
        id: `page_${Date.now()}`,
        ...data,
        meta: data.meta || {},
        published: data.published ?? true,
        createdAt: Date.now(),
        updatedAt: Date.now(),
      };
      this._pages.set(page.id, page);
      console.log(`📄 페이지 생성: ${page.title}`);
      return page;
    },

    update: async (id: PageId, data: Partial<PageData>): Promise<Page> => {
      const page = this._pages.get(id);
      if (!page) throw new Error('Page not found');
      const updated = { ...page, ...data, updatedAt: Date.now() };
      this._pages.set(id, updated);
      return updated;
    },

    delete: async (id: PageId): Promise<void> => {
      this._pages.delete(id);
    },

    list: async (): Promise<Page[]> => {
      return Array.from(this._pages.values());
    },

    get: async (id: PageId): Promise<Page | undefined> => {
      return this._pages.get(id);
    },
  };

  // Blog
  blog = {
    createPost: async (data: Omit<BlogPost, 'id' | 'createdAt' | 'updatedAt'>): Promise<BlogPost> => {
      const post: BlogPost = {
        id: `post_${Date.now()}`,
        ...data,
        createdAt: Date.now(),
        updatedAt: Date.now(),
      };
      this._posts.set(post.id, post);
      console.log(`📝 블로그 포스트 생성: ${post.title}`);
      return post;
    },

    updatePost: async (id: PostId, data: Partial<BlogPost>): Promise<BlogPost> => {
      const post = this._posts.get(id);
      if (!post) throw new Error('Post not found');
      const updated = { ...post, ...data, updatedAt: Date.now() };
      this._posts.set(id, updated);
      return updated;
    },

    deletePost: async (id: PostId): Promise<void> => {
      this._posts.delete(id);
    },

    listPosts: async (): Promise<BlogPost[]> => {
      return Array.from(this._posts.values());
    },
  };

  // Media
  media = {
    upload: async (file: { name: string; type: string; size: number }): Promise<MediaRef> => {
      const media: MediaRef = {
        id: `media_${Date.now()}`,
        type: file.type.startsWith('image') ? 'image' : 'document',
        url: `/media/${file.name}`,
        filename: file.name,
        mimeType: file.type,
        size: file.size,
      };
      this._media.set(media.id, media);
      console.log(`📷 미디어 업로드: ${file.name}`);
      return media;
    },

    delete: async (id: MediaId): Promise<void> => {
      this._media.delete(id);
    },

    list: async (): Promise<MediaRef[]> => {
      return Array.from(this._media.values());
    },

    optimize: async (id: MediaId): Promise<MediaRef> => {
      const media = this._media.get(id);
      if (!media) throw new Error('Media not found');
      const optimized = { ...media, optimized: true };
      this._media.set(id, optimized);
      console.log(`⚡ 미디어 최적화: ${media.filename}`);
      return optimized;
    },
  };

  // Shop
  shop = {
    addProduct: async (data: Omit<Product, 'id' | 'createdAt' | 'updatedAt'>): Promise<Product> => {
      const product: Product = {
        id: `product_${Date.now()}`,
        ...data,
        createdAt: Date.now(),
        updatedAt: Date.now(),
      };
      this._products.set(product.id, product);
      console.log(`🛍️ 상품 추가: ${product.name}`);
      return product;
    },

    updateProduct: async (id: ProductId, data: Partial<Product>): Promise<Product> => {
      const product = this._products.get(id);
      if (!product) throw new Error('Product not found');
      const updated = { ...product, ...data, updatedAt: Date.now() };
      this._products.set(id, updated);
      return updated;
    },

    removeProduct: async (id: ProductId): Promise<void> => {
      this._products.delete(id);
    },

    listProducts: async (): Promise<Product[]> => {
      return Array.from(this._products.values());
    },

    processOrder: async (order: Order): Promise<OrderResult> => {
      console.log(`📦 주문 처리: ${order.id}`);
      return {
        success: true,
        orderId: order.id,
        message: '주문이 접수되었습니다',
      };
    },

    setPayment: (config: PaymentIntegration): void => {
      console.log('💳 결제 설정 완료');
    },
  };
}

// ============================================================================
// Booking Manager
// ============================================================================

class BookingManager {
  private availability?: BookingAvailability;
  private services: BookingService[] = [];
  private bookings: Booking[] = [];
  private notifications?: BookingNotifications;

  setup(config: {
    availability: BookingAvailability;
    services: BookingService[];
    notifications?: BookingNotifications;
  }): void {
    this.availability = config.availability;
    this.services = config.services;
    this.notifications = config.notifications;
    console.log('📅 예약 시스템 설정 완료');
  }

  async createBooking(data: Omit<Booking, 'id' | 'status' | 'createdAt'>): Promise<Booking> {
    const booking: Booking = {
      id: `booking_${Date.now()}`,
      ...data,
      status: 'pending',
      createdAt: Date.now(),
    };
    this.bookings.push(booking);
    console.log(`📅 예약 생성: ${booking.customerName} - ${booking.date} ${booking.time}`);
    return booking;
  }

  async confirmBooking(id: string): Promise<void> {
    const booking = this.bookings.find((b) => b.id === id);
    if (booking) {
      booking.status = 'confirmed';
      console.log(`✅ 예약 확정: ${id}`);
    }
  }

  async cancelBooking(id: string): Promise<void> {
    const booking = this.bookings.find((b) => b.id === id);
    if (booking) {
      booking.status = 'cancelled';
      console.log(`❌ 예약 취소: ${id}`);
    }
  }

  getBookings(status?: Booking['status']): Booking[] {
    if (status) {
      return this.bookings.filter((b) => b.status === status);
    }
    return this.bookings;
  }

  getAvailableSlots(date: string): string[] {
    if (!this.availability) return [];

    const slots: string[] = [];
    const start = parseInt(this.availability.hours.start.split(':')[0]);
    const end = parseInt(this.availability.hours.end.split(':')[0]);
    const duration = this.availability.slotDuration;

    for (let hour = start; hour < end; hour++) {
      for (let minute = 0; minute < 60; minute += duration) {
        const time = `${hour.toString().padStart(2, '0')}:${minute.toString().padStart(2, '0')}`;
        // Check if slot is available
        const booked = this.bookings.some(
          (b) => b.date === date && b.time === time && b.status !== 'cancelled'
        );
        if (!booked) {
          slots.push(time);
        }
      }
    }

    return slots;
  }
}

// ============================================================================
// Analytics Manager
// ============================================================================

class AnalyticsManager {
  async get(options: { period: string }): Promise<AnalyticsData> {
    // In real implementation, fetch from analytics store
    return {
      period: options.period,
      visitors: {
        total: 1234,
        unique: 890,
        returning: 344,
      },
      pageViews: {
        total: 5678,
        perPage: {
          '/': 2000,
          '/about': 800,
          '/products': 1500,
        },
      },
      topPages: [
        { path: '/', views: 2000 },
        { path: '/products', views: 1500 },
        { path: '/about', views: 800 },
      ],
      referrers: [
        { source: 'google', count: 500 },
        { source: 'instagram', count: 300 },
        { source: 'direct', count: 200 },
      ],
      devices: {
        desktop: 40,
        mobile: 55,
        tablet: 5,
      },
      countries: [
        { code: 'KR', count: 800 },
        { code: 'US', count: 100 },
        { code: 'JP', count: 50 },
      ],
    };
  }

  realtime(callback: (data: RealtimeData) => void): Unsubscribe {
    const interval = setInterval(() => {
      callback({
        activeVisitors: Math.floor(Math.random() * 20),
        currentPages: [
          { path: '/', visitors: Math.floor(Math.random() * 10) },
          { path: '/products', visitors: Math.floor(Math.random() * 5) },
        ],
      });
    }, 5000);

    return () => clearInterval(interval);
  }
}

// ============================================================================
// Main WIAHome Class
// ============================================================================

/**
 * WIA-HOME 메인 클래스
 * 가족의 집 - "건물주 없는 내 집" 🏠
 */
export class WIAHome {
  private config: WIAHomeConfig;
  private _domain?: WIADomain;

  // Components
  private server: PersonalServer;
  private p2p: P2PNetworkManager;
  private dns: WIADNSManager;
  private content: ContentManager;
  private _booking: BookingManager;
  private _analytics: AnalyticsManager;

  // Static analyzer
  private static analyzer = new SiteAnalyzer();

  // Event handlers
  private eventHandlers: Map<HomeEventType, Set<EventCallback>> = new Map();

  constructor(config: Partial<WIAHomeConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };

    // Initialize components
    this.server = new PersonalServer(this.config.server || DEFAULT_SERVER_CONFIG);
    this.p2p = new P2PNetworkManager();
    this.dns = new WIADNSManager();
    this.content = new ContentManager();
    this._booking = new BookingManager();
    this._analytics = new AnalyticsManager();

    console.log(`🏠 WIA-HOME 준비 완료: "${this.config.name}"`);
    console.log('💡 "건물주 없는 내 집"');
  }

  // ============================================================================
  // Static Methods
  // ============================================================================

  /**
   * URL 분석
   */
  static async analyze(url: string): Promise<SiteAnalysis> {
    return WIAHome.analyzer.analyze(url);
  }

  /**
   * 사이트 클론
   */
  static async clone(url: string, options: CloneOptions): Promise<WIAHome> {
    return WIAHome.analyzer.clone(url, options);
  }

  /**
   * 빠른 시작 (벤치마킹 + 내 정보)
   */
  static async quickStart(options: QuickStartOptions): Promise<WIAHome> {
    console.log('🚀 빠른 시작...');

    const analysis = await WIAHome.analyze(options.benchmark);

    const home = new WIAHome({
      name: options.myInfo.name,
      features: {
        shop: analysis.features.hasShop,
        blog: analysis.features.hasBlog,
        booking: analysis.features.hasBooking,
      },
    });

    // Add basic pages with user info
    await home.pages.create({
      title: 'Home',
      slug: '',
      content: `
        <h1>${options.myInfo.name}</h1>
        <p>${options.myInfo.description || ''}</p>
        ${options.myInfo.address ? `<p>주소: ${options.myInfo.address}</p>` : ''}
        ${options.myInfo.phone ? `<p>전화: ${options.myInfo.phone}</p>` : ''}
        ${options.myInfo.hours ? `<p>영업시간: ${options.myInfo.hours}</p>` : ''}
      `,
    });

    console.log('✅ 빠른 시작 완료!');
    return home;
  }

  /**
   * 의도 기반 생성 (아버지 연동)
   */
  static async fromIntent(intent: string): Promise<WIAHome> {
    console.log('👨 아버지(INTENT)가 집 설계 중...');
    console.log(`📝 의도: "${intent}"`);

    // In real implementation, parse intent using WIA-INTENT
    // This is a simplified version

    const home = new WIAHome({
      name: 'My Home',
    });

    // Mock intent parsing
    if (intent.includes('빵집') || intent.includes('베이커리')) {
      home.config.features = { shop: true, blog: false, booking: false };
    } else if (intent.includes('미용실') || intent.includes('헤어')) {
      home.config.features = { shop: false, blog: false, booking: true };
    } else if (intent.includes('포트폴리오') || intent.includes('디자이너')) {
      home.config.features = { shop: false, blog: true, booking: false };
    }

    console.log('✅ 집 설계 완료!');
    return home;
  }

  /**
   * 템플릿에서 생성
   */
  static async fromTemplate(template: IndustryTemplate, config?: TemplateConfig): Promise<WIAHome> {
    console.log(`📋 템플릿 적용: ${template}`);

    const templateFeatures: Record<IndustryTemplate, WIAHomeConfig['features']> = {
      restaurant: { shop: true, blog: false, booking: true },
      cafe: { shop: true, blog: false, booking: false },
      bakery: { shop: true, blog: false, booking: false },
      hairSalon: { shop: false, blog: false, booking: true },
      beautySalon: { shop: true, blog: false, booking: true },
      clinic: { shop: false, blog: false, booking: true },
      dental: { shop: false, blog: false, booking: true },
      academy: { shop: false, blog: true, booking: true },
      gym: { shop: true, blog: false, booking: true },
      yoga: { shop: true, blog: true, booking: true },
      retailShop: { shop: true, blog: false, booking: false },
      florist: { shop: true, blog: false, booking: true },
      petShop: { shop: true, blog: true, booking: true },
      photographer: { shop: false, blog: true, booking: true },
      designer: { shop: false, blog: true, booking: false },
      developer: { shop: false, blog: true, booking: false },
      consultant: { shop: false, blog: true, booking: true },
      lawyer: { shop: false, blog: true, booking: true },
      realEstate: { shop: false, blog: true, booking: true },
    };

    const home = new WIAHome({
      name: `My ${template}`,
      features: templateFeatures[template],
    });

    console.log('✅ 템플릿 적용 완료!');
    return home;
  }

  /**
   * 마이그레이션 (기존 사이트 이전)
   */
  static async migrate(config: MigrationConfig): Promise<WIAHome> {
    console.log(`📦 마이그레이션: ${config.from}`);

    // In real implementation, import from source
    const home = new WIAHome({
      name: 'Migrated Site',
    });

    const result: MigrationResult = {
      success: true,
      imported: {
        pages: 10,
        posts: 25,
        media: 50,
        products: 0,
      },
      errors: [],
    };

    console.log(`✅ 마이그레이션 완료: ${result.imported.pages}개 페이지, ${result.imported.posts}개 포스트`);
    return home;
  }

  // ============================================================================
  // Instance Methods
  // ============================================================================

  /**
   * 서버 시작
   */
  async start(): Promise<void> {
    // Join P2P network
    await this.p2p.join();

    // Register domain
    if (!this._domain) {
      const name = this.config.name.toLowerCase().replace(/\s+/g, '-');
      this._domain = await this.dns.register(name);
    }

    // Start server
    await this.server.start();

    this.emit('server_started', { domain: this._domain });
    console.log(`🏠 ${this._domain} 오픈!`);
  }

  /**
   * 서버 중지
   */
  async stop(): Promise<void> {
    await this.server.stop();
    await this.p2p.leave();
    this.emit('server_stopped', {});
  }

  /**
   * 도메인 가져오기
   */
  get domain(): WIADomain | undefined {
    return this._domain;
  }

  /**
   * 상태 가져오기
   */
  get status(): ServerStatus {
    return this.server.status;
  }

  /**
   * 통계 가져오기
   */
  get stats(): { visitors: VisitorStats; resources: ServerResources } {
    return {
      visitors: this.server.getVisitorStats(),
      resources: this.server.getResources(),
    };
  }

  // ============================================================================
  // Content Management
  // ============================================================================

  get pages() {
    return this.content.pages;
  }

  get blog() {
    return this.content.blog;
  }

  get media() {
    return this.content.media;
  }

  get shop() {
    return this.content.shop;
  }

  // ============================================================================
  // Booking
  // ============================================================================

  get booking() {
    return this._booking;
  }

  // ============================================================================
  // Analytics
  // ============================================================================

  get analytics() {
    return this._analytics;
  }

  // ============================================================================
  // Theme
  // ============================================================================

  setTheme(theme: Partial<Theme>): void {
    this.config.theme = { ...this.config.theme, ...theme };
    console.log('🎨 테마 적용됨');
  }

  // ============================================================================
  // Integrations
  // ============================================================================

  async integrate(config: PlatformIntegration): Promise<void> {
    this.config.integrations = { ...this.config.integrations, ...config };
    console.log('🔗 플랫폼 연동 완료');
  }

  async connectDomain(customDomain: string): Promise<void> {
    if (this._domain) {
      await this.dns.linkCustomDomain(this._domain, customDomain);
    }
  }

  // ============================================================================
  // Events
  // ============================================================================

  on(event: HomeEventType, callback: EventCallback): Unsubscribe {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set());
    }
    this.eventHandlers.get(event)!.add(callback);
    return () => this.eventHandlers.get(event)?.delete(callback);
  }

  private emit(event: HomeEventType, data: unknown): void {
    const handlers = this.eventHandlers.get(event);
    if (handlers) {
      const homeEvent: HomeEvent = {
        type: event,
        timestamp: Date.now(),
        data,
      };
      handlers.forEach((handler) => handler(homeEvent));
    }
  }
}

// ============================================================================
// Export Default
// ============================================================================

export default WIAHome;
