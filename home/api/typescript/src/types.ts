/**
 * WIA-HOME Type Definitions
 * 가족의 집 타입 정의 - "건물주 없는 내 집"
 *
 * @packageDocumentation
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = number;
export type Bytes = number;
export type Megabytes = number;
export type Gigabytes = number;
export type Milliseconds = number;
export type Minutes = number;
export type Days = number;
export type Percentage = number;
export type Mbps = number;
export type BytesPerSecond = number;
export type RequestsPerSecond = number;

export type PageId = string;
export type PostId = string;
export type MediaId = string;
export type ProductId = string;
export type PeerId = string;
export type ContentHash = string;
export type IP = string;

// ============================================================================
// Domain
// ============================================================================

export type WIADomainType = 'home' | 'shop' | 'blog' | 'page';
export type WIADomain = `${string}.wia.${WIADomainType}`;

export interface DomainRecord {
  name: string;
  type: WIADomainType;
  peerId: PeerId;
  publicKey: string;
  created: Timestamp;
  updated: Timestamp;
  verified: boolean;
  customDomain?: string;
}

// ============================================================================
// Server
// ============================================================================

export type ServerStatus = 'starting' | 'running' | 'stopping' | 'stopped' | 'error';

export interface ServerConfig {
  port: number | 'auto';
  maxConnections: number;
  bandwidthLimit?: BytesPerSecond;
  autoStart: boolean;
  sleepMode: boolean;
  sleepAfter: Minutes;
  allowRelay: boolean;
}

export interface ServerResources {
  cpu: Percentage;
  memory: Bytes;
  storage: Bytes;
  bandwidth: BandwidthStats;
}

export interface BandwidthStats {
  inbound: BytesPerSecond;
  outbound: BytesPerSecond;
  totalIn: Bytes;
  totalOut: Bytes;
}

export interface VisitorStats {
  today: number;
  thisWeek: number;
  thisMonth: number;
  total: number;
  activeNow: number;
}

// ============================================================================
// P2P Network
// ============================================================================

export type NATType = 'open' | 'full_cone' | 'restricted' | 'symmetric' | 'unknown';

export interface Peer {
  id: PeerId;
  address: string;
  natType: NATType;
  connected: boolean;
  latency: Milliseconds;
  bandwidth: Mbps;
}

export interface RelayNode {
  id: PeerId;
  address: string;
  region: string;
  load: Percentage;
  available: boolean;
}

export interface PeerAddress {
  peerId: PeerId;
  directAddress?: string;
  relayAddresses: string[];
}

// ============================================================================
// Site Analysis (Cloner)
// ============================================================================

export interface PageInfo {
  url: string;
  title: string;
  type: 'home' | 'about' | 'contact' | 'product' | 'blog' | 'other';
  hasForm: boolean;
  hasMedia: boolean;
}

export interface NavigationStructure {
  type: 'horizontal' | 'vertical' | 'hamburger' | 'mixed';
  items: NavigationItem[];
  depth: number;
}

export interface NavigationItem {
  label: string;
  href: string;
  children?: NavigationItem[];
}

export type LayoutType =
  | 'single-column'
  | 'two-column'
  | 'three-column'
  | 'grid'
  | 'masonry'
  | 'fullscreen';

export interface SiteAnalysis {
  url: string;
  analyzedAt: Timestamp;

  structure: {
    pages: PageInfo[];
    navigation: NavigationStructure;
    layout: LayoutType;
  };

  features: {
    hasShop: boolean;
    hasBlog: boolean;
    hasContact: boolean;
    hasBooking: boolean;
    hasSocialLinks: boolean;
    hasNewsletter: boolean;
    hasSearch: boolean;
    hasLogin: boolean;
  };

  tech: {
    framework?: string;
    cms?: string;
    ecommerce?: string;
    hosting?: string;
  };

  performance: {
    loadTime: Milliseconds;
    size: Bytes;
    score: number;
    issues: string[];
  };

  recommendations: Recommendation[];
}

export interface Recommendation {
  type: 'improvement' | 'feature' | 'warning';
  title: string;
  description: string;
  priority: 'low' | 'medium' | 'high';
}

// ============================================================================
// Clone Options
// ============================================================================

export interface CloneOptions {
  customize: {
    name: string;
    logo?: MediaRef;
    colors?: Partial<ColorScheme>;
    fonts?: Partial<FontScheme>;
    content?: ContentMap;
  };

  features?: {
    shop?: boolean;
    blog?: boolean;
    contact?: boolean;
    booking?: boolean;
  };

  optimize?: {
    images: boolean;
    minify: boolean;
    lazyLoad: boolean;
  };
}

export type ContentMap = Record<string, string | MediaRef>;

// ============================================================================
// Theme
// ============================================================================

export type Color = string;

export interface ColorScheme {
  primary: Color;
  secondary: Color;
  accent: Color;
  background: Color;
  surface: Color;
  text: Color;
  textSecondary: Color;
  error: Color;
  success: Color;
  warning: Color;
}

export interface FontScheme {
  heading: string;
  body: string;
  mono: string;
  sizes: {
    xs: string;
    sm: string;
    base: string;
    lg: string;
    xl: string;
    '2xl': string;
    '3xl': string;
  };
}

export interface SpacingScale {
  xs: string;
  sm: string;
  md: string;
  lg: string;
  xl: string;
}

export interface RadiusScale {
  none: string;
  sm: string;
  md: string;
  lg: string;
  full: string;
}

export interface ShadowScale {
  none: string;
  sm: string;
  md: string;
  lg: string;
  xl: string;
}

export interface Theme {
  name: string;
  colors: ColorScheme;
  fonts: FontScheme;
  spacing: SpacingScale;
  borderRadius: RadiusScale;
  shadows: ShadowScale;
  components: Record<string, unknown>;
}

export interface ColorPalette {
  primary: Color;
  shades: Color[];
  complementary: Color[];
  analogous: Color[];
}

export type LayoutTemplate =
  | 'landing'
  | 'blog'
  | 'portfolio'
  | 'shop'
  | 'dashboard'
  | 'documentation';

// ============================================================================
// Content
// ============================================================================

export interface MediaRef {
  id: MediaId;
  type: 'image' | 'video' | 'audio' | 'document';
  url: string;
  filename: string;
  mimeType: string;
  size: Bytes;
  width?: number;
  height?: number;
  alt?: string;
  optimized?: boolean;
}

export interface Page {
  id: PageId;
  title: string;
  slug: string;
  content: string;
  template?: string;
  meta: PageMeta;
  published: boolean;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

export interface PageMeta {
  description?: string;
  keywords?: string[];
  ogImage?: MediaRef;
  noIndex?: boolean;
}

export interface PageData {
  title: string;
  slug: string;
  content: string;
  template?: string;
  meta?: Partial<PageMeta>;
  published?: boolean;
}

export interface BlogPost {
  id: PostId;
  title: string;
  slug: string;
  content: string;
  excerpt?: string;
  featuredImage?: MediaRef;
  author: string;
  tags: string[];
  categories: string[];
  published: boolean;
  publishedAt?: Timestamp;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

// ============================================================================
// Shop
// ============================================================================

export interface Product {
  id: ProductId;
  name: string;
  slug: string;
  description: string;
  price: number;
  currency: string;
  images: MediaRef[];
  inventory: number;
  sku?: string;
  categories: string[];
  tags: string[];
  variants?: ProductVariant[];
  published: boolean;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

export interface ProductVariant {
  id: string;
  name: string;
  price?: number;
  inventory?: number;
  sku?: string;
  attributes: Record<string, string>;
}

export interface Order {
  id: string;
  items: OrderItem[];
  customer: CustomerInfo;
  shipping: ShippingInfo;
  payment: PaymentInfo;
  status: OrderStatus;
  total: number;
  currency: string;
  createdAt: Timestamp;
}

export interface OrderItem {
  productId: ProductId;
  variantId?: string;
  quantity: number;
  price: number;
}

export interface CustomerInfo {
  name: string;
  email: string;
  phone?: string;
}

export interface ShippingInfo {
  address: string;
  city: string;
  postalCode: string;
  country: string;
  method: string;
}

export interface PaymentInfo {
  method: PaymentMethod;
  status: 'pending' | 'completed' | 'failed' | 'refunded';
  transactionId?: string;
}

export type PaymentMethod =
  | 'kakao_pay'
  | 'naver_pay'
  | 'toss_pay'
  | 'card'
  | 'bank_transfer'
  | 'paypal'
  | 'stripe';

export type OrderStatus =
  | 'pending'
  | 'confirmed'
  | 'processing'
  | 'shipped'
  | 'delivered'
  | 'cancelled'
  | 'refunded';

export interface OrderResult {
  success: boolean;
  orderId: string;
  message?: string;
}

// ============================================================================
// Booking
// ============================================================================

export type DayOfWeek = 'mon' | 'tue' | 'wed' | 'thu' | 'fri' | 'sat' | 'sun';

export interface TimeRange {
  start: string; // HH:mm
  end: string;   // HH:mm
}

export interface BookingAvailability {
  days: DayOfWeek[];
  hours: TimeRange;
  slotDuration: Minutes;
  maxAdvance: Days;
  breaks?: TimeRange[];
  blockedDates?: string[]; // YYYY-MM-DD
}

export interface BookingService {
  id: string;
  name: string;
  duration: Minutes;
  price: number;
  description?: string;
}

export interface Booking {
  id: string;
  serviceId: string;
  customerName: string;
  customerPhone: string;
  customerEmail?: string;
  date: string; // YYYY-MM-DD
  time: string; // HH:mm
  status: 'pending' | 'confirmed' | 'completed' | 'cancelled';
  notes?: string;
  createdAt: Timestamp;
}

export type NotificationChannel = 'email' | 'sms' | 'kakao' | 'push';

export interface BookingNotifications {
  newBooking: NotificationChannel[];
  reminder: NotificationChannel[];
  cancellation: NotificationChannel[];
}

// ============================================================================
// Analytics
// ============================================================================

export interface AnalyticsData {
  period: string;
  visitors: {
    total: number;
    unique: number;
    returning: number;
  };
  pageViews: {
    total: number;
    perPage: Record<string, number>;
  };
  topPages: Array<{ path: string; views: number }>;
  referrers: Array<{ source: string; count: number }>;
  devices: {
    desktop: number;
    mobile: number;
    tablet: number;
  };
  countries: Array<{ code: string; count: number }>;
}

export interface RealtimeData {
  activeVisitors: number;
  currentPages: Array<{ path: string; visitors: number }>;
}

// ============================================================================
// Security
// ============================================================================

export interface SecurityConfig {
  tls: {
    enabled: boolean;
    autoRenew: boolean;
    protocol: string;
  };

  ddos: {
    rateLimit: RequestsPerSecond;
    blockList: IP[];
    p2pDistribution: boolean;
  };

  waf: {
    sqlInjection: boolean;
    xss: boolean;
    csrf: boolean;
  };

  airShield: {
    enabled: boolean;
    scanUploads: boolean;
    protectForms: boolean;
  };
}

export interface PrivacyConfig {
  tracking: {
    analytics: 'privacy-first' | 'full' | 'none';
    cookies: 'essential' | 'functional' | 'all';
    doNotTrack: boolean;
  };

  data: {
    encryption: boolean;
    localOnly: boolean;
    autoDelete: Days;
  };

  compliance: {
    gdpr: boolean;
    ccpa: boolean;
    pipa: boolean;
  };
}

// ============================================================================
// Low Spec Mode
// ============================================================================

export interface LowSpecConfig {
  limits: {
    maxMemory: Megabytes;
    maxCpu: Percentage;
    maxStorage: Gigabytes;
    maxBandwidth: Mbps;
  };

  powerSaving: {
    enabled: boolean;
    sleepAfterInactive: Minutes;
    wakeOnRequest: boolean;
  };

  optimization: {
    imageCompression: boolean;
    lazyLoading: boolean;
    minimalJs: boolean;
  };
}

// ============================================================================
// Platform Integration
// ============================================================================

export interface PlatformIntegration {
  naver?: {
    place?: boolean;
    placeId?: string;
    smartstore?: boolean;
    blog?: boolean;
    map?: boolean;
  };

  kakao?: {
    channel?: boolean;
    channelId?: string;
    map?: boolean;
    pay?: boolean;
    sync?: boolean;
  };

  delivery?: {
    baemin?: boolean;
    yogiyo?: boolean;
    coupangeats?: boolean;
  };
}

/**
 * 글로벌 결제 통합
 * 홍익인간 (弘益人間) - 모든 인류의 소상공인을 위해
 * 케냐도, 에티오피아도, 브라질도, 인도도... 모두!
 */
export interface PaymentIntegration {
  // 🇰🇷 한국
  korean?: {
    kakaoPay?: boolean;
    naverPay?: boolean;
    tossPay?: boolean;
    payco?: boolean;
  };

  // 🌍 아프리카 - 모바일 머니가 은행보다 더 보편적
  africa?: {
    mPesa?: boolean;        // 케냐, 탄자니아 (3억명 사용)
    airtelMoney?: boolean;  // 우간다, 말라위, 잠비아
    orangeMoney?: boolean;  // 세네갈, 코트디부아르, 말리
    mtnMoney?: boolean;     // 가나, 카메룬, 르완다
    ecocash?: boolean;      // 짐바브웨
    tigoPesa?: boolean;     // 탄자니아, 가나
    vodacom?: boolean;      // 남아프리카, 모잠비크
    telebirr?: boolean;     // 에티오피아 (1억명 인구)
  };

  // 🇮🇳 인도/남아시아 - UPI가 혁명
  india?: {
    upi?: boolean;          // 인도 통합결제 (10억명 사용)
    paytm?: boolean;
    phonePe?: boolean;
    googlePay?: boolean;
    razorpay?: boolean;
    bhim?: boolean;
  };

  // 🌏 동남아시아
  southeastAsia?: {
    grabPay?: boolean;      // 싱가포르, 말레이시아, 필리핀
    goPay?: boolean;        // 인도네시아
    ovo?: boolean;          // 인도네시아
    dana?: boolean;         // 인도네시아
    gcash?: boolean;        // 필리핀
    maya?: boolean;         // 필리핀 (구 PayMaya)
    promptPay?: boolean;    // 태국
    trueMoney?: boolean;    // 태국
    momo?: boolean;         // 베트남
    zalopay?: boolean;      // 베트남
    touchNGo?: boolean;     // 말레이시아
  };

  // 🌎 중남미
  latinAmerica?: {
    pix?: boolean;          // 브라질 (1.5억명 사용, 즉시이체)
    mercadoPago?: boolean;  // 아르헨티나, 멕시코, 브라질
    oxxo?: boolean;         // 멕시코 (편의점 결제)
    nequi?: boolean;        // 콜롬비아
    daviplata?: boolean;    // 콜롬비아
    yape?: boolean;         // 페루
    plin?: boolean;         // 페루
    fpay?: boolean;         // 칠레
  };

  // 🕌 중동/북아프리카
  middleEast?: {
    stcPay?: boolean;       // 사우디아라비아
    mada?: boolean;         // 사우디아라비아
    benefit?: boolean;      // 바레인
    knet?: boolean;         // 쿠웨이트
    fawry?: boolean;        // 이집트
    vodafoneCash?: boolean; // 이집트
    jawwalPay?: boolean;    // 팔레스타인
    zaincash?: boolean;     // 이라크
  };

  // 🇨🇳 중국
  china?: {
    alipay?: boolean;
    wechatPay?: boolean;
    unionPay?: boolean;
  };

  // 🇯🇵 일본
  japan?: {
    paypay?: boolean;
    linePay?: boolean;
    rakutenPay?: boolean;
    merpay?: boolean;
  };

  // 🇪🇺 유럽
  europe?: {
    sepa?: boolean;         // 유로존 즉시이체
    ideal?: boolean;        // 네덜란드
    bancontact?: boolean;   // 벨기에
    giropay?: boolean;      // 독일
    sofort?: boolean;       // 독일, 오스트리아
    swish?: boolean;        // 스웨덴
    vipps?: boolean;        // 노르웨이
    mobilePay?: boolean;    // 덴마크
    bizum?: boolean;        // 스페인
    mbway?: boolean;        // 포르투갈
  };

  // 🌐 글로벌
  global?: {
    stripe?: boolean;
    paypal?: boolean;
    wise?: boolean;         // 국제 송금
    remitly?: boolean;      // 해외 노동자 송금
  };

  // 💰 암호화폐 - 은행 인프라 없는 곳도 OK
  crypto?: {
    bitcoin?: boolean;
    ethereum?: boolean;
    usdc?: boolean;         // 스테이블코인
    usdt?: boolean;
    lightningNetwork?: boolean; // 소액결제
  };

  // 🏦 은행 이체 (전 세계 공통)
  bankTransfer?: {
    enabled: boolean;
    accounts: BankAccount[];
  };

  // 📱 통신사 결제 - 스마트폰만 있으면 OK
  carrierBilling?: {
    enabled: boolean;
    carriers?: string[];    // 지원 통신사 목록
  };
}

export interface BankAccount {
  bank: string;
  number: string;
  holder: string;
}

// ============================================================================
// Migration
// ============================================================================

export type MigrationSource =
  | 'wordpress'
  | 'wix'
  | 'squarespace'
  | 'shopify'
  | 'naver-blog'
  | 'tistory';

export interface MigrationConfig {
  from: MigrationSource;
  url?: string;
  exportFile?: string;
  credentials?: {
    username: string;
    password: string;
  };
  options?: {
    posts?: boolean;
    pages?: boolean;
    media?: boolean;
    comments?: boolean;
    products?: boolean;
  };
}

export interface MigrationResult {
  success: boolean;
  imported: {
    pages: number;
    posts: number;
    media: number;
    products: number;
  };
  errors: string[];
}

// ============================================================================
// Configuration
// ============================================================================

export interface WIAHomeConfig {
  name: string;
  domain?: WIADomain;

  server?: Partial<ServerConfig>;
  security?: Partial<SecurityConfig>;
  privacy?: Partial<PrivacyConfig>;
  lowSpecMode?: Partial<LowSpecConfig>;

  features?: {
    shop?: boolean;
    blog?: boolean;
    booking?: boolean;
  };

  integrations?: PlatformIntegration;
  payment?: PaymentIntegration;

  theme?: Partial<Theme>;
}

// ============================================================================
// Events
// ============================================================================

export type HomeEventType =
  | 'server_started'
  | 'server_stopped'
  | 'visitor_connected'
  | 'page_viewed'
  | 'order_received'
  | 'booking_received'
  | 'contact_received'
  | 'error';

export interface HomeEvent {
  type: HomeEventType;
  timestamp: Timestamp;
  data: unknown;
}

export type EventCallback = (event: HomeEvent) => void;
export type Unsubscribe = () => void;

// ============================================================================
// Quick Start
// ============================================================================

export interface QuickStartOptions {
  benchmark: string;
  myInfo: {
    name: string;
    phone?: string;
    email?: string;
    address?: string;
    hours?: string;
    description?: string;
  };
}

// ============================================================================
// Template
// ============================================================================

export type IndustryTemplate =
  | 'restaurant'
  | 'cafe'
  | 'bakery'
  | 'hairSalon'
  | 'beautySalon'
  | 'clinic'
  | 'dental'
  | 'academy'
  | 'gym'
  | 'yoga'
  | 'retailShop'
  | 'florist'
  | 'petShop'
  | 'photographer'
  | 'designer'
  | 'developer'
  | 'consultant'
  | 'lawyer'
  | 'realEstate';

export interface TemplateConfig {
  industry: IndustryTemplate;
  locale?: string;
  features?: string[];
}
