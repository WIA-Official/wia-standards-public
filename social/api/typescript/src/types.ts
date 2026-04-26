/**
 * WIA-SOCIAL Type Definitions
 * 조카의 타입 정의 - "나 통해서 다 연결돼!"
 *
 * @packageDocumentation
 */

// ============================================================================
// Basic Types
// ============================================================================

export type Timestamp = number;
export type PostId = string;
export type UserId = string;
export type PlatformPostId = string;
export type PlatformUserId = string;
export type ConversationId = string;
export type WIAId = `wia:${string}`;

// ============================================================================
// Platforms
// ============================================================================

/**
 * 지원 플랫폼
 */
export type Platform =
  // 메이저 플랫폼
  | 'instagram'
  | 'twitter'
  | 'facebook'
  | 'tiktok'
  | 'youtube'
  | 'linkedin'
  | 'pinterest'
  | 'snapchat'
  | 'reddit'
  | 'threads'
  // 분산형 소셜
  | 'mastodon'
  | 'bluesky'
  | 'nostr'
  | 'farcaster'
  | 'lens'
  // 메신저
  | 'telegram'
  | 'discord'
  | 'slack'
  | 'whatsapp'
  // 커스텀
  | 'custom';

/**
 * 플랫폼 기능
 */
export interface PlatformCapabilities {
  posting: boolean;
  stories: boolean;
  reels: boolean;
  liveStreaming: boolean;
  messaging: boolean;
  groups: boolean;
  events: boolean;
  polls: boolean;
  scheduling: boolean;
  analytics: boolean;
  export: boolean;
  import: boolean;
}

/**
 * 플랫폼 연결 정보
 */
export interface PlatformConnection {
  platform: Platform;
  handle: string;
  userId: PlatformUserId;
  connected: boolean;
  connectedAt: Timestamp;
  permissions: string[];
  capabilities: PlatformCapabilities;
}

// ============================================================================
// Identity
// ============================================================================

/**
 * 통합 정체성
 */
export interface UniversalIdentity {
  wia_id: WIAId;
  displayName: string;
  username: string;
  profile: UserProfile;
  connectedPlatforms: PlatformConnection[];
  privacy: PrivacySettings;
  verification: VerificationStatus;
  createdAt: Timestamp;
  updatedAt: Timestamp;
}

/**
 * 사용자 프로필
 */
export interface UserProfile {
  avatar?: MediaRef;
  cover?: MediaRef;
  bio: string;
  location?: string;
  website?: string;
  links: ProfileLink[];
  badges: Badge[];
}

export interface ProfileLink {
  title: string;
  url: string;
  verified: boolean;
}

export interface Badge {
  type: 'verified' | 'certified' | 'ambassador' | 'creator' | 'business';
  issuedAt: Timestamp;
  expiresAt?: Timestamp;
}

/**
 * 인증 상태
 */
export interface VerificationStatus {
  level: 'none' | 'basic' | 'verified' | 'certified' | 'ambassador';
  email: boolean;
  phone: boolean;
  identity: boolean;
  platformCount: number;
}

// ============================================================================
// Privacy (이모 연동)
// ============================================================================

/**
 * 프라이버시 레벨
 */
export type PrivacyLevel =
  | 'public'     // 모두에게 공개
  | 'protected'  // 팔로워만
  | 'private'    // 친구만
  | 'secret'     // 특정인만
  | 'anonymous'; // 익명

/**
 * 프라이버시 설정
 */
export interface PrivacySettings {
  defaultVisibility: PrivacyLevel;
  platformOverrides: Partial<Record<Platform, PrivacyLevel>>;
  dataCollection: DataCollectionSettings;
  airShield: AirShieldIntegration;
}

export interface DataCollectionSettings {
  allowAnalytics: boolean;
  allowTargetedAds: boolean;
  allowThirdPartySharing: boolean;
  allowLocationTracking: boolean;
}

export interface AirShieldIntegration {
  enabled: boolean;
  protectionMode: 'balanced' | 'fortress' | 'paranoid';
  autoScan: boolean;
}

// ============================================================================
// Media
// ============================================================================

/**
 * 미디어 참조
 */
export interface MediaRef {
  id: string;
  type: MediaType;
  url: string;
  thumbnailUrl?: string;
  width?: number;
  height?: number;
  duration?: number; // seconds, for video/audio
  mimeType: string;
  size: number; // bytes
  alt?: string;
  metadata?: MediaMetadata;
}

export type MediaType = 'image' | 'video' | 'audio' | 'gif' | 'document';

export interface MediaMetadata {
  location?: GeoLocation;
  takenAt?: Timestamp;
  camera?: string;
  // 이모가 제거해야 할 민감 정보
  stripped?: boolean;
}

export interface GeoLocation {
  latitude: number;
  longitude: number;
  accuracy?: number;
  name?: string;
}

// ============================================================================
// Posts
// ============================================================================

/**
 * 통합 포스트
 */
export interface UniversalPost {
  id: PostId;
  author: UniversalIdentity;
  content: PostContent;
  targets: PostTarget[];
  platformPosts: PlatformPost[];
  reactions: UnifiedReactions;
  comments: UnifiedComments;
  metadata: PostMetadata;
}

/**
 * 포스트 내용
 */
export interface PostContent {
  text?: string;
  media?: MediaRef[];
  poll?: Poll;
  location?: PostLocation;
  mentions?: Mention[];
  hashtags?: string[];
  links?: Link[];
}

export interface Poll {
  question: string;
  options: PollOption[];
  duration: number; // hours
  multipleChoice: boolean;
}

export interface PollOption {
  id: string;
  text: string;
  votes?: number;
}

export interface PostLocation {
  name: string;
  coordinates?: GeoLocation;
  placeId?: string;
}

export interface Mention {
  identity: UniversalIdentity;
  startIndex: number;
  endIndex: number;
}

export interface Link {
  url: string;
  title?: string;
  description?: string;
  image?: string;
  startIndex?: number;
  endIndex?: number;
}

/**
 * 포스트 타겟
 */
export interface PostTarget {
  platform: Platform;
  enabled: boolean;
  optimization?: PlatformOptimization;
}

export interface PlatformOptimization {
  // Instagram
  filters?: string;
  hashtags?: boolean;
  // Twitter
  characterLimit?: boolean;
  // TikTok
  sounds?: string;
  // 공통
  schedule?: Timestamp;
}

/**
 * 플랫폼별 포스트
 */
export interface PlatformPost {
  platform: Platform;
  platformPostId: PlatformPostId;
  url: string;
  postedAt: Timestamp;
  status: 'pending' | 'posted' | 'failed' | 'deleted';
  error?: string;
}

/**
 * 포스트 메타데이터
 */
export interface PostMetadata {
  createdAt: Timestamp;
  updatedAt: Timestamp;
  visibility: PrivacyLevel;
  scheduled?: Timestamp;
  expiresAt?: Timestamp; // for ephemeral posts
  sensitive: boolean;
  spoiler?: string;
  language?: string;
  replyTo?: PostId;
  quotedPost?: PostId;
}

// ============================================================================
// Reactions & Comments
// ============================================================================

/**
 * 통합 반응
 */
export interface UnifiedReactions {
  total: number;
  byType: ReactionCounts;
  byPlatform: Partial<Record<Platform, ReactionCounts>>;
}

export interface ReactionCounts {
  like: number;
  love: number;
  haha: number;
  wow: number;
  sad: number;
  angry: number;
  repost: number;
  quote: number;
  bookmark: number;
}

/**
 * 통합 댓글
 */
export interface UnifiedComments {
  total: number;
  items: Comment[];
  hasMore: boolean;
}

export interface Comment {
  id: string;
  platform: Platform;
  author: UniversalIdentity | PlatformUser;
  content: string;
  createdAt: Timestamp;
  reactions: ReactionCounts;
  replies?: Comment[];
}

export interface PlatformUser {
  platform: Platform;
  platformUserId: PlatformUserId;
  handle: string;
  displayName: string;
  avatar?: string;
}

// ============================================================================
// Feed
// ============================================================================

/**
 * 통합 피드
 */
export interface UnifiedFeed {
  posts: UniversalPost[];
  sources: FeedSource[];
  filters: FeedFilters;
  sorting: FeedSorting;
  cursor?: string;
  hasMore: boolean;
}

export interface FeedSource {
  platform: Platform;
  enabled: boolean;
  lastFetched: Timestamp;
  error?: string;
}

export interface FeedFilters {
  platforms?: Platform[];
  contentTypes?: ('text' | 'image' | 'video' | 'link')[];
  authors?: WIAId[];
  hashtags?: string[];
  dateRange?: { start: Timestamp; end: Timestamp };
  hideReposts?: boolean;
  hideSeen?: boolean;
}

export type FeedSorting = 'chronological' | 'reverse_chronological' | 'algorithmic' | 'engagement';

// ============================================================================
// Social Graph
// ============================================================================

/**
 * 관계
 */
export interface Relationship {
  following: boolean;
  followedBy: boolean;
  blocked: boolean;
  blockedBy: boolean;
  muted: boolean;
  closeFreind: boolean;
}

/**
 * 연결 (팔로우 관계)
 */
export interface Connection {
  identity: UniversalIdentity;
  relationship: Relationship;
  platforms: Platform[]; // 연결된 플랫폼들
  connectedAt: Timestamp;
  interactionScore: number; // 0-100
}

/**
 * 친구 추천
 */
export interface FriendSuggestion {
  identity: UniversalIdentity;
  reason: SuggestionReason;
  mutualConnections: number;
  score: number;
}

export type SuggestionReason =
  | 'mutual_friends'
  | 'same_platform'
  | 'similar_interests'
  | 'location'
  | 'imported_contacts';

// ============================================================================
// Messaging
// ============================================================================

/**
 * 대화
 */
export interface Conversation {
  id: ConversationId;
  type: 'direct' | 'group';
  participants: (UniversalIdentity | PlatformUser)[];
  platform: Platform;
  lastMessage: Message;
  unreadCount: number;
  muted: boolean;
  archived: boolean;
  updatedAt: Timestamp;
}

/**
 * 메시지
 */
export interface Message {
  id: string;
  conversationId: ConversationId;
  sender: UniversalIdentity | PlatformUser;
  content: MessageContent;
  sentAt: Timestamp;
  readAt?: Timestamp;
  status: 'sending' | 'sent' | 'delivered' | 'read' | 'failed';
  encrypted: boolean;
}

export interface MessageContent {
  text?: string;
  media?: MediaRef[];
  sticker?: Sticker;
  voiceNote?: MediaRef;
  sharedPost?: PostId;
  location?: GeoLocation;
}

export interface Sticker {
  id: string;
  url: string;
  pack: string;
}

// ============================================================================
// Data Portability
// ============================================================================

/**
 * 내보내기 패키지
 */
export interface ExportPackage {
  format: 'wia-social-v1';
  exportedAt: Timestamp;
  identity: UniversalIdentity;
  posts: UniversalPost[];
  connections: Connection[];
  messages: Conversation[];
  media: MediaArchive;
  metadata: ExportMetadata;
}

export interface MediaArchive {
  totalSize: number;
  items: MediaRef[];
  downloadUrl?: string;
}

export interface ExportMetadata {
  platforms: Platform[];
  dateRange: { start: Timestamp; end: Timestamp };
  postCount: number;
  connectionCount: number;
  messageCount: number;
  mediaCount: number;
}

/**
 * 가져오기 결과
 */
export interface ImportResult {
  success: boolean;
  imported: {
    posts: number;
    connections: number;
    messages: number;
    media: number;
  };
  skipped: {
    posts: number;
    connections: number;
    messages: number;
    media: number;
  };
  errors: ImportError[];
}

export interface ImportError {
  type: string;
  item: string;
  message: string;
}

// ============================================================================
// Privacy Scan (이모 연동)
// ============================================================================

/**
 * 프라이버시 스캔 결과
 */
export interface PrivacyScanResult {
  safe: boolean;
  risks: PrivacyRisk[];
  recommendation: 'proceed' | 'review' | 'block';
}

export interface PrivacyRisk {
  type: PrivacyRiskType;
  severity: 'low' | 'medium' | 'high';
  description: string;
  recommendation: string;
  autoFix?: () => Promise<void>;
}

export type PrivacyRiskType =
  | 'location_exposure'
  | 'face_detection'
  | 'sensitive_info'
  | 'metadata_leak'
  | 'cross_reference'
  | 'public_to_private';

// ============================================================================
// Configuration
// ============================================================================

/**
 * WIA-SOCIAL 설정
 */
export interface WIASocialConfig {
  identity?: UniversalIdentity;
  platforms: Platform[];
  privacy: Partial<PrivacySettings>;
  autoSync: boolean;
  syncInterval: number; // minutes
  enableAirShield: boolean;
  enableAnalytics: boolean;
}

// ============================================================================
// Events
// ============================================================================

/**
 * 이벤트 타입
 */
export type SocialEventType =
  | 'new_post'
  | 'new_comment'
  | 'new_reaction'
  | 'new_follower'
  | 'new_message'
  | 'mention'
  | 'platform_connected'
  | 'platform_disconnected'
  | 'privacy_alert';

export interface SocialEvent {
  type: SocialEventType;
  timestamp: Timestamp;
  platform?: Platform;
  data: unknown;
}

export type EventCallback = (event: SocialEvent) => void;
export type PostCallback = (post: UniversalPost) => void;
export type Unsubscribe = () => void;

// ============================================================================
// API Options
// ============================================================================

export interface PaginationOptions {
  limit?: number;
  cursor?: string;
}

export interface FeedOptions extends PaginationOptions {
  filters?: FeedFilters;
  sorting?: FeedSorting;
  includeReposts?: boolean;
}

export interface SearchOptions extends PaginationOptions {
  type?: 'all' | 'users' | 'posts' | 'hashtags';
  platforms?: Platform[];
}

// ============================================================================
// Results
// ============================================================================

export interface CrossPostResult {
  success: boolean;
  results: {
    platform: Platform;
    success: boolean;
    postId?: PlatformPostId;
    url?: string;
    error?: string;
  }[];
}

export interface SyncResult {
  success: boolean;
  synced: {
    posts: number;
    connections: number;
    messages: number;
  };
  errors: string[];
  duration: number;
}
