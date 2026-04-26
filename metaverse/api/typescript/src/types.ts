/**
 * WIA-EDU-018: Metaverse Standard - TypeScript Type Definitions
 *
 * @module @wia/metaverse/types
 * @author WIA (World Certification Industry Association)
 * @license MIT
 */

// ============================================================================
// Core Types
// ============================================================================

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Transform {
  position: Vector3;
  rotation: Quaternion;
  scale: Vector3;
}

// ============================================================================
// World Types
// ============================================================================

export interface WorldConfig {
  worldId: string;
  name: string;
  description?: string;
  type: WorldType;
  region?: string;
  maxUsers: number;
  features?: WorldFeatures;
}

export type WorldType =
  | 'educational'
  | 'social'
  | 'gaming'
  | 'commerce'
  | 'creative'
  | 'enterprise';

export interface WorldFeatures {
  voiceChat?: boolean;
  spatialAudio?: boolean;
  textChat?: boolean;
  teleportation?: boolean;
  physics?: boolean;
  npcSupport?: boolean;
}

export interface SpaceConfig {
  name: string;
  type: string;
  size: Vector3;
  terrain?: string;
  skybox?: string;
  physics?: PhysicsConfig;
  features?: WorldFeatures;
}

export interface PhysicsConfig {
  gravity: number;
  collisions: boolean;
}

// ============================================================================
// Portal Types
// ============================================================================

export interface Portal {
  portalId: string;
  sourceWorld: string;
  destinationWorld: string;
  position: Vector3;
  appearance?: string;
  bidirectional: boolean;
  authentication?: PortalAuthentication;
  preserveState?: PortalPreserveState;
}

export interface PortalAuthentication {
  required: boolean;
  method?: 'universal-identity' | 'world-specific' | 'none';
}

export interface PortalPreserveState {
  avatar?: boolean;
  inventory?: boolean;
  progress?: boolean;
}

// ============================================================================
// Avatar Types
// ============================================================================

export interface AvatarConfig {
  format: AvatarFormat;
  allowCustomization: boolean;
  bodyTypes?: string[];
  accessories?: AccessoryConfig;
  animations?: AnimationConfig;
}

export type AvatarFormat = 'VRM' | 'glTF' | 'USD' | 'FBX';

export interface AccessoryConfig {
  enabled: boolean;
  nftSupport?: boolean;
  crossPlatform?: boolean;
}

export interface AnimationConfig {
  standard: string[];
  custom?: boolean;
}

export interface Avatar {
  avatarId: string;
  userId: string;
  format: AvatarFormat;
  modelUrl: string;
  thumbnailUrl?: string;
  customization?: AvatarCustomization;
  wearables?: Wearable[];
  animations?: Animations;
}

export interface AvatarCustomization {
  bodyType?: string;
  height?: number;
  proportions?: string;
  skinTone?: string;
}

export interface Wearable {
  slot: WearableSlot;
  itemId: string;
  nftContract?: string;
  tokenId?: string;
}

export type WearableSlot =
  | 'head'
  | 'torso'
  | 'legs'
  | 'feet'
  | 'hands'
  | 'accessory';

export interface Animations {
  standard: string[];
  custom?: string[];
}

export interface AvatarSpawnConfig {
  userId: string;
  avatarUrl?: string;
  position: Vector3;
  rotation: Quaternion;
  metadata?: AvatarMetadata;
}

export interface AvatarMetadata {
  name?: string;
  reputation?: number;
  achievements?: string[];
}

// ============================================================================
// Asset Types
// ============================================================================

export interface AssetManager {
  blockchain: string;
  nftStandard: NFTStandard;
  marketplace?: MarketplaceConfig;
  ownership?: OwnershipConfig;
}

export type NFTStandard = 'ERC-721' | 'ERC-1155';

export interface MarketplaceConfig {
  enabled: boolean;
  royaltyPercentage: number;
  currency: string[];
}

export interface OwnershipConfig {
  transferable: boolean;
  verifiable: boolean;
  crossWorld: boolean;
}

export interface DigitalAsset {
  assetId: string;
  name: string;
  type: AssetType;
  modelUrl?: string;
  rarity?: AssetRarity;
  metadata?: AssetMetadata;
  nft?: NFTConfig;
}

export type AssetType =
  | 'collectible'
  | 'wearable'
  | 'real-estate'
  | 'tool'
  | 'art'
  | 'access-token';

export type AssetRarity =
  | 'common'
  | 'uncommon'
  | 'rare'
  | 'epic'
  | 'legendary';

export interface AssetMetadata {
  creator?: string;
  created?: string;
  description?: string;
  attributes?: Record<string, any>;
}

export interface NFTConfig {
  contract: string;
  tokenId: string;
  blockchain?: string;
}

export interface AssetTransfer {
  assetId: string;
  from: string;
  to: string;
  reason?: string;
  timestamp?: string;
}

// ============================================================================
// Economy Types
// ============================================================================

export interface EconomyConfig {
  currency: CurrencyConfig;
  transactions: TransactionConfig;
  monetization: MonetizationConfig;
}

export interface CurrencyConfig {
  name: string;
  symbol: string;
  exchangeRate?: Record<string, number>;
  blockchain?: boolean;
}

export interface TransactionConfig {
  enabled: boolean;
  feePercentage: number;
  minimumAmount: number;
}

export interface MonetizationConfig {
  creatorRoyalties?: boolean;
  landRental?: boolean;
  eventTickets?: boolean;
}

// ============================================================================
// Interoperability Types
// ============================================================================

export interface InteroperabilityConfig {
  protocols: string[];
  platforms: string[];
  avatarSync: boolean;
  assetSync: boolean;
  socialSync: boolean;
}

// ============================================================================
// Analytics Types
// ============================================================================

export interface Analytics {
  activeUsers: number;
  transactions: number;
  revenue: number;
  avgSessionMinutes: number;
}

export interface AnalyticsQuery {
  period: 'last-24-hours' | 'last-7-days' | 'last-30-days' | 'all-time';
}

// ============================================================================
// Event Types
// ============================================================================

export interface UserJoinedEvent {
  userId: string;
  name: string;
  timestamp: string;
}

export interface AssetTradedEvent {
  asset: string;
  price: number;
  currency: string;
  from: string;
  to: string;
  timestamp: string;
}

export interface PortalUsedEvent {
  userId: string;
  sourceWorld: string;
  destination: string;
  timestamp: string;
}

// ============================================================================
// Error Types
// ============================================================================

export class MetaverseError extends Error {
  constructor(
    message: string,
    public code: string,
    public details?: any
  ) {
    super(message);
    this.name = 'MetaverseError';
  }
}
