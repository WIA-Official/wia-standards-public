/**
 * WIA-FIN-009 NFT Standard - TypeScript Type Definitions
 * @module @wia/nft-sdk
 */

/**
 * NFT Metadata following OpenSea standard
 */
export interface NFTMetadata {
  /** Token name */
  name: string;
  /** Detailed description supporting markdown */
  description: string;
  /** Image URI (IPFS, HTTP, or data URI) */
  image: string;
  /** External URL to view item */
  external_url?: string;
  /** Animation or multimedia content */
  animation_url?: string;
  /** Background color (6-char hex without #) */
  background_color?: string;
  /** Array of token attributes/traits */
  attributes?: NFTAttribute[];
  /** Additional custom properties */
  properties?: Record<string, any>;
}

/**
 * NFT attribute/trait definition
 */
export interface NFTAttribute {
  /** Trait category/type */
  trait_type: string;
  /** Trait value */
  value: string | number;
  /** Display type for special formatting */
  display_type?: 'boost_number' | 'boost_percentage' | 'number' | 'date';
  /** Maximum value for numeric traits */
  max_value?: number;
  /** Number of items in collection with this trait */
  trait_count?: number;
}

/**
 * NFT token information
 */
export interface NFT {
  /** Contract address */
  contractAddress: string;
  /** Token ID */
  tokenId: string;
  /** Current owner address */
  owner: string;
  /** Token URI pointing to metadata */
  tokenURI: string;
  /** Parsed metadata */
  metadata?: NFTMetadata;
  /** Token standard (ERC-721 or ERC-1155) */
  standard: 'ERC-721' | 'ERC-1155';
  /** Blockchain network */
  network: Network;
}

/**
 * Collection metadata
 */
export interface CollectionMetadata {
  /** Collection name */
  name: string;
  /** Collection description */
  description: string;
  /** Collection image */
  image: string;
  /** External link to project website */
  external_link?: string;
  /** Royalty in basis points (e.g., 750 = 7.5%) */
  seller_fee_basis_points: number;
  /** Royalty recipient address */
  fee_recipient: string;
  /** Additional properties */
  properties?: {
    category?: string;
    total_supply?: number;
    mint_date?: string;
    [key: string]: any;
  };
}

/**
 * Royalty information (ERC-2981)
 */
export interface RoyaltyInfo {
  /** Royalty receiver address */
  receiver: string;
  /** Royalty amount in wei */
  royaltyAmount: string;
  /** Royalty percentage (0-100) */
  royaltyPercentage: number;
}

/**
 * Minting options
 */
export interface MintOptions {
  /** Recipient address */
  recipient: string;
  /** Token metadata or URI */
  metadata: NFTMetadata | string;
  /** Royalty percentage (optional) */
  royaltyPercentage?: number;
  /** Payment amount in wei (optional) */
  value?: string;
}

/**
 * Transfer options
 */
export interface TransferOptions {
  /** Sender address */
  from: string;
  /** Recipient address */
  to: string;
  /** Token ID to transfer */
  tokenId: string;
  /** Contract address */
  contractAddress: string;
}

/**
 * Marketplace listing
 */
export interface MarketplaceListing {
  /** Unique listing ID */
  listingId: string;
  /** Seller address */
  seller: string;
  /** Contract address */
  contractAddress: string;
  /** Token ID */
  tokenId: string;
  /** Listing price in wei */
  price: string;
  /** Currency (ETH, WETH, etc.) */
  currency: string;
  /** Listing expiration timestamp */
  expiresAt: number;
  /** Listing status */
  status: 'active' | 'sold' | 'cancelled' | 'expired';
  /** Marketplace platform */
  marketplace: 'OpenSea' | 'Rarible' | 'LooksRare' | 'Blur' | 'Custom';
}

/**
 * Transaction receipt
 */
export interface TransactionReceipt {
  /** Transaction hash */
  transactionHash: string;
  /** Block number */
  blockNumber: number;
  /** Gas used */
  gasUsed: string;
  /** Status (1 = success, 0 = failure) */
  status: number;
  /** Events emitted */
  events?: any[];
}

/**
 * Supported blockchain networks
 */
export type Network =
  | 'ethereum'
  | 'polygon'
  | 'arbitrum'
  | 'optimism'
  | 'base'
  | 'goerli'
  | 'sepolia'
  | 'mumbai';

/**
 * SDK configuration
 */
export interface NFTClientConfig {
  /** API key for authenticated requests */
  apiKey?: string;
  /** Blockchain network */
  network: Network;
  /** RPC endpoint URL */
  rpcUrl?: string;
  /** IPFS gateway URL */
  ipfsGateway?: string;
  /** Enable debug logging */
  debug?: boolean;
}

/**
 * IPFS upload result
 */
export interface IPFSUploadResult {
  /** IPFS content identifier */
  cid: string;
  /** IPFS URI */
  url: string;
  /** File size in bytes */
  size: number;
  /** Whether content is pinned */
  pinned: boolean;
}

/**
 * Collection statistics
 */
export interface CollectionStats {
  /** Total supply */
  totalSupply: number;
  /** Number of unique owners */
  uniqueOwners: number;
  /** Floor price in ETH */
  floorPrice: string;
  /** Total volume traded in ETH */
  volumeTraded: string;
  /** Total number of sales */
  totalSales: number;
  /** 24h volume change percentage */
  volumeChange24h?: number;
  /** 24h floor price change percentage */
  floorPriceChange24h?: number;
}

/**
 * Error response
 */
export interface APIError {
  /** Error code */
  code: string;
  /** Error message */
  message: string;
  /** Additional error details */
  details?: Record<string, any>;
}

/**
 * Pagination parameters
 */
export interface PaginationParams {
  /** Number of results per page */
  limit?: number;
  /** Cursor for pagination */
  cursor?: string;
  /** Sort order */
  sort?: 'asc' | 'desc';
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  /** Results */
  data: T[];
  /** Next page cursor */
  nextCursor?: string;
  /** Total count (if available) */
  total?: number;
}

/**
 * Event filter options
 */
export interface EventFilter {
  /** Contract address to filter */
  contractAddress?: string;
  /** From block number */
  fromBlock?: number;
  /** To block number */
  toBlock?: number;
  /** Event topics */
  topics?: string[];
}

/**
 * Auction data
 */
export interface Auction {
  /** Auction ID */
  auctionId: string;
  /** Seller address */
  seller: string;
  /** Contract address */
  contractAddress: string;
  /** Token ID */
  tokenId: string;
  /** Starting price */
  startingPrice: string;
  /** Reserve price */
  reservePrice?: string;
  /** Auction end time */
  endTime: number;
  /** Current highest bid */
  highestBid?: string;
  /** Current highest bidder */
  highestBidder?: string;
  /** Auction status */
  status: 'active' | 'ended' | 'cancelled';
}

/**
 * Gas estimation result
 */
export interface GasEstimate {
  /** Estimated gas limit */
  gasLimit: string;
  /** Suggested gas price in wei */
  gasPrice: string;
  /** Maximum priority fee per gas */
  maxPriorityFeePerGas?: string;
  /** Maximum fee per gas */
  maxFeePerGas?: string;
  /** Estimated total cost in ETH */
  totalCostEth: string;
}
