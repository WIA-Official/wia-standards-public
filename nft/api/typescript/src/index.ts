/**
 * WIA-FIN-009 NFT Standard - TypeScript SDK
 * @module @wia/nft-sdk
 */

import { ethers } from 'ethers';
import axios, { AxiosInstance } from 'axios';
import {
  NFT,
  NFTMetadata,
  NFTClientConfig,
  MintOptions,
  TransferOptions,
  RoyaltyInfo,
  CollectionStats,
  MarketplaceListing,
  TransactionReceipt,
  IPFSUploadResult,
  PaginationParams,
  PaginatedResponse
} from './types';

export * from './types';

/**
 * Main NFT client for interacting with NFT contracts and APIs
 */
export class NFTClient {
  private provider: ethers.Provider;
  private signer?: ethers.Signer;
  private apiClient: AxiosInstance;
  private config: NFTClientConfig;

  /**
   * Create a new NFT client
   * @param config - Client configuration
   */
  constructor(config: NFTClientConfig) {
    this.config = config;

    // Initialize provider
    if (config.rpcUrl) {
      this.provider = new ethers.JsonRpcProvider(config.rpcUrl);
    } else {
      this.provider = ethers.getDefaultProvider(config.network);
    }

    // Initialize API client
    this.apiClient = axios.create({
      baseURL: 'https://api.wia.org/v1',
      headers: config.apiKey ? {
        'Authorization': `Bearer ${config.apiKey}`,
        'X-API-Key': config.apiKey
      } : {}
    });

    if (config.debug) {
      this.apiClient.interceptors.request.use(request => {
        console.log('[NFTClient] Request:', request);
        return request;
      });
    }
  }

  /**
   * Connect a signer for transaction signing
   * @param signer - Ethers signer instance
   */
  connectSigner(signer: ethers.Signer): void {
    this.signer = signer;
  }

  /**
   * Get NFT information
   * @param contractAddress - NFT contract address
   * @param tokenId - Token ID
   * @returns NFT data
   */
  async getNFT(contractAddress: string, tokenId: string): Promise<NFT> {
    const contract = new ethers.Contract(
      contractAddress,
      ['function ownerOf(uint256) view returns (address)',
       'function tokenURI(uint256) view returns (string)',
       'function supportsInterface(bytes4) view returns (bool)'],
      this.provider
    );

    const [owner, tokenURI] = await Promise.all([
      contract.ownerOf(tokenId),
      contract.tokenURI(tokenId)
    ]);

    // Detect standard
    const is721 = await contract.supportsInterface('0x80ac58cd');
    const is1155 = await contract.supportsInterface('0xd9b67a26');

    // Fetch metadata
    let metadata: NFTMetadata | undefined;
    try {
      metadata = await this.fetchMetadata(tokenURI);
    } catch (error) {
      console.error('Failed to fetch metadata:', error);
    }

    return {
      contractAddress,
      tokenId,
      owner,
      tokenURI,
      metadata,
      standard: is721 ? 'ERC-721' : is1155 ? 'ERC-1155' : 'ERC-721',
      network: this.config.network
    };
  }

  /**
   * Fetch metadata from URI
   * @param uri - Metadata URI (IPFS, HTTP, or data URI)
   * @returns Parsed metadata
   */
  async fetchMetadata(uri: string): Promise<NFTMetadata> {
    let url = uri;

    // Convert IPFS URIs to HTTP gateway
    if (uri.startsWith('ipfs://')) {
      const cid = uri.replace('ipfs://', '');
      const gateway = this.config.ipfsGateway || 'https://ipfs.io/ipfs';
      url = `${gateway}/${cid}`;
    }

    // Handle data URIs
    if (uri.startsWith('data:application/json')) {
      const base64Data = uri.split(',')[1];
      const jsonString = Buffer.from(base64Data, 'base64').toString('utf-8');
      return JSON.parse(jsonString);
    }

    const response = await axios.get(url);
    return response.data;
  }

  /**
   * Mint a new NFT
   * @param options - Minting options
   * @returns Transaction receipt
   */
  async mint(options: MintOptions): Promise<TransactionReceipt> {
    if (!this.signer) {
      throw new Error('Signer not connected. Call connectSigner() first.');
    }

    const response = await this.apiClient.post('/nft/mint', {
      recipient: options.recipient,
      metadata: options.metadata,
      royaltyPercentage: options.royaltyPercentage,
      network: this.config.network
    });

    return response.data;
  }

  /**
   * Transfer an NFT
   * @param options - Transfer options
   * @returns Transaction receipt
   */
  async transfer(options: TransferOptions): Promise<TransactionReceipt> {
    if (!this.signer) {
      throw new Error('Signer not connected');
    }

    const contract = new ethers.Contract(
      options.contractAddress,
      ['function transferFrom(address,address,uint256)'],
      this.signer
    );

    const tx = await contract.transferFrom(
      options.from,
      options.to,
      options.tokenId
    );

    const receipt = await tx.wait();

    return {
      transactionHash: receipt.hash,
      blockNumber: receipt.blockNumber,
      gasUsed: receipt.gasUsed.toString(),
      status: receipt.status,
      events: receipt.logs
    };
  }

  /**
   * Get royalty information for a token
   * @param contractAddress - Contract address
   * @param tokenId - Token ID
   * @param salePrice - Sale price in wei
   * @returns Royalty information
   */
  async getRoyaltyInfo(
    contractAddress: string,
    tokenId: string,
    salePrice: string
  ): Promise<RoyaltyInfo> {
    const contract = new ethers.Contract(
      contractAddress,
      ['function royaltyInfo(uint256,uint256) view returns (address,uint256)'],
      this.provider
    );

    const [receiver, royaltyAmount] = await contract.royaltyInfo(
      tokenId,
      salePrice
    );

    const royaltyPercentage = (
      (Number(royaltyAmount) / Number(salePrice)) * 100
    ).toFixed(2);

    return {
      receiver,
      royaltyAmount: royaltyAmount.toString(),
      royaltyPercentage: parseFloat(royaltyPercentage)
    };
  }

  /**
   * Get NFTs owned by an address
   * @param ownerAddress - Owner wallet address
   * @param params - Pagination parameters
   * @returns List of owned NFTs
   */
  async getNFTsByOwner(
    ownerAddress: string,
    params?: PaginationParams
  ): Promise<PaginatedResponse<NFT>> {
    const response = await this.apiClient.get('/nft/owner', {
      params: {
        address: ownerAddress,
        network: this.config.network,
        ...params
      }
    });

    return response.data;
  }

  /**
   * Get collection statistics
   * @param contractAddress - Collection contract address
   * @returns Collection stats
   */
  async getCollectionStats(contractAddress: string): Promise<CollectionStats> {
    const response = await this.apiClient.get(
      `/analytics/collection/${contractAddress}`,
      { params: { network: this.config.network } }
    );

    return response.data;
  }

  /**
   * List NFT for sale on marketplace
   * @param contractAddress - NFT contract address
   * @param tokenId - Token ID
   * @param price - Listing price in ETH
   * @param duration - Listing duration in seconds
   * @returns Listing information
   */
  async listForSale(
    contractAddress: string,
    tokenId: string,
    price: string,
    duration: number
  ): Promise<MarketplaceListing> {
    const response = await this.apiClient.post('/marketplace/list', {
      contractAddress,
      tokenId,
      price,
      currency: 'ETH',
      duration,
      network: this.config.network
    });

    return response.data;
  }

  /**
   * Upload metadata to IPFS
   * @param metadata - NFT metadata object
   * @returns IPFS upload result
   */
  async uploadToIPFS(metadata: NFTMetadata): Promise<IPFSUploadResult> {
    const response = await this.apiClient.post('/ipfs/upload', metadata);
    return response.data;
  }

  /**
   * Upload file to IPFS
   * @param file - File buffer or Blob
   * @param filename - File name
   * @returns IPFS upload result
   */
  async uploadFileToIPFS(
    file: Buffer | Blob,
    filename: string
  ): Promise<IPFSUploadResult> {
    const formData = new FormData();
    formData.append('file', file, filename);

    const response = await this.apiClient.post('/ipfs/upload', formData, {
      headers: { 'Content-Type': 'multipart/form-data' }
    });

    return response.data;
  }

  /**
   * Get marketplace listings
   * @param params - Pagination and filter parameters
   * @returns Paginated listings
   */
  async getListings(
    params?: PaginationParams & { contractAddress?: string }
  ): Promise<PaginatedResponse<MarketplaceListing>> {
    const response = await this.apiClient.get('/marketplace/listings', {
      params: { network: this.config.network, ...params }
    });

    return response.data;
  }

  /**
   * Estimate gas for minting
   * @param options - Minting options
   * @returns Gas estimate
   */
  async estimateMintGas(options: MintOptions): Promise<string> {
    if (!this.signer) {
      throw new Error('Signer not connected');
    }

    const response = await this.apiClient.post('/gas/estimate', {
      operation: 'mint',
      params: options,
      network: this.config.network
    });

    return response.data.gasEstimate;
  }
}

/**
 * Create a new NFT client instance
 * @param config - Client configuration
 * @returns NFT client instance
 */
export function createNFTClient(config: NFTClientConfig): NFTClient {
  return new NFTClient(config);
}

/**
 * Utility function to convert ETH to wei
 * @param eth - Amount in ETH
 * @returns Amount in wei
 */
export function toWei(eth: string): string {
  return ethers.parseEther(eth).toString();
}

/**
 * Utility function to convert wei to ETH
 * @param wei - Amount in wei
 * @returns Amount in ETH
 */
export function fromWei(wei: string): string {
  return ethers.formatEther(wei);
}

/**
 * Validate Ethereum address
 * @param address - Address to validate
 * @returns True if valid
 */
export function isValidAddress(address: string): boolean {
  return ethers.isAddress(address);
}

/**
 * Generate IPFS URI from CID
 * @param cid - IPFS content identifier
 * @returns IPFS URI
 */
export function ipfsUri(cid: string): string {
  return `ipfs://${cid}`;
}

// Export default client creator
export default createNFTClient;
