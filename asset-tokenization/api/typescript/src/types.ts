/**
 * WIA-FIN-008 Asset Tokenization Standard
 * TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 */

export type AssetClass =
  | 'REAL_ESTATE'
  | 'ART'
  | 'COMMODITY'
  | 'PRIVATE_EQUITY'
  | 'DEBT_INSTRUMENT'
  | 'INTELLECTUAL_PROPERTY'
  | 'FUND'
  | 'OTHER';

export type RegulatoryFramework =
  | 'REG_D_506B'
  | 'REG_D_506C'
  | 'REG_A_TIER1'
  | 'REG_A_TIER2'
  | 'REG_S'
  | 'REG_CF'
  | 'FULLY_REGISTERED'
  | 'EU_MiFID_II'
  | 'UK_FCA'
  | 'OTHER';

export type BlockchainNetwork =
  | 'ethereum-mainnet'
  | 'polygon-mainnet'
  | 'bsc-mainnet'
  | 'avalanche-mainnet'
  | 'arbitrum-mainnet'
  | 'optimism-mainnet'
  | 'sepolia'
  | 'mumbai';

export type TokenStandard =
  | 'ERC-1400'
  | 'ERC-3643'
  | 'ERC-20'
  | 'ERC-1155'
  | 'CUSTOM';

export interface TokenMetadata {
  '@context': string;
  '@type': 'WIA-FIN-008/AssetToken';
  version: string;
  tokenId: string;
  name: string;
  symbol: string;
  assetClass: AssetClass;
  subClass?: string;
  totalSupply: number;
  decimals: number;
  issuer: IssuerInfo;
  issuanceDate: string;
  maturityDate?: string | null;
  regulatory: RegulatoryInfo;
  blockchain: BlockchainInfo;
}

export interface IssuerInfo {
  legalName: string;
  jurisdiction: string;
  lei?: string;
  website?: string;
  contact?: {
    email?: string;
    phone?: string;
  };
}

export interface RegulatoryInfo {
  framework: RegulatoryFramework;
  filingNumber?: string;
  jurisdictions?: string[];
  investorRestrictions?: {
    accreditedOnly?: boolean;
    minInvestment?: {
      amount: number;
      currency: string;
    };
    maxInvestors?: number;
    lockupPeriod?: string;
  };
}

export interface BlockchainInfo {
  network: BlockchainNetwork;
  contractAddress: string;
  standard: TokenStandard;
  deployedAt?: string;
  txHash?: string;
}

export interface RealEstateAsset {
  propertyId: string;
  address: {
    street: string;
    city: string;
    state: string;
    postalCode: string;
    country: string;
    coordinates?: {
      latitude: number;
      longitude: number;
    };
  };
  propertyType: string;
  buildingClass?: string;
  yearBuilt?: number;
  squareFeet: number;
  floors?: number;
  tenancy?: TenancyInfo;
  valuation: ValuationInfo;
  financials?: PropertyFinancials;
}

export interface TenancyInfo {
  occupancyRate: number;
  majorTenants?: Array<{
    name: string;
    squareFeet: number;
    leaseExpiry: string;
    annualRent: number;
  }>;
  totalAnnualRent: number;
  averageLeaseLength?: string;
}

export interface ValuationInfo {
  purchasePrice?: number;
  purchaseDate?: string;
  currentValue: number;
  valuationDate: string;
  valuationMethod: string;
  capRate?: number;
  appraisalFirm?: string;
}

export interface PropertyFinancials {
  mortgage?: {
    principal: number;
    interestRate: number;
    maturity: string;
    monthlyPayment: number;
  };
  operatingExpenses?: {
    propertyTax: number;
    insurance: number;
    maintenance: number;
    utilities: number;
    management: number;
    total: number;
  };
  noi?: number;
  cashFlow?: number;
}

export interface ArtAsset {
  artworkId: string;
  title: string;
  artist: {
    name: string;
    birthYear?: number;
    deathYear?: number;
    nationality?: string;
  };
  medium: string;
  dimensions: {
    height: number;
    width: number;
    depth?: number;
    unit: 'cm' | 'in';
  };
  created: number;
  provenance: ProvenanceRecord[];
  authentication: AuthenticationInfo;
  condition: ConditionInfo;
  custody: CustodyInfo;
  valuation: ValuationInfo;
}

export interface ProvenanceRecord {
  owner: string;
  period?: string;
  date?: string;
  acquisitionMethod: string;
  salePrice?: number;
  currency?: string;
}

export interface AuthenticationInfo {
  catalogueRaisonne?: string;
  expertOpinions?: Array<{
    expert: string;
    institution: string;
    certificationDate: string;
    authenticity: 'CONFIRMED' | 'DISPUTED' | 'UNCERTAIN';
  }>;
  technicalAnalysis?: {
    xRayCompleted?: boolean;
    pigmentAnalysis?: string;
    carbonDating?: string;
  };
}

export interface ConditionInfo {
  rating: 'EXCELLENT' | 'VERY_GOOD' | 'GOOD' | 'FAIR' | 'POOR';
  lastInspection: string;
  conservationHistory?: Array<{
    date: string;
    work: string;
    conservator: string;
  }>;
}

export interface CustodyInfo {
  location: string;
  insurance: {
    provider: string;
    value: number;
    policyNumber: string;
  };
  security?: string;
}

export interface CommodityAsset {
  commodityType: string;
  grade: string;
  form: string;
  quantity: {
    value: number;
    unit: string;
    metric?: number;
  };
  purity: number;
  serialNumbers?: string[];
  assay: AssayInfo;
  storage: StorageInfo;
  valuation: CommodityValuation;
  tokenization: TokenizationInfo;
}

export interface AssayInfo {
  certifier: string;
  certificationDate: string;
  certificateNumber: string;
}

export interface StorageInfo {
  custodian: string;
  vaultLocation: string;
  vaultAddress: string;
  insurance: {
    provider: string;
    coverage: number;
    policyNumber: string;
  };
  auditFrequency: string;
  lastAudit: string;
  auditor: string;
}

export interface CommodityValuation {
  spotPrice: number;
  priceUnit: string;
  totalValue: number;
  pricingSource: string;
  lastUpdated: string;
}

export interface TokenizationInfo {
  tokensPerOunce?: number;
  totalTokens: number;
  tokenValue: number;
  redemptionRights?: {
    enabled: boolean;
    minimumRedemption: number;
    redemptionFee: number;
    deliveryOptions: string[];
  };
}

export interface InvestorRecord {
  holderId: string;
  walletAddress: string;
  identityVerification: KYCInfo;
  accreditation: AccreditationInfo;
  jurisdiction: string;
  investorType: 'INDIVIDUAL' | 'INSTITUTIONAL';
  holdings: HoldingsInfo;
  restrictions: TransferRestrictions;
  distributions?: DistributionHistory;
}

export interface KYCInfo {
  kycProvider: string;
  kycStatus: 'VERIFIED' | 'PENDING' | 'REJECTED' | 'EXPIRED';
  kycDate: string;
  kycExpiry: string;
  amlStatus: 'CLEAR' | 'FLAGGED' | 'PENDING';
  amlDate: string;
}

export interface AccreditationInfo {
  status: 'ACCREDITED' | 'NOT_ACCREDITED' | 'PENDING' | 'EXPIRED';
  method?: 'INCOME' | 'NET_WORTH' | 'PROFESSIONAL_CERTIFICATION';
  verifiedBy?: string;
  verificationDate?: string;
  expiryDate?: string;
}

export interface HoldingsInfo {
  totalTokens: number;
  percentOwnership: number;
  acquisitions: Array<{
    date: string;
    tokens: number;
    pricePerToken: number;
    totalCost: number;
    currency: string;
    transactionHash: string;
  }>;
  costBasis: number;
  currentValue: number;
  unrealizedGain: number;
}

export interface TransferRestrictions {
  transferRestricted: boolean;
  lockupExpiry?: string;
  maxOwnershipPercent?: number;
  allowedJurisdictions?: string[];
}

export interface DistributionHistory {
  totalReceived: number;
  distributions: Array<{
    date: string;
    type: 'DIVIDEND' | 'RENTAL_INCOME' | 'INTEREST' | 'CAPITAL_GAIN';
    amount: number;
    currency: string;
    txHash: string;
  }>;
}

export interface Transfer {
  transferId: string;
  tokenId: string;
  from: string;
  to: string;
  amount: number;
  status: 'PENDING' | 'IN_PROGRESS' | 'COMPLETED' | 'FAILED' | 'REJECTED';
  txHash?: string;
  completedAt?: string;
  gasUsed?: number;
  gasCost?: string;
}

export interface ComplianceCheck {
  allowed: boolean;
  checks: Array<{
    rule: string;
    status: 'PASSED' | 'FAILED' | 'PENDING';
    message: string;
  }>;
  estimatedGas?: number;
  estimatedFee?: string;
}

export interface Distribution {
  distributionId: string;
  tokenId: string;
  type: 'DIVIDEND' | 'RENTAL_INCOME' | 'INTEREST' | 'CAPITAL_GAIN';
  totalAmount: number;
  currency: string;
  recordDate: string;
  paymentDate: string;
  description?: string;
  status: 'SCHEDULED' | 'IN_PROGRESS' | 'COMPLETED' | 'FAILED';
  eligibleTokens?: number;
  amountPerToken?: number;
  totalRecipients?: number;
}

export interface APIConfig {
  apiKey: string;
  baseUrl?: string;
  environment?: 'production' | 'staging' | 'development';
  network?: BlockchainNetwork;
  timeout?: number;
}

export interface APIResponse<T> {
  data: T;
  success: boolean;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  requestId?: string;
}
