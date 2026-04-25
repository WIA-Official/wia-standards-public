# Chapter 3: Data Formats

## Asset Representation and Exchange Schemas

### Introduction

Effective cryonics asset management requires standardized data formats for representing diverse asset types, tracking valuations over time, managing trust structures, and facilitating data exchange between organizations. This chapter defines the core schemas and data structures used in the WIA Cryo-Asset Standard, ensuring interoperability between cryonics organizations, financial institutions, legal service providers, and technology platforms.

---

## 3.1 Core Entity Schemas

### Patient Identity Schema

```typescript
// Core patient identity for asset management
interface CryoPatient {
  // Unique identifiers
  id: string;  // UUID format
  organizationPatientId: string;  // Org-specific ID
  externalIds: ExternalIdentifier[];

  // Legal identity
  legalIdentity: {
    fullLegalName: string;
    previousNames: string[];
    dateOfBirth: Date;
    dateOfLegalDeath: Date | null;
    placeOfBirth: string;
    citizenship: string[];
    taxIdentifiers: TaxIdentifier[];
    governmentIds: GovernmentId[];
  };

  // Preservation status
  preservationStatus: {
    status: PreservationStatus;
    preservationType: 'WHOLE_BODY' | 'NEURO' | 'BRAIN_ONLY';
    preservationDate: Date | null;
    preservationLocation: string;
    organizationId: string;
  };

  // Contact and relationships
  relationships: {
    emergencyContacts: Contact[];
    legalRepresentatives: LegalRepresentative[];
    familyMembers: FamilyMember[];
    authorizedAgents: AuthorizedAgent[];
  };

  // Document references
  documents: {
    membershipAgreement: DocumentReference;
    lastWill: DocumentReference | null;
    advanceDirective: DocumentReference | null;
    trustDocuments: DocumentReference[];
    identityDocuments: DocumentReference[];
  };

  // Metadata
  metadata: {
    createdAt: Date;
    updatedAt: Date;
    dataSource: string;
    verificationLevel: 'UNVERIFIED' | 'BASIC' | 'ENHANCED' | 'FULL';
  };
}

// Supporting types
enum PreservationStatus {
  MEMBER_ACTIVE = 'MEMBER_ACTIVE',
  MEMBER_STANDBY = 'MEMBER_STANDBY',
  MEMBER_SUSPENDED = 'MEMBER_SUSPENDED',
  IN_TRANSPORT = 'IN_TRANSPORT',
  PRESERVED = 'PRESERVED',
  REVIVED = 'REVIVED',
  TERMINATED = 'TERMINATED',
}

interface ExternalIdentifier {
  system: string;  // e.g., 'alcor', 'ci', 'ssn'
  value: string;
  verifiedAt: Date | null;
}

interface TaxIdentifier {
  country: string;  // ISO 3166-1 alpha-2
  type: 'SSN' | 'TIN' | 'EIN' | 'ITIN' | 'OTHER';
  value: string;  // Encrypted
  validFrom: Date;
  validTo: Date | null;
}

interface GovernmentId {
  type: 'PASSPORT' | 'DRIVERS_LICENSE' | 'NATIONAL_ID' | 'OTHER';
  country: string;
  number: string;  // Encrypted
  issueDate: Date;
  expiryDate: Date | null;
  documentHash: string;  // SHA-256 of document image
}

interface LegalRepresentative {
  role: 'POWER_OF_ATTORNEY' | 'HEALTHCARE_PROXY' | 'EXECUTOR' | 'TRUSTEE';
  person: Contact;
  authority: string;
  effectiveDate: Date;
  expirationDate: Date | null;
  documentReference: DocumentReference;
}
```

### Asset Schema

```typescript
// Comprehensive asset representation
interface CryoAsset {
  // Identity
  id: string;  // UUID
  portfolioId: string;
  patientId: string;

  // Classification
  category: AssetCategory;
  subcategory: string;
  assetType: AssetType;

  // Description
  name: string;
  description: string;
  notes: string;

  // Ownership
  ownership: AssetOwnership;

  // Valuation
  acquisitionInfo: AcquisitionInfo;
  currentValuation: Valuation;
  valuationHistory: Valuation[];

  // Location and custody
  location: AssetLocation;
  custody: CustodyInfo;

  // Documentation
  documentation: AssetDocumentation;

  // Blockchain reference
  blockchainRef: BlockchainReference | null;

  // Status and lifecycle
  status: AssetStatus;
  lifecycle: AssetLifecycle;

  // Metadata
  metadata: AssetMetadata;
}

// Asset categories
enum AssetCategory {
  FINANCIAL = 'FINANCIAL',
  REAL_PROPERTY = 'REAL_PROPERTY',
  PERSONAL_PROPERTY = 'PERSONAL_PROPERTY',
  DIGITAL = 'DIGITAL',
  INTELLECTUAL_PROPERTY = 'INTELLECTUAL_PROPERTY',
  BUSINESS_INTEREST = 'BUSINESS_INTEREST',
  CRYONICS_SPECIFIC = 'CRYONICS_SPECIFIC',
}

// Detailed asset types
interface AssetType {
  category: AssetCategory;
  type: string;
  subtype: string | null;
}

const AssetTypeRegistry = {
  FINANCIAL: {
    BANK_ACCOUNT: {
      subtypes: ['CHECKING', 'SAVINGS', 'MONEY_MARKET', 'CD'],
    },
    BROKERAGE: {
      subtypes: ['INDIVIDUAL', 'JOINT', 'MARGIN'],
    },
    RETIREMENT: {
      subtypes: ['401K', 'IRA', 'ROTH_IRA', 'SEP_IRA', 'PENSION'],
    },
    INSURANCE: {
      subtypes: ['LIFE_WHOLE', 'LIFE_TERM', 'LIFE_UNIVERSAL', 'ANNUITY'],
    },
    CRYPTOCURRENCY: {
      subtypes: ['BITCOIN', 'ETHEREUM', 'STABLECOIN', 'OTHER'],
    },
    BONDS: {
      subtypes: ['GOVERNMENT', 'MUNICIPAL', 'CORPORATE', 'SAVINGS'],
    },
  },

  REAL_PROPERTY: {
    RESIDENTIAL: {
      subtypes: ['PRIMARY_RESIDENCE', 'VACATION', 'RENTAL'],
    },
    COMMERCIAL: {
      subtypes: ['OFFICE', 'RETAIL', 'INDUSTRIAL', 'MIXED_USE'],
    },
    LAND: {
      subtypes: ['DEVELOPED', 'UNDEVELOPED', 'AGRICULTURAL'],
    },
  },

  DIGITAL: {
    ACCOUNT: {
      subtypes: ['EMAIL', 'SOCIAL_MEDIA', 'CLOUD_STORAGE', 'SUBSCRIPTION'],
    },
    DOMAIN: {
      subtypes: ['DOMAIN_NAME', 'WEBSITE', 'APP'],
    },
    NFT: {
      subtypes: ['ART', 'COLLECTIBLE', 'UTILITY'],
    },
    DATA: {
      subtypes: ['PERSONAL_ARCHIVE', 'MEDIA_LIBRARY', 'DOCUMENTS'],
    },
  },

  CRYONICS_SPECIFIC: {
    PATIENT_CARE_FUND: {
      subtypes: ['STORAGE', 'MAINTENANCE', 'EMERGENCY'],
    },
    REVIVAL_FUND: {
      subtypes: ['PROCEDURE', 'REHABILITATION', 'GENERAL'],
    },
    MEMBERSHIP: {
      subtypes: ['LIFETIME', 'ANNUAL', 'PREPAID'],
    },
  },
};

// Ownership structure
interface AssetOwnership {
  type: OwnershipType;
  holders: OwnershipHolder[];
  restrictions: OwnershipRestriction[];
  encumbrances: Encumbrance[];
}

enum OwnershipType {
  SOLE = 'SOLE',
  JOINT_TENANCY = 'JOINT_TENANCY',
  TENANCY_IN_COMMON = 'TENANCY_IN_COMMON',
  COMMUNITY_PROPERTY = 'COMMUNITY_PROPERTY',
  TRUST = 'TRUST',
  CORPORATE = 'CORPORATE',
  PARTNERSHIP = 'PARTNERSHIP',
}

interface OwnershipHolder {
  entityType: 'INDIVIDUAL' | 'TRUST' | 'CORPORATION' | 'PARTNERSHIP' | 'OTHER';
  entityId: string;
  name: string;
  percentage: number;  // 0-100
  rights: string[];  // e.g., 'INCOME', 'PRINCIPAL', 'VOTING'
}

interface OwnershipRestriction {
  type: 'TRANSFER' | 'SALE' | 'ENCUMBRANCE' | 'OTHER';
  description: string;
  source: string;  // Legal document reference
  expirationDate: Date | null;
}

interface Encumbrance {
  type: 'MORTGAGE' | 'LIEN' | 'EASEMENT' | 'PLEDGE' | 'OTHER';
  holder: string;
  amount: number | null;
  currency: string | null;
  description: string;
  documentReference: DocumentReference;
}
```

### Valuation Schema

```typescript
// Asset valuation structure
interface Valuation {
  id: string;
  assetId: string;

  // Valuation details
  valuationDate: Date;
  effectiveDate: Date;
  value: number;
  currency: string;  // ISO 4217

  // Methodology
  valuationMethod: ValuationMethod;
  valuationBasis: ValuationBasis;
  valuationSource: ValuationSource;

  // Supporting data
  marketData: MarketData | null;
  appraisalData: AppraisalData | null;
  assumptions: ValuationAssumption[];

  // Confidence and quality
  confidenceLevel: 'HIGH' | 'MEDIUM' | 'LOW';
  qualityScore: number;  // 0-100

  // Audit trail
  performedBy: string;
  reviewedBy: string | null;
  approvedBy: string | null;
  approvalDate: Date | null;

  // Blockchain anchor
  blockchainHash: string | null;
}

enum ValuationMethod {
  MARKET_QUOTE = 'MARKET_QUOTE',  // Real-time market price
  RECENT_TRANSACTION = 'RECENT_TRANSACTION',  // Based on recent sale
  APPRAISAL = 'APPRAISAL',  // Professional appraisal
  DISCOUNTED_CASH_FLOW = 'DISCOUNTED_CASH_FLOW',  // DCF analysis
  COMPARABLE_SALES = 'COMPARABLE_SALES',  // Comp-based
  BOOK_VALUE = 'BOOK_VALUE',  // Accounting book value
  COST_BASIS = 'COST_BASIS',  // Original cost
  ESTIMATED = 'ESTIMATED',  // Internal estimate
  NOTIONAL = 'NOTIONAL',  // Face/notional value
}

enum ValuationBasis {
  FAIR_MARKET_VALUE = 'FAIR_MARKET_VALUE',
  LIQUIDATION_VALUE = 'LIQUIDATION_VALUE',
  REPLACEMENT_COST = 'REPLACEMENT_COST',
  BOOK_VALUE = 'BOOK_VALUE',
  TAX_VALUE = 'TAX_VALUE',
  INSURANCE_VALUE = 'INSURANCE_VALUE',
}

interface ValuationSource {
  type: 'MARKET' | 'APPRAISER' | 'INSTITUTION' | 'INTERNAL' | 'TAX_AUTHORITY';
  name: string;
  identifier: string | null;
  timestamp: Date;
  rawData: Record<string, any> | null;
}

interface MarketData {
  exchange: string | null;
  ticker: string | null;
  price: number;
  currency: string;
  volume: number | null;
  bid: number | null;
  ask: number | null;
  timestamp: Date;
  source: string;
}

interface AppraisalData {
  appraiser: {
    name: string;
    company: string;
    credentials: string[];
    licenseNumber: string;
  };
  appraisalDate: Date;
  reportReference: DocumentReference;
  methodology: string;
  comparables: ComparableProperty[] | null;
}

interface ValuationAssumption {
  category: string;
  assumption: string;
  impact: 'HIGH' | 'MEDIUM' | 'LOW';
  sensitivity: string | null;
}
```

---

## 3.2 Trust and Legal Entity Schemas

### Trust Schema

```typescript
// Comprehensive trust representation
interface CryonicsTrust {
  // Identity
  id: string;
  externalId: string | null;
  name: string;

  // Type and classification
  trustType: TrustType;
  taxClassification: TaxClassification;

  // Jurisdiction
  jurisdiction: TrustJurisdiction;

  // Parties
  parties: TrustParties;

  // Terms
  terms: TrustTerms;

  // Assets
  assets: {
    initialFunding: FundingRecord[];
    currentHoldings: AssetHolding[];
    totalValue: number;
    lastValuationDate: Date;
  };

  // Governance
  governance: TrustGovernance;

  // Documentation
  documentation: TrustDocumentation;

  // Status
  status: TrustStatus;
  statusHistory: StatusChange[];

  // Metadata
  metadata: {
    createdAt: Date;
    updatedAt: Date;
    version: number;
    blockchainRef: BlockchainReference | null;
  };
}

enum TrustType {
  // Standard trust types
  REVOCABLE_LIVING = 'REVOCABLE_LIVING',
  IRREVOCABLE = 'IRREVOCABLE',
  TESTAMENTARY = 'TESTAMENTARY',

  // Cryonics-specific
  PERSONAL_REVIVAL = 'PERSONAL_REVIVAL',
  PATIENT_CARE = 'PATIENT_CARE',
  DYNASTY = 'DYNASTY',
  ASSET_PROTECTION = 'ASSET_PROTECTION',

  // Special purpose
  CHARITABLE_REMAINDER = 'CHARITABLE_REMAINDER',
  GRANTOR_RETAINED = 'GRANTOR_RETAINED',
  QUALIFIED_PERSONAL_RESIDENCE = 'QUALIFIED_PERSONAL_RESIDENCE',
  LIFE_INSURANCE = 'LIFE_INSURANCE',
}

interface TrustJurisdiction {
  state: string | null;  // For US trusts
  country: string;
  governingLaw: string;
  situs: string;  // Location of trust administration
  registrationNumber: string | null;
}

interface TrustParties {
  grantor: TrustParty;
  trustees: TrusteeInfo[];
  successorTrustees: TrusteeInfo[];
  trustProtector: TrustParty | null;
  beneficiaries: Beneficiary[];
  remainderBeneficiaries: Beneficiary[];
}

interface TrustParty {
  entityType: 'INDIVIDUAL' | 'CORPORATION' | 'TRUST' | 'OTHER';
  name: string;
  identifier: string;
  address: Address;
  contact: ContactInfo;
}

interface TrusteeInfo extends TrustParty {
  trusteeType: 'INDIVIDUAL' | 'CORPORATE' | 'PROFESSIONAL';
  powers: TrusteePower[];
  limitations: string[];
  compensation: TrusteeCompensation;
  bondRequired: boolean;
  bondAmount: number | null;
  appointmentDate: Date;
  terminationDate: Date | null;
}

interface TrusteePower {
  category: 'INVESTMENT' | 'DISTRIBUTION' | 'ADMINISTRATIVE' | 'TAX' | 'SPECIAL';
  power: string;
  limitations: string | null;
}

interface TrusteeCompensation {
  type: 'FIXED' | 'PERCENTAGE' | 'STATUTORY' | 'NONE';
  amount: number | null;
  percentage: number | null;
  frequency: 'ANNUAL' | 'MONTHLY' | 'ON_TRANSACTION';
  description: string;
}

interface Beneficiary {
  beneficiaryType: BeneficiaryType;
  party: TrustParty | null;  // Null for patient-upon-revival
  patientId: string | null;  // For cryonics beneficiaries
  interestType: 'INCOME' | 'PRINCIPAL' | 'BOTH' | 'CONTINGENT';
  share: number | null;  // Percentage, null for discretionary
  conditions: BeneficiaryCondition[];
  distributionStandard: string;
}

enum BeneficiaryType {
  INDIVIDUAL = 'INDIVIDUAL',
  CLASS = 'CLASS',  // e.g., "my descendants"
  CHARITY = 'CHARITY',
  PATIENT_UPON_REVIVAL = 'PATIENT_UPON_REVIVAL',
  CRYONICS_ORGANIZATION = 'CRYONICS_ORGANIZATION',
}

interface BeneficiaryCondition {
  type: 'AGE' | 'EVENT' | 'REVIVAL' | 'SURVIVAL' | 'OTHER';
  condition: string;
  description: string;
}

// Trust terms
interface TrustTerms {
  purpose: string;
  duration: TrustDuration;

  // Distribution provisions
  distributionProvisions: DistributionProvision[];

  // Revival-specific provisions
  revivalProvisions: RevivalProvisions | null;

  // Investment policy
  investmentPolicy: InvestmentPolicy;

  // Administrative provisions
  administrativeProvisions: AdministrativeProvision[];

  // Modification and termination
  modificationProvisions: ModificationProvision[];
  terminationConditions: TerminationCondition[];

  // Tax elections
  taxElections: TaxElection[];

  // Spendthrift provisions
  spendthrift: boolean;
  spendthriftExceptions: string[];
}

interface TrustDuration {
  type: 'PERPETUAL' | 'TERM' | 'LIFE' | 'UNTIL_EVENT';
  termYears: number | null;
  measuringLives: string[] | null;
  terminationEvent: string | null;
  ruleAgainstPerpetuities: boolean;
  rapCompliance: string | null;
}

interface RevivalProvisions {
  revivalDefinition: string;  // What constitutes successful revival
  identityVerificationMethod: string;  // How to verify identity
  revivalTriggerProcess: string;  // Process when revival occurs
  distributionUponRevival: DistributionProvision;
  rehabilitationPeriod: number | null;  // Months of support
  rehabilitationSupport: string;  // Description of support
  failedRevivalProvisions: string;  // If revival fails
  multipleRevivalAttempts: string;  // Policy for multiple attempts
}

interface DistributionProvision {
  id: string;
  type: 'MANDATORY' | 'DISCRETIONARY' | 'ASCERTAINABLE_STANDARD';
  trigger: string;
  beneficiaries: string[];  // Beneficiary IDs
  amount: DistributionAmount;
  frequency: 'ONE_TIME' | 'PERIODIC' | 'AS_NEEDED';
  conditions: string[];
  standard: string | null;  // For ascertainable standard (HEMS, etc.)
}

interface DistributionAmount {
  type: 'FIXED' | 'PERCENTAGE' | 'ALL' | 'NET_INCOME' | 'UNITRUST' | 'DISCRETIONARY';
  fixedAmount: number | null;
  percentage: number | null;
  currency: string | null;
  cap: number | null;
  floor: number | null;
}
```

---

## 3.3 Portfolio and Investment Schemas

### Portfolio Schema

```typescript
// Patient portfolio structure
interface PatientPortfolio {
  id: string;
  patientId: string;
  organizationId: string;

  // Status
  status: PortfolioStatus;

  // Core funds
  preservationFund: PreservationFund;
  revivalFund: RevivalFund;

  // Other assets
  personalAssets: AssetCategory[];
  trusts: TrustReference[];

  // Governance
  governance: PortfolioGovernance;

  // Performance
  performance: PortfolioPerformance;

  // Valuation
  totalValue: number;
  lastValuationDate: Date;
  valuationHistory: PortfolioValuation[];

  // Compliance
  compliance: ComplianceStatus;

  // Metadata
  metadata: {
    createdAt: Date;
    updatedAt: Date;
    nextReviewDate: Date;
  };
}

enum PortfolioStatus {
  ACTIVE = 'ACTIVE',
  SUSPENDED = 'SUSPENDED',
  DISTRIBUTING = 'DISTRIBUTING',
  TERMINATED = 'TERMINATED',
}

interface PreservationFund {
  id: string;
  type: 'PATIENT_CARE_FUND';

  // Target and actual
  targetAmount: number;
  currentAmount: number;
  fundingStatus: 'UNDERFUNDED' | 'ADEQUATE' | 'OVERFUNDED';

  // Structure
  structure: 'INTERNAL' | 'EXTERNAL_TRUST' | 'HYBRID';
  custodian: string;

  // Investments
  holdings: InvestmentHolding[];
  allocationPolicy: AllocationPolicy;
  currentAllocation: AssetAllocation;

  // Cash flow
  annualExpenses: number;
  projectedYearsOfFunding: number;
  lastDistribution: Date | null;
}

interface RevivalFund {
  id: string;
  type: 'REVIVAL_FUND';

  // Target and actual
  targetAmount: number | null;  // May be open-ended
  currentAmount: number;
  projectedRevivalCost: CostProjection;

  // Structure
  structure: 'TRUST' | 'ACCOUNT' | 'INSURANCE' | 'MIXED';
  trustId: string | null;
  custodians: string[];

  // Investments
  holdings: InvestmentHolding[];
  allocationPolicy: AllocationPolicy;
  currentAllocation: AssetAllocation;

  // Growth projection
  projectedGrowth: GrowthProjection[];
}

interface CostProjection {
  estimatedRevivalCost: number;
  estimatedRehabilitationCost: number;
  estimatedLivingSupport: number;
  totalProjectedNeed: number;
  projectionBasis: string;
  projectionDate: Date;
  confidenceInterval: { low: number; mid: number; high: number };
}

interface InvestmentHolding {
  id: string;
  security: SecurityInfo;
  quantity: number;
  costBasis: number;
  currentValue: number;
  unrealizedGain: number;
  weight: number;  // Percentage of portfolio
  acquisitionDate: Date;
  holdingPeriod: 'SHORT_TERM' | 'LONG_TERM';
}

interface SecurityInfo {
  type: SecurityType;
  identifier: SecurityIdentifier;
  name: string;
  issuer: string | null;
  currency: string;
  exchange: string | null;
  assetClass: string;
  sector: string | null;
  region: string | null;
}

enum SecurityType {
  EQUITY = 'EQUITY',
  FIXED_INCOME = 'FIXED_INCOME',
  MUTUAL_FUND = 'MUTUAL_FUND',
  ETF = 'ETF',
  REIT = 'REIT',
  COMMODITY = 'COMMODITY',
  CRYPTOCURRENCY = 'CRYPTOCURRENCY',
  CASH = 'CASH',
  ALTERNATIVE = 'ALTERNATIVE',
}

interface SecurityIdentifier {
  type: 'CUSIP' | 'ISIN' | 'SEDOL' | 'TICKER' | 'CONTRACT_ADDRESS' | 'OTHER';
  value: string;
}

interface AllocationPolicy {
  id: string;
  name: string;
  type: 'STRATEGIC' | 'TACTICAL' | 'DYNAMIC';

  // Target allocation
  targetAllocation: AllocationTarget[];

  // Constraints
  constraints: AllocationConstraint[];

  // Rebalancing
  rebalancingPolicy: RebalancingPolicy;

  // Risk parameters
  riskTolerance: 'CONSERVATIVE' | 'MODERATE' | 'AGGRESSIVE';
  maxDrawdown: number;
  volatilityTarget: number | null;

  // Time horizon
  timeHorizon: 'SHORT' | 'MEDIUM' | 'LONG' | 'PERPETUAL';
  timeHorizonYears: number | null;
}

interface AllocationTarget {
  assetClass: string;
  targetWeight: number;
  minWeight: number;
  maxWeight: number;
  benchmarkIndex: string | null;
}

interface AllocationConstraint {
  type: 'PROHIBITED' | 'LIMITED' | 'REQUIRED';
  target: string;  // Asset class, sector, or security
  limit: number | null;
  reason: string;
}

interface RebalancingPolicy {
  trigger: 'CALENDAR' | 'THRESHOLD' | 'HYBRID';
  calendarFrequency: 'MONTHLY' | 'QUARTERLY' | 'ANNUAL' | null;
  thresholdPercent: number | null;
  taxAware: boolean;
  tradingCostLimit: number | null;
}
```

---

## 3.4 Transaction and Event Schemas

### Transaction Schema

```typescript
// Financial transaction representation
interface CryoTransaction {
  id: string;
  portfolioId: string;
  accountId: string;

  // Transaction type
  type: TransactionType;
  subtype: string | null;

  // Timing
  transactionDate: Date;
  settlementDate: Date;
  recordedAt: Date;

  // Amounts
  amount: TransactionAmount;
  fees: Fee[];
  taxes: Tax[];
  netAmount: number;

  // Security details (for trades)
  security: SecurityInfo | null;
  quantity: number | null;
  price: number | null;

  // Parties
  counterparty: Counterparty | null;

  // Reference
  externalReference: string | null;
  parentTransactionId: string | null;
  relatedTransactions: string[];

  // Documentation
  documentation: DocumentReference[];

  // Status
  status: TransactionStatus;
  statusHistory: StatusChange[];

  // Blockchain
  blockchainRef: BlockchainReference | null;

  // Metadata
  metadata: {
    source: string;
    importedAt: Date | null;
    verifiedAt: Date | null;
    verifiedBy: string | null;
  };
}

enum TransactionType {
  // Cash movements
  DEPOSIT = 'DEPOSIT',
  WITHDRAWAL = 'WITHDRAWAL',
  TRANSFER = 'TRANSFER',
  FEE = 'FEE',
  TAX = 'TAX',

  // Trading
  BUY = 'BUY',
  SELL = 'SELL',
  SHORT = 'SHORT',
  COVER = 'COVER',

  // Income
  DIVIDEND = 'DIVIDEND',
  INTEREST = 'INTEREST',
  DISTRIBUTION = 'DISTRIBUTION',
  ROYALTY = 'ROYALTY',

  // Corporate actions
  SPLIT = 'SPLIT',
  MERGER = 'MERGER',
  SPINOFF = 'SPINOFF',
  RIGHTS = 'RIGHTS',

  // Trust-specific
  TRUST_FUNDING = 'TRUST_FUNDING',
  TRUST_DISTRIBUTION = 'TRUST_DISTRIBUTION',
  BENEFICIARY_PAYMENT = 'BENEFICIARY_PAYMENT',
  TRUSTEE_FEE = 'TRUSTEE_FEE',

  // Cryonics-specific
  MEMBERSHIP_PAYMENT = 'MEMBERSHIP_PAYMENT',
  PRESERVATION_FEE = 'PRESERVATION_FEE',
  STANDBY_PAYMENT = 'STANDBY_PAYMENT',
  REVIVAL_EXPENSE = 'REVIVAL_EXPENSE',
}

interface TransactionAmount {
  gross: number;
  net: number;
  currency: string;
  exchangeRate: number | null;
  baseCurrencyAmount: number | null;
}

interface Fee {
  type: 'COMMISSION' | 'PLATFORM' | 'MANAGEMENT' | 'TRANSACTION' | 'WIRE' | 'OTHER';
  amount: number;
  currency: string;
  description: string;
  payee: string | null;
}

interface Tax {
  type: 'WITHHOLDING' | 'CAPITAL_GAINS' | 'INCOME' | 'ESTATE' | 'OTHER';
  amount: number;
  currency: string;
  jurisdiction: string;
  rate: number | null;
  basis: number | null;
}

enum TransactionStatus {
  PENDING = 'PENDING',
  PROCESSING = 'PROCESSING',
  SETTLED = 'SETTLED',
  FAILED = 'FAILED',
  CANCELLED = 'CANCELLED',
  REVERSED = 'REVERSED',
}
```

### Event Schema

```typescript
// System and lifecycle events
interface CryoEvent {
  id: string;
  eventType: EventType;
  category: EventCategory;

  // Timing
  occurredAt: Date;
  recordedAt: Date;
  effectiveDate: Date | null;

  // Subject
  subject: EventSubject;

  // Details
  summary: string;
  details: Record<string, any>;
  impact: EventImpact | null;

  // Related entities
  relatedEntities: RelatedEntity[];

  // Actions
  triggeredActions: TriggeredAction[];
  requiredActions: RequiredAction[];

  // Notification
  notifications: EventNotification[];

  // Source
  source: EventSource;

  // Blockchain
  blockchainRef: BlockchainReference | null;
}

enum EventCategory {
  PATIENT = 'PATIENT',
  ASSET = 'ASSET',
  TRUST = 'TRUST',
  COMPLIANCE = 'COMPLIANCE',
  VALUATION = 'VALUATION',
  GOVERNANCE = 'GOVERNANCE',
  SYSTEM = 'SYSTEM',
}

enum EventType {
  // Patient events
  PATIENT_PRESERVED = 'PATIENT_PRESERVED',
  PATIENT_REVIVED = 'PATIENT_REVIVED',
  PATIENT_MEMBERSHIP_CHANGE = 'PATIENT_MEMBERSHIP_CHANGE',
  PATIENT_DOCUMENT_UPDATE = 'PATIENT_DOCUMENT_UPDATE',

  // Asset events
  ASSET_REGISTERED = 'ASSET_REGISTERED',
  ASSET_VALUED = 'ASSET_VALUED',
  ASSET_TRANSFERRED = 'ASSET_TRANSFERRED',
  ASSET_LIQUIDATED = 'ASSET_LIQUIDATED',
  ASSET_STATUS_CHANGE = 'ASSET_STATUS_CHANGE',

  // Trust events
  TRUST_CREATED = 'TRUST_CREATED',
  TRUST_FUNDED = 'TRUST_FUNDED',
  TRUST_DISTRIBUTION = 'TRUST_DISTRIBUTION',
  TRUST_AMENDMENT = 'TRUST_AMENDMENT',
  TRUST_TERMINATED = 'TRUST_TERMINATED',
  TRUSTEE_CHANGE = 'TRUSTEE_CHANGE',

  // Compliance events
  COMPLIANCE_REVIEW = 'COMPLIANCE_REVIEW',
  COMPLIANCE_VIOLATION = 'COMPLIANCE_VIOLATION',
  FILING_SUBMITTED = 'FILING_SUBMITTED',
  AUDIT_COMPLETED = 'AUDIT_COMPLETED',

  // Governance events
  INVESTMENT_DECISION = 'INVESTMENT_DECISION',
  POLICY_CHANGE = 'POLICY_CHANGE',
  APPROVAL_GRANTED = 'APPROVAL_GRANTED',
  APPROVAL_DENIED = 'APPROVAL_DENIED',
}

interface EventSubject {
  type: 'PATIENT' | 'ASSET' | 'TRUST' | 'PORTFOLIO' | 'ORGANIZATION';
  id: string;
  name: string;
}

interface EventImpact {
  financial: {
    valueChange: number | null;
    cashFlowImpact: number | null;
  } | null;
  compliance: {
    riskLevel: 'HIGH' | 'MEDIUM' | 'LOW' | null;
    filingRequired: boolean;
  } | null;
  operational: {
    actionRequired: boolean;
    urgency: 'IMMEDIATE' | 'SOON' | 'ROUTINE' | null;
  } | null;
}

interface TriggeredAction {
  actionType: string;
  status: 'COMPLETED' | 'IN_PROGRESS' | 'FAILED';
  result: Record<string, any> | null;
  executedAt: Date;
}

interface RequiredAction {
  actionType: string;
  description: string;
  assignee: string | null;
  dueDate: Date | null;
  priority: 'HIGH' | 'MEDIUM' | 'LOW';
  status: 'PENDING' | 'IN_PROGRESS' | 'COMPLETED' | 'OVERDUE';
}
```

---

## 3.5 Data Exchange Formats

### API Data Transfer Objects

```typescript
// Standard API response wrapper
interface ApiResponse<T> {
  success: boolean;
  data: T | null;
  error: ApiError | null;
  metadata: ResponseMetadata;
}

interface ApiError {
  code: string;
  message: string;
  details: Record<string, any> | null;
  field: string | null;  // For validation errors
  stack: string | null;  // Only in development
}

interface ResponseMetadata {
  requestId: string;
  timestamp: Date;
  version: string;
  pagination: PaginationInfo | null;
}

interface PaginationInfo {
  page: number;
  pageSize: number;
  totalItems: number;
  totalPages: number;
  hasNext: boolean;
  hasPrevious: boolean;
}

// Patient DTOs
interface PatientSummaryDTO {
  id: string;
  name: string;
  preservationStatus: PreservationStatus;
  organizationId: string;
  portfolioValue: number;
  lastUpdated: Date;
}

interface PatientDetailDTO extends PatientSummaryDTO {
  legalIdentity: {
    fullName: string;
    dateOfBirth: Date;
    dateOfLegalDeath: Date | null;
    citizenship: string[];
  };
  portfolioSummary: PortfolioSummaryDTO;
  trusts: TrustSummaryDTO[];
  recentActivity: ActivityDTO[];
}

// Asset DTOs
interface AssetSummaryDTO {
  id: string;
  name: string;
  category: AssetCategory;
  type: string;
  currentValue: number;
  currency: string;
  status: AssetStatus;
}

interface AssetDetailDTO extends AssetSummaryDTO {
  description: string;
  ownership: OwnershipDTO;
  valuationHistory: ValuationSummaryDTO[];
  location: LocationDTO | null;
  documentation: DocumentListDTO;
  blockchainRef: string | null;
}

// Portfolio DTOs
interface PortfolioSummaryDTO {
  id: string;
  patientId: string;
  totalValue: number;
  preservationFundValue: number;
  revivalFundValue: number;
  personalAssetsValue: number;
  lastValuationDate: Date;
  fundingStatus: 'UNDERFUNDED' | 'ADEQUATE' | 'OVERFUNDED';
}

interface PortfolioDetailDTO extends PortfolioSummaryDTO {
  preservationFund: FundDetailDTO;
  revivalFund: FundDetailDTO;
  holdings: HoldingDTO[];
  allocation: AllocationDTO;
  performance: PerformanceDTO;
  governance: GovernanceDTO;
}

// Bulk data exchange
interface BulkDataExport {
  exportId: string;
  exportDate: Date;
  exportedBy: string;
  format: 'JSON' | 'CSV' | 'XML';

  // Content
  patients: PatientExportRecord[];
  assets: AssetExportRecord[];
  trusts: TrustExportRecord[];
  transactions: TransactionExportRecord[];

  // Metadata
  recordCounts: {
    patients: number;
    assets: number;
    trusts: number;
    transactions: number;
  };
  checksum: string;
  signature: string;
}

interface PatientExportRecord {
  patientId: string;
  externalIds: Record<string, string>;
  legalName: string;
  dateOfBirth: string;  // ISO 8601
  dateOfLegalDeath: string | null;
  preservationStatus: string;
  preservationDate: string | null;
  organizationId: string;
  portfolioId: string;
  totalAssetValue: number;
  assetValueCurrency: string;
  lastUpdated: string;
}

interface AssetExportRecord {
  assetId: string;
  portfolioId: string;
  patientId: string;
  category: string;
  type: string;
  name: string;
  description: string;
  ownershipType: string;
  ownershipPercentage: number;
  currentValue: number;
  currency: string;
  valuationDate: string;
  valuationMethod: string;
  status: string;
  blockchainRef: string | null;
}
```

### Inter-Organization Data Exchange

```typescript
// Secure data exchange between cryonics organizations
interface OrganizationDataExchange {
  exchangeId: string;
  exchangeType: ExchangeType;

  // Parties
  sourceOrganization: OrganizationIdentifier;
  targetOrganization: OrganizationIdentifier;

  // Security
  encryption: EncryptionInfo;
  signature: SignatureInfo;

  // Content
  contentType: string;
  content: EncryptedPayload;

  // Audit
  timestamp: Date;
  expiresAt: Date;
  accessLog: AccessLogEntry[];
}

enum ExchangeType {
  PATIENT_TRANSFER = 'PATIENT_TRANSFER',
  ASSET_VERIFICATION = 'ASSET_VERIFICATION',
  VALUATION_SHARE = 'VALUATION_SHARE',
  COMPLIANCE_REPORT = 'COMPLIANCE_REPORT',
  EMERGENCY_NOTIFICATION = 'EMERGENCY_NOTIFICATION',
}

interface EncryptionInfo {
  algorithm: 'AES-256-GCM' | 'ChaCha20-Poly1305';
  keyExchange: 'ECDH-P384' | 'X25519';
  keyId: string;
  nonce: string;
}

interface SignatureInfo {
  algorithm: 'Ed25519' | 'ECDSA-P384';
  publicKey: string;
  signature: string;
  signedAt: Date;
}

interface EncryptedPayload {
  ciphertext: string;  // Base64
  authTag: string;  // For AEAD
  metadata: Record<string, string>;
}

// Patient transfer record
interface PatientTransferRecord {
  transferId: string;
  patientId: string;

  // Organizations
  sourceOrg: {
    id: string;
    name: string;
    patientId: string;  // Their ID for patient
  };
  targetOrg: {
    id: string;
    name: string;
    newPatientId: string;
  };

  // Transfer details
  transferType: 'FULL' | 'PARTIAL';
  transferDate: Date;
  effectiveDate: Date;

  // Assets transferred
  assetsTransferred: TransferredAsset[];
  totalValueTransferred: number;

  // Documentation
  transferAgreement: DocumentReference;
  supportingDocuments: DocumentReference[];

  // Verification
  sourceSignature: string;
  targetAcknowledgment: string;
  verificationStatus: 'PENDING' | 'VERIFIED' | 'DISPUTED';
}

interface TransferredAsset {
  assetId: string;
  assetType: string;
  name: string;
  transferredValue: number;
  newAssetId: string | null;
  transferComplete: boolean;
}
```

---

## 3.6 Blockchain Data Structures

### On-Chain Asset Registration

```typescript
// Blockchain data structures for asset registration
interface BlockchainAssetRecord {
  // On-chain data (immutable)
  assetHash: string;  // SHA-256 of asset details
  registrationTimestamp: number;  // Unix timestamp
  registrar: string;  // Wallet address
  organizationId: string;

  // Asset classification (on-chain)
  assetCategory: number;  // Enum as number
  assetType: number;

  // Valuation anchor
  valuationHash: string;
  valuationTimestamp: number;

  // Off-chain reference
  ipfsHash: string;  // Full details on IPFS
  encryptionKeyId: string;  // Key to decrypt IPFS content
}

// Smart contract interface
interface ICryoAssetRegistry {
  // Registration
  registerAsset(
    assetHash: string,
    category: number,
    assetType: number,
    valuationHash: string,
    ipfsHash: string,
    encryptionKeyId: string
  ): Promise<string>;  // Returns transaction hash

  // Update valuation
  updateValuation(
    assetHash: string,
    newValuationHash: string,
    timestamp: number
  ): Promise<string>;

  // Transfer
  transferAsset(
    assetHash: string,
    newOrganization: string,
    transferHash: string
  ): Promise<string>;

  // Query
  getAssetRecord(assetHash: string): Promise<BlockchainAssetRecord>;
  getAssetHistory(assetHash: string): Promise<AssetHistoryEntry[]>;
  getOrganizationAssets(orgId: string): Promise<string[]>;
}

// Solidity contract structure
const solidityContract = `
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

contract CryoAssetRegistry {
    struct AssetRecord {
        bytes32 assetHash;
        uint256 registrationTimestamp;
        address registrar;
        bytes32 organizationId;
        uint8 assetCategory;
        uint16 assetType;
        bytes32 currentValuationHash;
        uint256 lastValuationTimestamp;
        string ipfsHash;
        bytes32 encryptionKeyId;
        bool isActive;
    }

    struct ValuationHistory {
        bytes32 valuationHash;
        uint256 timestamp;
        address updatedBy;
    }

    mapping(bytes32 => AssetRecord) public assets;
    mapping(bytes32 => ValuationHistory[]) public valuationHistory;
    mapping(bytes32 => bytes32[]) public organizationAssets;

    event AssetRegistered(
        bytes32 indexed assetHash,
        bytes32 indexed organizationId,
        uint256 timestamp
    );

    event ValuationUpdated(
        bytes32 indexed assetHash,
        bytes32 valuationHash,
        uint256 timestamp
    );

    event AssetTransferred(
        bytes32 indexed assetHash,
        bytes32 fromOrg,
        bytes32 toOrg,
        uint256 timestamp
    );

    function registerAsset(
        bytes32 _assetHash,
        uint8 _category,
        uint16 _assetType,
        bytes32 _valuationHash,
        string memory _ipfsHash,
        bytes32 _encryptionKeyId,
        bytes32 _organizationId
    ) external returns (bool) {
        require(assets[_assetHash].registrationTimestamp == 0, "Asset exists");

        assets[_assetHash] = AssetRecord({
            assetHash: _assetHash,
            registrationTimestamp: block.timestamp,
            registrar: msg.sender,
            organizationId: _organizationId,
            assetCategory: _category,
            assetType: _assetType,
            currentValuationHash: _valuationHash,
            lastValuationTimestamp: block.timestamp,
            ipfsHash: _ipfsHash,
            encryptionKeyId: _encryptionKeyId,
            isActive: true
        });

        valuationHistory[_assetHash].push(ValuationHistory({
            valuationHash: _valuationHash,
            timestamp: block.timestamp,
            updatedBy: msg.sender
        }));

        organizationAssets[_organizationId].push(_assetHash);

        emit AssetRegistered(_assetHash, _organizationId, block.timestamp);
        return true;
    }

    function updateValuation(
        bytes32 _assetHash,
        bytes32 _newValuationHash
    ) external returns (bool) {
        require(assets[_assetHash].isActive, "Asset not active");

        assets[_assetHash].currentValuationHash = _newValuationHash;
        assets[_assetHash].lastValuationTimestamp = block.timestamp;

        valuationHistory[_assetHash].push(ValuationHistory({
            valuationHash: _newValuationHash,
            timestamp: block.timestamp,
            updatedBy: msg.sender
        }));

        emit ValuationUpdated(_assetHash, _newValuationHash, block.timestamp);
        return true;
    }

    function getAssetRecord(bytes32 _assetHash)
        external view returns (AssetRecord memory)
    {
        return assets[_assetHash];
    }

    function getValuationHistory(bytes32 _assetHash)
        external view returns (ValuationHistory[] memory)
    {
        return valuationHistory[_assetHash];
    }
}
`;
```

---

## 3.7 Document and File Schemas

### Document Management

```typescript
// Document reference and management
interface CryoDocument {
  id: string;
  type: DocumentType;
  category: DocumentCategory;

  // Identification
  title: string;
  description: string;
  documentNumber: string | null;

  // Content
  format: DocumentFormat;
  size: number;  // Bytes
  pages: number | null;
  language: string;  // ISO 639-1

  // Storage
  storage: DocumentStorage;

  // Integrity
  checksum: string;  // SHA-256
  signature: DocumentSignature | null;

  // Classification
  confidentiality: ConfidentialityLevel;
  retentionPolicy: RetentionPolicy;

  // Relationships
  relatedEntities: EntityReference[];
  supersedes: string | null;
  supersededBy: string | null;

  // Lifecycle
  status: DocumentStatus;
  effectiveDate: Date | null;
  expirationDate: Date | null;

  // Audit
  createdBy: string;
  createdAt: Date;
  modifiedBy: string | null;
  modifiedAt: Date | null;
  accessLog: DocumentAccess[];
}

enum DocumentType {
  // Legal documents
  TRUST_INSTRUMENT = 'TRUST_INSTRUMENT',
  WILL = 'WILL',
  POWER_OF_ATTORNEY = 'POWER_OF_ATTORNEY',
  ADVANCE_DIRECTIVE = 'ADVANCE_DIRECTIVE',

  // Identity documents
  PASSPORT = 'PASSPORT',
  DRIVERS_LICENSE = 'DRIVERS_LICENSE',
  BIRTH_CERTIFICATE = 'BIRTH_CERTIFICATE',
  DEATH_CERTIFICATE = 'DEATH_CERTIFICATE',

  // Financial documents
  BANK_STATEMENT = 'BANK_STATEMENT',
  BROKERAGE_STATEMENT = 'BROKERAGE_STATEMENT',
  TAX_RETURN = 'TAX_RETURN',
  INSURANCE_POLICY = 'INSURANCE_POLICY',

  // Asset documents
  DEED = 'DEED',
  TITLE = 'TITLE',
  APPRAISAL = 'APPRAISAL',
  CERTIFICATE = 'CERTIFICATE',

  // Cryonics documents
  MEMBERSHIP_AGREEMENT = 'MEMBERSHIP_AGREEMENT',
  STANDBY_INSTRUCTIONS = 'STANDBY_INSTRUCTIONS',
  PRESERVATION_RECORD = 'PRESERVATION_RECORD',

  // Administrative
  CORRESPONDENCE = 'CORRESPONDENCE',
  REPORT = 'REPORT',
  RECEIPT = 'RECEIPT',
  OTHER = 'OTHER',
}

enum DocumentCategory {
  LEGAL = 'LEGAL',
  FINANCIAL = 'FINANCIAL',
  IDENTITY = 'IDENTITY',
  PROPERTY = 'PROPERTY',
  MEDICAL = 'MEDICAL',
  CRYONICS = 'CRYONICS',
  ADMINISTRATIVE = 'ADMINISTRATIVE',
}

interface DocumentStorage {
  location: 'LOCAL' | 'S3' | 'IPFS' | 'ARCHIVE';
  path: string;
  encrypted: boolean;
  encryptionKeyId: string | null;

  // For distributed storage
  ipfsHash: string | null;
  arweaveId: string | null;

  // Backup
  backupLocations: string[];
  lastBackup: Date | null;
}

interface DocumentSignature {
  signer: string;
  signerRole: string;
  signatureType: 'DIGITAL' | 'ELECTRONIC' | 'WET_INK';
  algorithm: string | null;
  signature: string;
  certificate: string | null;
  signedAt: Date;
  verified: boolean;
  verifiedAt: Date | null;
}

enum ConfidentialityLevel {
  PUBLIC = 'PUBLIC',
  INTERNAL = 'INTERNAL',
  CONFIDENTIAL = 'CONFIDENTIAL',
  HIGHLY_CONFIDENTIAL = 'HIGHLY_CONFIDENTIAL',
  RESTRICTED = 'RESTRICTED',
}

interface RetentionPolicy {
  policyId: string;
  retentionPeriod: number;  // Days, -1 for permanent
  disposition: 'DELETE' | 'ARCHIVE' | 'REVIEW';
  legalHold: boolean;
  legalHoldReason: string | null;
}
```

---

## 3.8 Data Validation Rules

### Validation Schema

```typescript
// Validation rules for core entities
import { z } from 'zod';

// Patient validation
const PatientSchema = z.object({
  id: z.string().uuid(),
  organizationPatientId: z.string().min(1).max(50),

  legalIdentity: z.object({
    fullLegalName: z.string().min(2).max(200),
    previousNames: z.array(z.string()).default([]),
    dateOfBirth: z.date().max(new Date()),
    dateOfLegalDeath: z.date().nullable(),
    placeOfBirth: z.string().max(200),
    citizenship: z.array(z.string().length(2)).min(1),  // ISO 3166-1 alpha-2
  }),

  preservationStatus: z.object({
    status: z.nativeEnum(PreservationStatus),
    preservationType: z.enum(['WHOLE_BODY', 'NEURO', 'BRAIN_ONLY']),
    preservationDate: z.date().nullable(),
    organizationId: z.string().uuid(),
  }),
});

// Asset validation
const AssetSchema = z.object({
  id: z.string().uuid(),
  portfolioId: z.string().uuid(),
  patientId: z.string().uuid(),

  category: z.nativeEnum(AssetCategory),
  name: z.string().min(1).max(500),
  description: z.string().max(5000),

  ownership: z.object({
    type: z.nativeEnum(OwnershipType),
    holders: z.array(z.object({
      entityType: z.enum(['INDIVIDUAL', 'TRUST', 'CORPORATION', 'PARTNERSHIP', 'OTHER']),
      entityId: z.string(),
      name: z.string(),
      percentage: z.number().min(0).max(100),
    })).min(1),
  }).refine(
    (ownership) => {
      const totalPercentage = ownership.holders.reduce((sum, h) => sum + h.percentage, 0);
      return Math.abs(totalPercentage - 100) < 0.01;
    },
    { message: 'Ownership percentages must sum to 100%' }
  ),

  currentValuation: z.object({
    value: z.number().nonnegative(),
    currency: z.string().length(3),  // ISO 4217
    valuationDate: z.date(),
    valuationMethod: z.nativeEnum(ValuationMethod),
  }),

  status: z.nativeEnum(AssetStatus),
});

// Trust validation
const TrustSchema = z.object({
  id: z.string().uuid(),
  name: z.string().min(1).max(500),
  trustType: z.nativeEnum(TrustType),

  jurisdiction: z.object({
    state: z.string().nullable(),
    country: z.string().length(2),
    governingLaw: z.string(),
  }),

  parties: z.object({
    grantor: z.object({
      entityType: z.enum(['INDIVIDUAL', 'CORPORATION', 'TRUST', 'OTHER']),
      name: z.string(),
      identifier: z.string(),
    }),
    trustees: z.array(z.object({
      entityType: z.enum(['INDIVIDUAL', 'CORPORATE', 'PROFESSIONAL']),
      name: z.string(),
      appointmentDate: z.date(),
    })).min(1),
    beneficiaries: z.array(z.object({
      beneficiaryType: z.nativeEnum(BeneficiaryType),
      interestType: z.enum(['INCOME', 'PRINCIPAL', 'BOTH', 'CONTINGENT']),
    })).min(1),
  }),

  status: z.nativeEnum(TrustStatus),
});

// Valuation validation
const ValuationSchema = z.object({
  id: z.string().uuid(),
  assetId: z.string().uuid(),
  valuationDate: z.date(),
  value: z.number().nonnegative(),
  currency: z.string().length(3),
  valuationMethod: z.nativeEnum(ValuationMethod),
  confidenceLevel: z.enum(['HIGH', 'MEDIUM', 'LOW']),
  performedBy: z.string(),
}).refine(
  (val) => val.valuationDate <= new Date(),
  { message: 'Valuation date cannot be in the future' }
);

// Custom validation functions
class DataValidator {
  validatePatient(data: unknown): ValidationResult<CryoPatient> {
    try {
      const parsed = PatientSchema.parse(data);
      return { valid: true, data: parsed as CryoPatient, errors: [] };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          valid: false,
          data: null,
          errors: error.errors.map(e => ({
            path: e.path.join('.'),
            message: e.message,
            code: e.code,
          })),
        };
      }
      throw error;
    }
  }

  validateAsset(data: unknown): ValidationResult<CryoAsset> {
    try {
      const parsed = AssetSchema.parse(data);
      return { valid: true, data: parsed as CryoAsset, errors: [] };
    } catch (error) {
      if (error instanceof z.ZodError) {
        return {
          valid: false,
          data: null,
          errors: error.errors.map(e => ({
            path: e.path.join('.'),
            message: e.message,
            code: e.code,
          })),
        };
      }
      throw error;
    }
  }

  // Cross-entity validation
  validatePortfolioIntegrity(portfolio: PatientPortfolio): ValidationResult<void> {
    const errors: ValidationError[] = [];

    // Check preservation fund adequacy
    if (portfolio.preservationFund.currentAmount < portfolio.preservationFund.targetAmount * 0.5) {
      errors.push({
        path: 'preservationFund.currentAmount',
        message: 'Preservation fund is below 50% of target',
        code: 'UNDERFUNDED',
      });
    }

    // Check all assets have recent valuations
    const thirtyDaysAgo = new Date(Date.now() - 30 * 24 * 60 * 60 * 1000);
    if (portfolio.lastValuationDate < thirtyDaysAgo) {
      errors.push({
        path: 'lastValuationDate',
        message: 'Portfolio valuation is more than 30 days old',
        code: 'STALE_VALUATION',
      });
    }

    // Check governance requirements
    if (!portfolio.governance.trustees || portfolio.governance.trustees.length === 0) {
      errors.push({
        path: 'governance.trustees',
        message: 'Portfolio must have at least one trustee',
        code: 'MISSING_TRUSTEE',
      });
    }

    return {
      valid: errors.length === 0,
      data: undefined,
      errors,
    };
  }
}

interface ValidationResult<T> {
  valid: boolean;
  data: T | null;
  errors: ValidationError[];
}

interface ValidationError {
  path: string;
  message: string;
  code: string;
}
```

---

## Chapter Summary

This chapter defined the comprehensive data formats and schemas required for cryonics asset management:

1. **Core Entity Schemas**: Patient identity, asset representation, and valuation structures
2. **Trust Schemas**: Legal entity representation including cryonics-specific provisions
3. **Portfolio Schemas**: Investment holdings, allocation policies, and performance tracking
4. **Transaction Schemas**: Financial transaction and event recording
5. **Exchange Formats**: Inter-organization data exchange with security
6. **Blockchain Structures**: On-chain asset registration and verification
7. **Document Schemas**: Comprehensive document management
8. **Validation Rules**: Data integrity and consistency checking

These schemas provide the foundation for building interoperable cryonics asset management systems that can reliably track and manage assets over the extended time horizons required for cryonics preservation.

---

*Next Chapter: API Interface - Service integration specifications for the Cryo-Asset platform*
