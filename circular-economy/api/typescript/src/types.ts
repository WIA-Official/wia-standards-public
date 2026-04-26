/**
 * WIA-IND-030: Circular Economy Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Circular Economy Types
// ============================================================================

/**
 * Lifecycle stage in circular economy
 */
export type LifecycleStage =
  | 'design'
  | 'production'
  | 'distribution'
  | 'use'
  | 'collection'
  | 'refurbishment'
  | 'recycling'
  | 'disposal'
  | 'regeneration';

/**
 * Material origin type
 */
export type MaterialOrigin = 'virgin' | 'recycled' | 'bio-based' | 'renewable';

/**
 * Circularity rating
 */
export type CircularityRating = 'A' | 'B' | 'C' | 'D' | 'E';

/**
 * Product condition
 */
export type ProductCondition =
  | 'new'
  | 'like-new'
  | 'excellent'
  | 'good'
  | 'fair'
  | 'refurbished'
  | 'end-of-life';

// ============================================================================
// Material Passport
// ============================================================================

/**
 * Material type with properties
 */
export interface Material {
  /** Material type/name */
  type: string;

  /** Material mass */
  mass: number;

  /** Mass unit */
  massUnit: 'kg' | 'g' | 'lb' | 'oz';

  /** Recyclability percentage (0-100) */
  recyclability: number;

  /** Recycled content percentage (0-100) */
  recycledContent?: number;

  /** Material origin */
  origin: MaterialOrigin;

  /** Supplier information */
  supplier?: string;

  /** Certifications */
  certifications?: string[];

  /** Toxicity level */
  toxicity?: 'none' | 'low' | 'medium' | 'high';

  /** Is critical raw material */
  criticalMaterial?: boolean;

  /** Biodegradable */
  biodegradable?: boolean;

  /** Compostable */
  compostable?: boolean;

  /** Chemical composition */
  chemicalComposition?: Record<string, number>;
}

/**
 * Design for circularity principles
 */
export interface CircularDesignPrinciples {
  /** Modular design */
  modular: boolean;

  /** Can be disassembled */
  disassemblable: boolean;

  /** Uses standardized components */
  standardizedComponents?: boolean;

  /** Uses standardized fasteners */
  standardizedFasteners?: boolean;

  /** Reparability index (0-10) */
  repairabilityIndex: number;

  /** Upgradeability */
  upgradeability?: boolean;

  /** Designed for multiple lifecycles */
  multipleLifecycles?: boolean;

  /** Mono-material design */
  monoMaterial?: boolean;
}

/**
 * Blockchain verification for material passport
 */
export interface BlockchainVerification {
  /** Blockchain network */
  network: 'ethereum' | 'polygon' | 'hyperledger' | 'private';

  /** Smart contract address */
  contractAddress?: string;

  /** NFT token ID */
  tokenId?: string;

  /** Transaction hash */
  txHash?: string;

  /** Verified status */
  verified: boolean;

  /** Verification timestamp */
  verifiedAt?: Date | string;
}

/**
 * End-of-life information
 */
export interface EndOfLifeInfo {
  /** Take-back program available */
  takebackProgram: boolean;

  /** Recycling instructions */
  recyclingInstructions?: string;

  /** Expected recovery rate (0-100%) */
  recoveryRate?: number;

  /** Disposal restrictions */
  disposalRestrictions?: string[];

  /** Hazardous components */
  hazardousComponents?: string[];

  /** Recycling facilities */
  recyclingFacilities?: string[];
}

/**
 * Material passport - digital identity for materials
 */
export interface MaterialPassport {
  /** Unique passport ID */
  passportId: string;

  /** Associated product ID */
  productId: string;

  /** Product name */
  productName?: string;

  /** Manufacturer */
  manufacturer?: string;

  /** Manufacture date */
  manufactureDate?: Date | string;

  /** Materials used */
  materials: Material[];

  /** Overall recyclability (0-100%) */
  overallRecyclability: number;

  /** Total recycled content (0-100%) */
  totalRecycledContent?: number;

  /** Design principles */
  designPrinciples: CircularDesignPrinciples;

  /** Expected lifespan in days */
  expectedLifespan?: number;

  /** Warranty period in days */
  warrantyPeriod?: number;

  /** Refurbishment potential */
  refurbishmentPotential?: 'high' | 'medium' | 'low';

  /** End-of-life information */
  endOfLife: EndOfLifeInfo;

  /** Blockchain verification */
  blockchain?: BlockchainVerification;

  /** Certifications */
  certifications?: string[];

  /** Created timestamp */
  createdAt: Date | string;

  /** Updated timestamp */
  updatedAt: Date | string;
}

// ============================================================================
// Product Lifecycle Tracking
// ============================================================================

/**
 * Lifecycle event
 */
export interface LifecycleEvent {
  /** Event ID */
  eventId: string;

  /** Event type */
  type: LifecycleStage;

  /** Timestamp */
  timestamp: Date | string;

  /** Location */
  location?: {
    name: string;
    address?: string;
    coordinates?: {
      latitude: number;
      longitude: number;
    };
  };

  /** Actor (person, company, facility) */
  actor?: string;

  /** Description */
  description: string;

  /** Condition at this event */
  condition?: ProductCondition;

  /** Data recorded */
  data?: Record<string, any>;

  /** Blockchain transaction */
  txHash?: string;
}

/**
 * Refurbishment record
 */
export interface RefurbishmentRecord {
  /** Refurbishment ID */
  refurbishmentId: string;

  /** Date of refurbishment */
  date: Date | string;

  /** Facility */
  facility: string;

  /** Parts replaced */
  partsReplaced?: Array<{
    part: string;
    quantity: number;
    reason: string;
  }>;

  /** Condition before */
  conditionBefore: ProductCondition;

  /** Condition after */
  conditionAfter: ProductCondition;

  /** Cost */
  cost?: number;

  /** Currency */
  currency?: string;

  /** Certification */
  certification?: string;

  /** Warranty extended (days) */
  warrantyExtension?: number;

  /** Notes */
  notes?: string;
}

/**
 * Product lifecycle tracking
 */
export interface ProductLifecycle {
  /** Product ID */
  productId: string;

  /** Current stage */
  currentStage: LifecycleStage;

  /** Current condition */
  currentCondition: ProductCondition;

  /** Product age in days */
  age: number;

  /** Expected remaining life in days */
  remainingLife?: number;

  /** Usage hours (if applicable) */
  usageHours?: number;

  /** Current owner */
  currentOwner?: string;

  /** Ownership history */
  ownershipHistory?: Array<{
    owner: string;
    startDate: Date | string;
    endDate?: Date | string;
    ownershipType: 'purchase' | 'lease' | 'rental' | 'sharing';
  }>;

  /** Lifecycle events */
  events: LifecycleEvent[];

  /** Refurbishment history */
  refurbishmentHistory: RefurbishmentRecord[];

  /** Number of refurbishments */
  refurbishmentCount: number;

  /** Total usage cycles */
  usageCycles?: number;

  /** Carbon footprint accumulated */
  accumulatedCarbon?: number;

  /** Created timestamp */
  createdAt: Date | string;

  /** Updated timestamp */
  updatedAt: Date | string;
}

// ============================================================================
// Circularity Metrics
// ============================================================================

/**
 * Circular economy score breakdown
 */
export interface CircularityScoreBreakdown {
  /** Recycled input materials score (0-100) */
  recycledInput: number;

  /** Product longevity score (0-100) */
  longevity: number;

  /** Design for circularity score (0-100) */
  design: number;

  /** End-of-life recovery score (0-100) */
  eolRecovery: number;

  /** Reparability score (0-100) */
  reparability: number;

  /** Material efficiency score (0-100) */
  materialEfficiency: number;
}

/**
 * Circularity assessment
 */
export interface CircularityAssessment {
  /** Product ID */
  productId: string;

  /** Overall circularity score (0-100) */
  score: number;

  /** Circularity rating */
  rating: CircularityRating;

  /** Score breakdown */
  breakdown: CircularityScoreBreakdown;

  /** Material Circularity Indicator (MCI) */
  mci?: number;

  /** Carbon saved vs virgin production (kg CO2) */
  carbonSaved?: number;

  /** Carbon reduction percentage */
  carbonReduction?: number;

  /** Waste reduction percentage */
  wasteReduction?: number;

  /** Resource productivity (value/mass) */
  resourceProductivity?: number;

  /** Recommendations for improvement */
  recommendations?: Array<{
    category: string;
    priority: 'high' | 'medium' | 'low';
    action: string;
    potentialImpact: number; // Score improvement
  }>;

  /** Assessment timestamp */
  assessedAt: Date | string;

  /** Valid until */
  validUntil?: Date | string;
}

// ============================================================================
// Recycling & Recovery
// ============================================================================

/**
 * Recycling facility
 */
export interface RecyclingFacility {
  /** Facility ID */
  id: string;

  /** Facility name */
  name: string;

  /** Address */
  address: {
    street: string;
    city: string;
    state?: string;
    postalCode: string;
    country: string;
    coordinates?: {
      latitude: number;
      longitude: number;
    };
  };

  /** Materials accepted */
  materialsAccepted: string[];

  /** Processing capabilities */
  processingCapabilities: Array<{
    material: string;
    recoveryRate: number; // Percentage
    capacity: number; // kg/day
  }>;

  /** Certifications */
  certifications?: string[];

  /** Operating hours */
  operatingHours?: string;

  /** Contact */
  contact?: {
    phone: string;
    email: string;
    website?: string;
  };

  /** Accepts drop-off */
  dropOffAvailable: boolean;

  /** Offers pickup service */
  pickupAvailable: boolean;

  /** Rating */
  rating?: number;
}

/**
 * Recycling route
 */
export interface RecyclingRoute {
  /** Route ID */
  routeId: string;

  /** Product ID */
  productId: string;

  /** Origin location */
  origin: {
    address: string;
    coordinates?: {
      latitude: number;
      longitude: number;
    };
  };

  /** Recommended facility */
  facility: RecyclingFacility;

  /** Distance (km) */
  distance: number;

  /** Estimated transport time (minutes) */
  estimatedTime?: number;

  /** Transport mode */
  transportMode?: 'road' | 'rail' | 'drop-off';

  /** Overall recovery rate (%) */
  recoveryRate: number;

  /** Material recovery breakdown */
  materialRecovery: Array<{
    material: string;
    mass: number;
    recoveryRate: number;
    valuePerKg?: number;
  }>;

  /** Estimated recovery value */
  estimatedValue?: number;

  /** Currency */
  currency?: string;

  /** Carbon footprint of transport */
  transportCarbon?: number;

  /** Alternative facilities */
  alternatives?: RecyclingFacility[];
}

// ============================================================================
// Extended Producer Responsibility (EPR)
// ============================================================================

/**
 * EPR program
 */
export interface EPRProgram {
  /** Program ID */
  programId: string;

  /** Program name */
  name: string;

  /** Manufacturer/Producer */
  producer: string;

  /** Product categories covered */
  productCategories: string[];

  /** Take-back program details */
  takebackProgram: {
    available: boolean;
    locations?: string[];
    mailIn?: boolean;
    dropOff?: boolean;
    pickup?: boolean;
  };

  /** Collection targets */
  collectionTarget?: {
    percentage: number; // % of products sold
    year: number;
  };

  /** Recycling targets */
  recyclingTarget?: {
    percentage: number; // % of collected products
    year: number;
  };

  /** Fees */
  fees?: Array<{
    productCategory: string;
    feePerUnit: number;
    currency: string;
  }>;

  /** Compliance status */
  complianceStatus?: 'compliant' | 'non-compliant' | 'partial';

  /** Reporting period */
  reportingPeriod?: {
    start: Date | string;
    end: Date | string;
  };

  /** Contact information */
  contact?: {
    email: string;
    phone: string;
    website?: string;
  };
}

/**
 * EPR compliance report
 */
export interface EPRComplianceReport {
  /** Report ID */
  reportId: string;

  /** Program reference */
  programId: string;

  /** Reporting period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Products sold */
  productsSold: {
    count: number;
    totalMass: number; // kg
    byCategory: Record<string, number>;
  };

  /** Products collected */
  productsCollected: {
    count: number;
    totalMass: number; // kg
    collectionRate: number; // Percentage
  };

  /** Products recycled */
  productsRecycled: {
    count: number;
    totalMass: number; // kg
    recyclingRate: number; // Percentage
  };

  /** Products refurbished */
  productsRefurbished?: {
    count: number;
    resoldCount: number;
  };

  /** Fees collected */
  feesCollected?: {
    total: number;
    currency: string;
  };

  /** Compliance status */
  complianceStatus: 'compliant' | 'non-compliant' | 'partial';

  /** Issues identified */
  issues?: string[];

  /** Generated timestamp */
  generatedAt: Date | string;
}

// ============================================================================
// Sharing Economy & Product-as-a-Service
// ============================================================================

/**
 * Sharing model type
 */
export type SharingModel =
  | 'rental'
  | 'subscription'
  | 'lease'
  | 'peer-to-peer'
  | 'library'
  | 'product-as-service';

/**
 * Product-as-a-Service (PaaS) registration
 */
export interface ProductAsService {
  /** PaaS ID */
  paasId: string;

  /** Product ID */
  productId: string;

  /** Service model */
  model: SharingModel;

  /** Service provider */
  provider: {
    id: string;
    name: string;
    contact?: string;
  };

  /** Pricing */
  pricing: {
    basePrice: number;
    currency: string;
    billingPeriod: 'hourly' | 'daily' | 'weekly' | 'monthly' | 'yearly';
    deposit?: number;
  };

  /** Usage tracking */
  usageTracking: {
    enabled: boolean;
    metrics?: Array<'hours' | 'cycles' | 'distance' | 'volume'>;
  };

  /** Maintenance included */
  maintenanceIncluded: boolean;

  /** Insurance included */
  insuranceIncluded?: boolean;

  /** Minimum contract period */
  minimumPeriod?: number; // days

  /** Utilization rate (%) */
  utilizationRate?: number;

  /** Active bookings */
  activeBookings?: number;

  /** Total bookings */
  totalBookings?: number;

  /** Revenue generated */
  revenueGenerated?: number;

  /** Registered timestamp */
  registeredAt: Date | string;
}

/**
 * Sharing platform booking
 */
export interface SharingBooking {
  /** Booking ID */
  bookingId: string;

  /** PaaS reference */
  paasId: string;

  /** Product ID */
  productId: string;

  /** User/Borrower */
  user: {
    id: string;
    name: string;
  };

  /** Start date/time */
  startDate: Date | string;

  /** End date/time */
  endDate: Date | string;

  /** Actual return date */
  actualReturnDate?: Date | string;

  /** Status */
  status: 'pending' | 'confirmed' | 'active' | 'completed' | 'cancelled';

  /** Condition at checkout */
  conditionCheckout?: ProductCondition;

  /** Condition at return */
  conditionReturn?: ProductCondition;

  /** Damage reported */
  damageReported?: boolean;

  /** Damage cost */
  damageCost?: number;

  /** Total cost */
  totalCost: number;

  /** Currency */
  currency: string;

  /** Payment status */
  paymentStatus: 'pending' | 'paid' | 'refunded';
}

// ============================================================================
// Waste Management
// ============================================================================

/**
 * Waste stream type
 */
export type WasteStreamType =
  | 'production-waste'
  | 'packaging-waste'
  | 'end-of-life-products'
  | 'food-waste'
  | 'hazardous-waste'
  | 'electronic-waste'
  | 'construction-waste';

/**
 * Waste treatment method
 */
export type WasteTreatmentMethod =
  | 'recycling'
  | 'composting'
  | 'anaerobic-digestion'
  | 'energy-recovery'
  | 'incineration'
  | 'landfill';

/**
 * Waste stream
 */
export interface WasteStream {
  /** Stream ID */
  streamId: string;

  /** Waste type */
  type: WasteStreamType;

  /** Source facility */
  sourceFacility: string;

  /** Material composition */
  composition: Array<{
    material: string;
    percentage: number;
  }>;

  /** Total mass */
  totalMass: number;

  /** Mass unit */
  massUnit: 'kg' | 'ton';

  /** Treatment method */
  treatment: WasteTreatmentMethod;

  /** Recovery rate (%) */
  recoveryRate?: number;

  /** Landfill diversion rate (%) */
  diversionRate?: number;

  /** Value recovered */
  valueRecovered?: number;

  /** Currency */
  currency?: string;

  /** Period */
  period: {
    start: Date | string;
    end: Date | string;
  };
}

/**
 * Waste reduction metrics
 */
export interface WasteReductionMetrics {
  /** Facility or company ID */
  entityId: string;

  /** Reporting period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Total waste generated (kg) */
  totalWasteGenerated: number;

  /** Waste recycled (kg) */
  wasteRecycled: number;

  /** Waste composted (kg) */
  wasteComposted?: number;

  /** Waste to energy (kg) */
  wasteToEnergy?: number;

  /** Waste to landfill (kg) */
  wasteToLandfill: number;

  /** Landfill diversion rate (%) */
  diversionRate: number;

  /** Zero waste certified */
  zeroWasteCertified?: boolean;

  /** Year-over-year improvement (%) */
  yoyImprovement?: number;

  /** Waste streams */
  wasteStreams: WasteStream[];
}

// ============================================================================
// Sustainability & Carbon
// ============================================================================

/**
 * Carbon footprint comparison
 */
export interface CarbonFootprint {
  /** Product ID */
  productId: string;

  /** Lifecycle stage */
  lifecycleStage?: LifecycleStage | 'complete';

  /** Virgin production carbon (kg CO2) */
  virgin: number;

  /** Circular production carbon (kg CO2) */
  circular: number;

  /** Carbon saved (kg CO2) */
  saved: number;

  /** Reduction percentage */
  reduction: number;

  /** Carbon offset value (USD) */
  offsetValue?: number;

  /** Breakdown by component */
  breakdown?: {
    materials?: number;
    manufacturing?: number;
    transport?: number;
    use?: number;
    endOfLife?: number;
  };

  /** Calculation method */
  method?: string;

  /** Calculated timestamp */
  calculatedAt: Date | string;
}

/**
 * Circular economy certification
 */
export interface CircularCertification {
  /** Certification ID */
  certificationId: string;

  /** Certification type */
  type:
    | 'cradle-to-cradle'
    | 'circular-economy-verified'
    | 'zero-waste'
    | 'eu-ecolabel'
    | 'bcorp'
    | 'other';

  /** Certification level/tier */
  level?: string; // e.g., "Gold", "Platinum"

  /** Issuing body */
  issuedBy: string;

  /** Issue date */
  issueDate: Date | string;

  /** Expiry date */
  expiryDate?: Date | string;

  /** Certificate number */
  certificateNumber: string;

  /** Product or company certified */
  entityId: string;

  /** Entity type */
  entityType: 'product' | 'company' | 'facility';

  /** Criteria met */
  criteriaMet?: string[];

  /** Score/Rating */
  score?: number;

  /** Verification document URL */
  verificationUrl?: string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * Circular Economy SDK configuration
 */
export interface CircularEconomySDKConfig {
  /** API key for authentication */
  apiKey: string;

  /** API base URL */
  apiUrl?: string;

  /** Enable blockchain features */
  enableBlockchain?: boolean;

  /** Blockchain configuration */
  blockchain?: {
    network: 'ethereum' | 'polygon' | 'hyperledger' | 'private';
    contractAddress?: string;
    privateKey?: string;
    rpcUrl?: string;
  };

  /** Default currency */
  defaultCurrency?: string;

  /** Timeout in milliseconds */
  timeout?: number;

  /** Enable debug logging */
  debug?: boolean;
}

// ============================================================================
// API Response Types
// ============================================================================

/**
 * Generic API response wrapper
 */
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    details?: any;
  };
  metadata?: {
    timestamp: string;
    requestId: string;
    version: string;
  };
}

/**
 * Paginated response
 */
export interface PaginatedResponse<T> {
  items: T[];
  pagination: {
    page: number;
    pageSize: number;
    totalItems: number;
    totalPages: number;
    hasNext: boolean;
    hasPrevious: boolean;
  };
}

// ============================================================================
// Export all types
// ============================================================================

export type {
  // Core
  LifecycleStage,
  MaterialOrigin,
  CircularityRating,
  ProductCondition,
  // Material Passport
  Material,
  CircularDesignPrinciples,
  BlockchainVerification,
  EndOfLifeInfo,
  MaterialPassport,
  // Lifecycle
  LifecycleEvent,
  RefurbishmentRecord,
  ProductLifecycle,
  // Circularity
  CircularityScoreBreakdown,
  CircularityAssessment,
  // Recycling
  RecyclingFacility,
  RecyclingRoute,
  // EPR
  EPRProgram,
  EPRComplianceReport,
  // Sharing Economy
  SharingModel,
  ProductAsService,
  SharingBooking,
  // Waste
  WasteStreamType,
  WasteTreatmentMethod,
  WasteStream,
  WasteReductionMetrics,
  // Sustainability
  CarbonFootprint,
  CircularCertification,
  // Configuration
  CircularEconomySDKConfig,
  // API
  ApiResponse,
  PaginatedResponse,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
