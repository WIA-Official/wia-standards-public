/**
 * WIA-IND-023: Supply Chain Standard - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Supply Chain Types
// ============================================================================

/**
 * Geographic coordinates for location tracking
 */
export interface GeoLocation {
  latitude: number;
  longitude: number;
  altitude?: number;
  accuracy?: number;
}

/**
 * Physical address structure
 */
export interface Address {
  street: string;
  city: string;
  state?: string;
  postalCode: string;
  country: string;
  countryCode: string; // ISO 3166-1 alpha-2
  coordinates?: GeoLocation;
}

/**
 * Contact information
 */
export interface ContactInfo {
  name: string;
  email: string;
  phone: string;
  role?: string;
  isPrimary?: boolean;
}

// ============================================================================
// Supplier Management
// ============================================================================

/**
 * Supplier tier levels in the supply chain
 */
export type SupplierTier = 1 | 2 | 3 | 4 | 5;

/**
 * Supplier certification types
 */
export type SupplierCertification =
  | 'ISO9001'
  | 'ISO14001'
  | 'ISO45001'
  | 'ISO27001'
  | 'IATF16949'
  | 'AS9100'
  | 'FDA'
  | 'GMP'
  | 'HACCP'
  | 'FSC'
  | 'FAIRTRADE'
  | 'BSCI';

/**
 * Supplier status
 */
export type SupplierStatus = 'active' | 'inactive' | 'pending' | 'suspended' | 'blacklisted';

/**
 * Supplier profile
 */
export interface Supplier {
  /** Unique supplier identifier */
  id: string;

  /** Supplier name */
  name: string;

  /** Legal entity name */
  legalName?: string;

  /** Supplier tier (1 = direct, 2+ = indirect) */
  tier: SupplierTier;

  /** Supplier status */
  status: SupplierStatus;

  /** Primary address */
  address: Address;

  /** Contact information */
  contacts: ContactInfo[];

  /** Performance rating (0-5) */
  rating: number;

  /** Certifications */
  certifications: SupplierCertification[];

  /** Financial stability score (0-100) */
  financialScore?: number;

  /** ESG score (0-100) */
  esgScore?: number;

  /** Risk score (0-100, lower is better) */
  riskScore?: number;

  /** Approved categories */
  categories: string[];

  /** Payment terms (e.g., "NET30", "NET60") */
  paymentTerms: string;

  /** Lead time in days */
  leadTimeDays: number;

  /** Minimum order quantity */
  moq?: number;

  /** Currency code (ISO 4217) */
  currency: string;

  /** Contract start date */
  contractStart?: Date | string;

  /** Contract end date */
  contractEnd?: Date | string;

  /** Created timestamp */
  createdAt: Date | string;

  /** Last updated timestamp */
  updatedAt: Date | string;
}

/**
 * Supplier performance metrics
 */
export interface SupplierPerformance {
  supplierId: string;
  period: {
    start: Date | string;
    end: Date | string;
  };
  metrics: {
    onTimeDeliveryRate: number; // Percentage
    qualityAcceptanceRate: number; // Percentage
    responseTime: number; // Hours
    defectRate: number; // PPM (parts per million)
    fillRate: number; // Percentage
    costVariance: number; // Percentage
  };
  totalOrders: number;
  totalValue: number;
  currency: string;
}

// ============================================================================
// Procurement & Orders
// ============================================================================

/**
 * Purchase order status
 */
export type OrderStatus =
  | 'draft'
  | 'pending_approval'
  | 'approved'
  | 'sent'
  | 'acknowledged'
  | 'in_production'
  | 'ready_to_ship'
  | 'shipped'
  | 'delivered'
  | 'completed'
  | 'cancelled'
  | 'disputed';

/**
 * Order line item
 */
export interface OrderLineItem {
  /** Line item number */
  lineNumber: number;

  /** SKU or part number */
  sku: string;

  /** Item description */
  description: string;

  /** Quantity ordered */
  quantity: number;

  /** Unit of measure */
  uom: string; // e.g., "EA", "KG", "M"

  /** Unit price */
  unitPrice: number;

  /** Line total (quantity × unitPrice) */
  lineTotal: number;

  /** Requested delivery date */
  requestedDate?: Date | string;

  /** Confirmed delivery date */
  confirmedDate?: Date | string;

  /** Tax amount */
  tax?: number;

  /** Discount amount */
  discount?: number;

  /** Product category */
  category?: string;

  /** HS code for customs */
  hsCode?: string;

  /** Country of origin */
  countryOfOrigin?: string;

  /** Blockchain hash for traceability */
  blockchainHash?: string;
}

/**
 * Purchase order
 */
export interface PurchaseOrder {
  /** Purchase order ID */
  id: string;

  /** PO number (human-readable) */
  poNumber: string;

  /** Supplier information */
  supplier: {
    id: string;
    name: string;
    address: Address;
  };

  /** Buyer information */
  buyer: {
    id: string;
    name: string;
    address: Address;
  };

  /** Order status */
  status: OrderStatus;

  /** Line items */
  items: OrderLineItem[];

  /** Subtotal */
  subtotal: number;

  /** Tax total */
  tax: number;

  /** Shipping cost */
  shipping: number;

  /** Grand total */
  total: number;

  /** Currency */
  currency: string;

  /** Payment terms */
  paymentTerms: string;

  /** Delivery address */
  deliveryAddress: Address;

  /** Requested delivery date */
  requestedDeliveryDate: Date | string;

  /** Confirmed delivery date */
  confirmedDeliveryDate?: Date | string;

  /** Incoterms (e.g., "FOB", "CIF", "DDP") */
  incoterms?: string;

  /** Special instructions */
  notes?: string;

  /** Created by user */
  createdBy: string;

  /** Approved by user */
  approvedBy?: string;

  /** Created timestamp */
  createdAt: Date | string;

  /** Updated timestamp */
  updatedAt: Date | string;

  /** Related shipment IDs */
  shipmentIds?: string[];
}

// ============================================================================
// Logistics & Shipment Tracking
// ============================================================================

/**
 * Transportation mode
 */
export type TransportMode = 'air' | 'sea' | 'road' | 'rail' | 'multimodal';

/**
 * Shipment status
 */
export type ShipmentStatus =
  | 'pending'
  | 'picked_up'
  | 'in_transit'
  | 'customs_clearance'
  | 'out_for_delivery'
  | 'delivered'
  | 'exception'
  | 'returned'
  | 'cancelled';

/**
 * Tracking event
 */
export interface TrackingEvent {
  /** Event timestamp */
  timestamp: Date | string;

  /** Event type */
  type: ShipmentStatus;

  /** Location */
  location: {
    name: string;
    city?: string;
    country?: string;
    coordinates?: GeoLocation;
  };

  /** Event description */
  description: string;

  /** Handled by (carrier, warehouse, etc.) */
  handledBy?: string;

  /** Temperature reading (for cold chain) */
  temperature?: number;

  /** Humidity reading */
  humidity?: number;

  /** Damage noted */
  damageNoted?: boolean;

  /** Notes */
  notes?: string;
}

/**
 * Shipment details
 */
export interface Shipment {
  /** Shipment ID */
  id: string;

  /** Tracking number */
  trackingNumber: string;

  /** Purchase order reference */
  purchaseOrderId: string;

  /** Carrier */
  carrier: {
    id: string;
    name: string;
    service: string; // e.g., "Express", "Standard"
  };

  /** Transportation mode */
  mode: TransportMode;

  /** Current status */
  status: ShipmentStatus;

  /** Origin */
  origin: {
    address: Address;
    departureDate?: Date | string;
  };

  /** Destination */
  destination: {
    address: Address;
    arrivalDate?: Date | string;
  };

  /** Current location */
  currentLocation?: {
    name: string;
    coordinates?: GeoLocation;
    timestamp: Date | string;
  };

  /** Estimated time of arrival */
  eta?: Date | string;

  /** Actual delivery time */
  actualDelivery?: Date | string;

  /** Tracking events */
  events: TrackingEvent[];

  /** Package details */
  packages: Array<{
    packageId: string;
    weight: number;
    weightUnit: string; // "kg", "lb"
    dimensions: {
      length: number;
      width: number;
      height: number;
      unit: string; // "cm", "in"
    };
    contents: string;
  }>;

  /** Customs information */
  customs?: {
    declarationNumber: string;
    value: number;
    currency: string;
    cleared: boolean;
    clearedDate?: Date | string;
  };

  /** Insurance */
  insurance?: {
    provider: string;
    value: number;
    currency: string;
    policyNumber: string;
  };

  /** Temperature control (cold chain) */
  temperatureControl?: {
    required: boolean;
    minTemp: number;
    maxTemp: number;
    unit: 'C' | 'F';
    currentTemp?: number;
  };

  /** Created timestamp */
  createdAt: Date | string;

  /** Updated timestamp */
  updatedAt: Date | string;
}

// ============================================================================
// Blockchain Traceability
// ============================================================================

/**
 * Blockchain network
 */
export type BlockchainNetwork = 'ethereum' | 'polygon' | 'hyperledger' | 'private';

/**
 * Blockchain checkpoint in product journey
 */
export interface BlockchainCheckpoint {
  /** Checkpoint ID */
  id: string;

  /** Stage in journey */
  stage:
    | 'origin'
    | 'manufacturing'
    | 'quality_check'
    | 'packaging'
    | 'warehouse'
    | 'shipping'
    | 'distribution'
    | 'retail'
    | 'consumer';

  /** Timestamp */
  timestamp: Date | string;

  /** Location */
  location: {
    name: string;
    address?: Address;
    coordinates?: GeoLocation;
  };

  /** Blockchain transaction hash */
  txHash: string;

  /** Block number */
  blockNumber: number;

  /** Data recorded */
  data: Record<string, any>;

  /** Verified by */
  verifiedBy?: string;

  /** Certification references */
  certifications?: string[];
}

/**
 * Product provenance record
 */
export interface ProductProvenance {
  /** Product SKU */
  sku: string;

  /** Serial number or batch ID */
  serialNumber: string;

  /** Blockchain network */
  network: BlockchainNetwork;

  /** Smart contract address */
  contractAddress: string;

  /** NFT token ID (if applicable) */
  tokenId?: string;

  /** Origin information */
  origin: {
    manufacturer: string;
    location: Address;
    date: Date | string;
  };

  /** Product journey checkpoints */
  journey: BlockchainCheckpoint[];

  /** Current owner */
  currentOwner?: string;

  /** Authenticity verified */
  isAuthentic: boolean;

  /** Verification timestamp */
  verifiedAt: Date | string;

  /** Total carbon footprint */
  carbonFootprint?: number;

  /** Sustainability certifications */
  sustainabilityCerts?: string[];
}

// ============================================================================
// Risk Management
// ============================================================================

/**
 * Risk level
 */
export type RiskLevel = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL';

/**
 * Risk category
 */
export type RiskCategory =
  | 'supplier'
  | 'logistics'
  | 'demand'
  | 'compliance'
  | 'financial'
  | 'geopolitical'
  | 'quality'
  | 'environmental';

/**
 * Risk factor
 */
export interface RiskFactor {
  /** Factor name */
  name: string;

  /** Category */
  category: RiskCategory;

  /** Weight (0-1) */
  weight: number;

  /** Current value (0-100) */
  value: number;

  /** Description */
  description: string;

  /** Trend (improving/declining) */
  trend?: 'improving' | 'stable' | 'declining';
}

/**
 * Risk assessment
 */
export interface RiskAssessment {
  /** Entity ID (supplier, product, route, etc.) */
  entityId: string;

  /** Entity type */
  entityType: 'supplier' | 'product' | 'route' | 'country';

  /** Overall risk score (0-100) */
  riskScore: number;

  /** Risk level */
  riskLevel: RiskLevel;

  /** Risk factors */
  factors: RiskFactor[];

  /** Mitigation recommendations */
  mitigations: Array<{
    priority: 'high' | 'medium' | 'low';
    action: string;
    estimatedCost?: number;
    timeline?: string;
  }>;

  /** Alternative options */
  alternatives?: Array<{
    entityId: string;
    name: string;
    riskScore: number;
    costDelta: number; // Percentage difference
    leadTimeDelta: number; // Days difference
  }>;

  /** Assessment timestamp */
  assessedAt: Date | string;

  /** Valid until */
  validUntil?: Date | string;
}

// ============================================================================
// Demand Planning & Forecasting
// ============================================================================

/**
 * Forecast method
 */
export type ForecastMethod =
  | 'moving_average'
  | 'exponential_smoothing'
  | 'arima'
  | 'prophet'
  | 'ml_model';

/**
 * Demand forecast
 */
export interface DemandForecast {
  /** Product SKU */
  sku: string;

  /** Forecast period */
  period: {
    start: Date | string;
    end: Date | string;
    interval: 'daily' | 'weekly' | 'monthly' | 'quarterly';
  };

  /** Forecast method used */
  method: ForecastMethod;

  /** Forecasted quantities by date */
  forecast: Array<{
    date: Date | string;
    quantity: number;
    confidence: number; // 0-1
    upperBound: number;
    lowerBound: number;
  }>;

  /** Historical accuracy */
  accuracy?: {
    mape: number; // Mean Absolute Percentage Error
    rmse: number; // Root Mean Squared Error
  };

  /** Factors considered */
  factors?: {
    seasonality: boolean;
    trends: boolean;
    promotions: boolean;
    externalEvents: boolean;
  };

  /** Generated timestamp */
  generatedAt: Date | string;
}

/**
 * Inventory recommendation
 */
export interface InventoryRecommendation {
  /** Product SKU */
  sku: string;

  /** Current stock level */
  currentStock: number;

  /** Safety stock */
  safetyStock: number;

  /** Reorder point */
  reorderPoint: number;

  /** Recommended order quantity */
  orderQuantity: number;

  /** Economic order quantity (EOQ) */
  economicOrderQuantity: number;

  /** Projected stockout date */
  stockoutDate?: Date | string;

  /** Days of supply remaining */
  daysOfSupply: number;

  /** Turnover rate */
  turnoverRate: number;

  /** Carrying cost */
  carryingCost?: number;

  /** Recommendations */
  recommendations: string[];
}

// ============================================================================
// Sustainability & ESG
// ============================================================================

/**
 * Carbon footprint calculation
 */
export interface CarbonFootprint {
  /** Entity ID (shipment, product, supplier) */
  entityId: string;

  /** Entity type */
  entityType: 'shipment' | 'product' | 'supplier' | 'facility';

  /** Total CO2 emissions (kg) */
  totalKg: number;

  /** Breakdown by source */
  breakdown: {
    transportation?: number;
    manufacturing?: number;
    packaging?: number;
    storage?: number;
    other?: number;
  };

  /** Emissions per unit */
  perUnit?: number;

  /** Offset cost (USD) */
  offsetCost?: number;

  /** Carbon credits purchased */
  creditsRetired?: number;

  /** Calculation method */
  method: string;

  /** Calculated timestamp */
  calculatedAt: Date | string;
}

/**
 * ESG (Environmental, Social, Governance) metrics
 */
export interface ESGMetrics {
  /** Entity ID */
  entityId: string;

  /** Overall ESG score (0-100) */
  overallScore: number;

  /** Environmental score */
  environmental: {
    score: number;
    carbonFootprint: number;
    renewableEnergy: number; // Percentage
    wasteReduction: number; // Percentage
    waterUsage: number; // Liters per unit
    recycledMaterials: number; // Percentage
  };

  /** Social score */
  social: {
    score: number;
    laborPractices: number; // 0-100
    safetyIncidents: number;
    diversityScore: number; // 0-100
    communityImpact: number; // 0-100
  };

  /** Governance score */
  governance: {
    score: number;
    compliance: number; // 0-100
    ethicsScore: number; // 0-100
    transparency: number; // 0-100
    certifications: string[];
  };

  /** Assessment period */
  period: {
    start: Date | string;
    end: Date | string;
  };

  /** Assessed timestamp */
  assessedAt: Date | string;
}

// ============================================================================
// SDK Configuration
// ============================================================================

/**
 * Supply chain SDK configuration
 */
export interface SupplyChainSDKConfig {
  /** API key for authentication */
  apiKey: string;

  /** API base URL */
  apiUrl?: string;

  /** Blockchain configuration */
  blockchain?: {
    network: BlockchainNetwork;
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
  GeoLocation,
  Address,
  ContactInfo,
  // Supplier
  SupplierTier,
  SupplierCertification,
  SupplierStatus,
  Supplier,
  SupplierPerformance,
  // Orders
  OrderStatus,
  OrderLineItem,
  PurchaseOrder,
  // Logistics
  TransportMode,
  ShipmentStatus,
  TrackingEvent,
  Shipment,
  // Blockchain
  BlockchainNetwork,
  BlockchainCheckpoint,
  ProductProvenance,
  // Risk
  RiskLevel,
  RiskCategory,
  RiskFactor,
  RiskAssessment,
  // Forecasting
  ForecastMethod,
  DemandForecast,
  InventoryRecommendation,
  // Sustainability
  CarbonFootprint,
  ESGMetrics,
  // Configuration
  SupplyChainSDKConfig,
  // API
  ApiResponse,
  PaginatedResponse,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
