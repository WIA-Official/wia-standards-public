/**
 * WIA-IND-022: Inventory Management - TypeScript Type Definitions
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 */

// ============================================================================
// Core Inventory Types
// ============================================================================

export interface InventoryItem {
  sku: string;
  name: string;
  description?: string;
  category: string;
  subcategory?: string;
  warehouse: string;
  zone?: string;
  location?: string;
  quantity: number;
  unitOfMeasure: UnitOfMeasure;
  unitCost: number;
  retailPrice?: number;
  batchNumber?: string;
  lotNumber?: string;
  serialNumber?: string;
  expirationDate?: Date;
  manufacturingDate?: Date;
  reorderPoint: number;
  reorderQuantity: number;
  safetyStock: number;
  leadTime: number;
  rfidTag?: string;
  barcode?: string;
  qrCode?: string;
  status: StockStatus;
  abcCategory?: ABCCategory;
  xyzCategory?: XYZCategory;
  createdAt: Date;
  updatedAt: Date;
  createdBy: string;
  updatedBy: string;
}

export type UnitOfMeasure =
  | 'EA'  // Each
  | 'BX'  // Box
  | 'CS'  // Case
  | 'PL'  // Pallet
  | 'KG'  // Kilogram
  | 'LB'  // Pound
  | 'LT'  // Liter
  | 'GL'  // Gallon
  | 'M'   // Meter
  | 'FT'; // Foot

export type StockStatus =
  | 'AVAILABLE'
  | 'ALLOCATED'
  | 'ON_HOLD'
  | 'IN_TRANSIT'
  | 'QUARANTINE'
  | 'OBSOLETE'
  | 'DAMAGED';

export type ABCCategory = 'A' | 'B' | 'C';
export type XYZCategory = 'X' | 'Y' | 'Z';

// ============================================================================
// Transaction Types
// ============================================================================

export interface Transaction {
  transactionId: string;
  transactionType: TransactionType;
  sku: string;
  quantity: number;
  warehouse: string;
  location?: string;
  reference?: string;
  timestamp: Date;
  operator: string;
  notes?: string;
}

export type TransactionType =
  | 'RECEIPT'
  | 'SHIPMENT'
  | 'TRANSFER'
  | 'ADJUSTMENT'
  | 'CYCLE_COUNT'
  | 'RETURN'
  | 'SCRAP'
  | 'RESERVATION'
  | 'ALLOCATION';

export interface ReceiptTransaction extends Transaction {
  transactionType: 'RECEIPT';
  purchaseOrder: string;
  supplier: string;
  lotNumber?: string;
  batchNumber?: string;
  expirationDate?: Date;
  unitCost: number;
  qualityCheck?: QualityCheckResult;
}

export interface ShipmentTransaction extends Transaction {
  transactionType: 'SHIPMENT';
  salesOrder: string;
  customer: string;
  carrier?: string;
  trackingNumber?: string;
  pickLocation: string;
  pickedBy: string;
  packedBy?: string;
}

export interface TransferTransaction extends Transaction {
  transactionType: 'TRANSFER';
  transferOrder: string;
  fromWarehouse: string;
  fromLocation: string;
  toWarehouse: string;
  toLocation: string;
  reason: string;
  transferStatus: TransferStatus;
}

export interface AdjustmentTransaction extends Transaction {
  transactionType: 'ADJUSTMENT';
  previousQuantity: number;
  newQuantity: number;
  adjustmentReason: AdjustmentReason;
  rootCause?: string;
  approvedBy: string;
  financialImpact: number;
}

export type QualityCheckResult = 'PASSED' | 'FAILED' | 'PENDING';
export type TransferStatus = 'INITIATED' | 'IN_TRANSIT' | 'RECEIVED' | 'CANCELLED';

export type AdjustmentReason =
  | 'CYCLE_COUNT'
  | 'PHYSICAL_INVENTORY'
  | 'DAMAGE'
  | 'THEFT'
  | 'SYSTEM_ERROR'
  | 'EXPIRATION'
  | 'OTHER';

// ============================================================================
// Warehouse Types
// ============================================================================

export interface Warehouse {
  warehouseId: string;
  name: string;
  code: string;
  address: Address;
  type: WarehouseType;
  capacity: number;
  utilization: number;
  zones: Zone[];
  isActive: boolean;
  manager: string;
  contact: Contact;
}

export type WarehouseType =
  | 'DISTRIBUTION_CENTER'
  | 'FULFILLMENT_CENTER'
  | 'CROSS_DOCK'
  | 'COLD_STORAGE'
  | 'BONDED_WAREHOUSE'
  | 'RETAIL_STORE'
  | 'MANUFACTURING';

export interface Zone {
  zoneId: string;
  zoneName: string;
  zoneType: ZoneType;
  capacity: number;
  utilization: number;
  locations: Location[];
}

export type ZoneType =
  | 'RECEIVING'
  | 'BULK_STORAGE'
  | 'FAST_PICK'
  | 'PACKING'
  | 'SHIPPING'
  | 'QUARANTINE'
  | 'COLD_STORAGE'
  | 'HAZMAT'
  | 'HIGH_VALUE';

export interface Location {
  locationId: string;
  locationCode: string;
  warehouse: string;
  zone: string;
  aisle?: string;
  bay?: string;
  level?: string;
  bin?: string;
  capacity: number;
  currentOccupancy: number;
  locationType: LocationType;
  dimensions?: Dimensions;
}

export type LocationType =
  | 'PALLET_RACK'
  | 'BIN_SHELF'
  | 'FLOOR'
  | 'DRIVE_IN_RACK'
  | 'PUSH_BACK_RACK'
  | 'CANTILEVER_RACK'
  | 'MEZZANINE';

export interface Address {
  street: string;
  city: string;
  state: string;
  postalCode: string;
  country: string;
}

export interface Contact {
  name: string;
  email: string;
  phone: string;
}

export interface Dimensions {
  length: number;
  width: number;
  height: number;
  unit: 'IN' | 'CM' | 'FT' | 'M';
}

// ============================================================================
// Batch and Lot Tracking Types
// ============================================================================

export interface Batch {
  batchNumber: string;
  sku: string;
  productionDate: Date;
  expirationDate?: Date;
  quantity: number;
  facility: string;
  shift?: string;
  operator?: string;
  qualityChecks: QualityCheck[];
  genealogy: Genealogy;
  movements: BatchMovement[];
  status: BatchStatus;
}

export type BatchStatus = 'ACTIVE' | 'QUARANTINED' | 'RELEASED' | 'RECALLED' | 'EXPIRED';

export interface QualityCheck {
  checkType: string;
  result: QualityCheckResult;
  inspector: string;
  timestamp: Date;
  measurements?: Record<string, any>;
  notes?: string;
}

export interface Genealogy {
  finalProduct: {
    sku: string;
    batchNumber: string;
    quantity: number;
  };
  components: Component[];
  productionRecords: ProductionRecord[];
}

export interface Component {
  componentSku: string;
  componentName: string;
  batchNumber?: string;
  lotNumber?: string;
  quantity: number;
  supplier?: string;
  supplierLot?: string;
}

export interface ProductionRecord {
  operator: string;
  shift: string;
  equipment: string;
  startTime: Date;
  endTime: Date;
  yield: number;
}

export interface BatchMovement {
  movementId: string;
  action: BatchAction;
  quantity: number;
  location: string;
  operator: string;
  timestamp: Date;
  reference?: string;
}

export type BatchAction =
  | 'PRODUCTION'
  | 'RECEIPT'
  | 'PICK'
  | 'PACK'
  | 'SHIP'
  | 'TRANSFER'
  | 'QUARANTINE'
  | 'RELEASE'
  | 'SCRAP';

// ============================================================================
// Demand Forecasting Types
// ============================================================================

export interface ForecastRequest {
  sku: string;
  historicalData: Record<string, number>; // { 'YYYY-MM': quantity }
  forecastMonths: number;
  method: ForecastMethod;
  seasonalIndices?: number[];
  confidenceLevel?: number;
}

export type ForecastMethod =
  | 'MOVING_AVERAGE'
  | 'WEIGHTED_AVERAGE'
  | 'EXPONENTIAL_SMOOTHING'
  | 'LINEAR_REGRESSION'
  | 'SEASONAL_DECOMPOSITION'
  | 'ARIMA'
  | 'MACHINE_LEARNING';

export interface ForecastResult {
  sku: string;
  method: ForecastMethod;
  predictions: ForecastPrediction[];
  accuracy: ForecastAccuracy;
  confidence: number;
  generatedAt: Date;
}

export interface ForecastPrediction {
  month: string;
  quantity: number;
  confidence: number;
  lowerBound?: number;
  upperBound?: number;
}

export interface ForecastAccuracy {
  mad: number;  // Mean Absolute Deviation
  mape: number; // Mean Absolute Percentage Error
  rmse: number; // Root Mean Square Error
  trackingSignal: number;
}

// ============================================================================
// EOQ and Reorder Point Types
// ============================================================================

export interface EOQCalculation {
  sku: string;
  annualDemand: number;
  orderCost: number;
  holdingCostPerUnit: number;
  orderQuantity: number;
  ordersPerYear: number;
  totalCost: number;
  orderingCost: number;
  holdingCost: number;
  daysBetweenOrders: number;
}

export interface ReorderPointCalculation {
  sku: string;
  averageDailyDemand: number;
  leadTime: number;
  safetyStock: number;
  reorderPoint: number;
  serviceLevel: number;
  demandStdDev?: number;
  leadTimeStdDev?: number;
}

export interface SafetyStockCalculation {
  sku: string;
  serviceLevel: number;
  zScore: number;
  demandStdDev: number;
  leadTime: number;
  safetyStock: number;
  expectedStockoutRate: number;
}

// ============================================================================
// Inventory Valuation Types
// ============================================================================

export type ValuationMethod = 'FIFO' | 'LIFO' | 'WEIGHTED_AVERAGE' | 'SPECIFIC_IDENTIFICATION';

export interface InventoryValuation {
  sku: string;
  method: ValuationMethod;
  quantity: number;
  totalValue: number;
  averageCost: number;
  layers?: InventoryLayer[];
  calculatedAt: Date;
}

export interface InventoryLayer {
  purchaseDate: Date;
  quantity: number;
  unitCost: number;
  totalCost: number;
  reference: string;
}

// ============================================================================
// Dead Stock Types
// ============================================================================

export interface DeadStockAnalysis {
  items: DeadStockItem[];
  totalValue: number;
  totalUnits: number;
  categoryBreakdown: Record<ABCCategory, number>;
  generatedAt: Date;
}

export interface DeadStockItem {
  sku: string;
  name: string;
  quantity: number;
  unitCost: number;
  totalValue: number;
  lastSaleDate?: Date;
  daysSinceLastSale: number;
  averageMonthlyDemand: number;
  category: ABCCategory;
  recommendedAction: DeadStockAction;
  reason: string;
}

export type DeadStockAction =
  | 'DISCOUNT'
  | 'LIQUIDATE'
  | 'RETURN_TO_SUPPLIER'
  | 'DONATE'
  | 'REPURPOSE'
  | 'SCRAP';

// ============================================================================
// Cycle Counting Types
// ============================================================================

export interface CycleCountPlan {
  countId: string;
  countDate: Date;
  countType: CycleCountType;
  locations: string[];
  skuList: string[];
  assignedTo: string;
  expectedDuration: number;
  priority: Priority;
  status: CycleCountStatus;
}

export type CycleCountType =
  | 'ABC_A_ITEMS'
  | 'ABC_B_ITEMS'
  | 'ABC_C_ITEMS'
  | 'RANDOM_SAMPLE'
  | 'ZONE_COUNT'
  | 'LOW_STOCK'
  | 'ZERO_STOCK'
  | 'CONTROL_GROUP';

export type Priority = 'LOW' | 'NORMAL' | 'HIGH' | 'CRITICAL';
export type CycleCountStatus = 'PLANNED' | 'IN_PROGRESS' | 'COMPLETED' | 'CANCELLED';

export interface CycleCountRecord {
  countId: string;
  sku: string;
  location: string;
  systemQuantity: number;
  countedQuantity: number;
  variance: number;
  variancePercent: number;
  countedBy: string;
  countTime: Date;
  notes?: string;
  requiresRecount: boolean;
  approved: boolean;
  approvedBy?: string;
}

export interface CycleCountAccuracy {
  period: string;
  totalCounts: number;
  accuracy: number;
  locationAccuracy: number;
  financialAccuracy: number;
  trend: 'UP' | 'DOWN' | 'STABLE';
  rootCauses: RootCauseAnalysis[];
}

export interface RootCauseAnalysis {
  cause: string;
  frequency: number;
  percentOfTotal: number;
  cumulativePercent: number;
}

// ============================================================================
// RFID and Barcode Types
// ============================================================================

export interface ScanEvent {
  timestamp: Date;
  scanner: string;
  operator: string;
  location: string;
  barcodeType?: BarcodeType;
  barcodeData?: string;
  rfidTag?: string;
  decodedData: DecodedData;
  action: ScanAction;
  quantity: number;
  transactionId: string;
}

export type BarcodeType =
  | 'UPC_A'
  | 'UPC_E'
  | 'EAN_13'
  | 'EAN_8'
  | 'CODE_39'
  | 'CODE_128'
  | 'QR_CODE'
  | 'DATA_MATRIX'
  | 'GS1_128';

export type ScanAction =
  | 'RECEIVE'
  | 'PICK'
  | 'PACK'
  | 'SHIP'
  | 'COUNT'
  | 'TRANSFER'
  | 'VERIFY';

export interface DecodedData {
  sku: string;
  productName?: string;
  uom: UnitOfMeasure;
  batchNumber?: string;
  expirationDate?: Date;
  serialNumber?: string;
}

export interface RFIDTag {
  epc: string; // Electronic Product Code
  tagType: RFIDTagType;
  encodedData: RFIDEncodedData;
  readEvents: RFIDReadEvent[];
}

export type RFIDTagType =
  | 'PASSIVE_UHF'
  | 'PASSIVE_HF'
  | 'ACTIVE'
  | 'BAP'; // Battery-Assisted Passive

export interface RFIDEncodedData {
  sku: string;
  serialNumber: string;
  manufacturingDate?: Date;
  expirationDate?: Date;
  batchNumber?: string;
  customData?: Record<string, any>;
}

export interface RFIDReadEvent {
  timestamp: Date;
  reader: string;
  location: string;
  rssi: number; // Received Signal Strength Indicator
  antenna: number;
}

// ============================================================================
// Performance Metrics Types
// ============================================================================

export interface InventoryMetrics {
  period: string;
  inventoryTurnover: number;
  daysInventoryOutstanding: number;
  inventoryAccuracy: number;
  orderFillRate: number;
  perfectOrderRate: number;
  carryingCostPercent: number;
  stockoutRate: number;
  backorderRate: number;
  returnRate: number;
  gmroi: number; // Gross Margin Return on Investment
  deadStockRatio: number;
  shrinkageRate: number;
  damageRate: number;
}

export interface WarehouseMetrics {
  period: string;
  orderPickAccuracy: number;
  unitsPickedPerHour: number;
  dockToStockTime: number;
  orderCycleTime: number;
  capacityUtilization: number;
  putawayAccuracy: number;
  costPerOrder: number;
  costPerUnitStored: number;
  laborCostPerOrder: number;
}

// ============================================================================
// Configuration Types
// ============================================================================

export interface InventoryManagerConfig {
  warehouses: string[];
  defaultValuationMethod: ValuationMethod;
  enableRFID?: boolean;
  enableDemandForecasting?: boolean;
  enableBatchTracking?: boolean;
  cycleCountFrequency?: Record<ABCCategory, number>;
  alertThresholds?: AlertThresholds;
  integrations?: IntegrationConfig;
}

export interface AlertThresholds {
  lowStock: number;
  criticalStock: number;
  expirationWarning: number; // days
  expirationCritical: number; // days
  deadStockDays: number;
  maxVariancePercent: number;
}

export interface IntegrationConfig {
  erp?: {
    enabled: boolean;
    endpoint: string;
    apiKey: string;
  };
  wms?: {
    enabled: boolean;
    endpoint: string;
    apiKey: string;
  };
  ecommerce?: {
    enabled: boolean;
    endpoint: string;
    apiKey: string;
  };
}

// ============================================================================
// Report Types
// ============================================================================

export interface InventoryReport {
  reportType: ReportType;
  generatedAt: Date;
  generatedBy: string;
  parameters: Record<string, any>;
  data: any;
  summary: ReportSummary;
}

export type ReportType =
  | 'INVENTORY_VALUATION'
  | 'DEAD_STOCK'
  | 'EXPIRING_ITEMS'
  | 'ABC_ANALYSIS'
  | 'STOCK_LEVEL'
  | 'TURNOVER_ANALYSIS'
  | 'CYCLE_COUNT_ACCURACY'
  | 'TRANSACTION_HISTORY'
  | 'BATCH_TRACEABILITY';

export interface ReportSummary {
  totalRecords: number;
  totalValue?: number;
  averageValue?: number;
  keyFindings: string[];
  recommendations?: string[];
}

// ============================================================================
// Utility Types
// ============================================================================

export interface PaginationParams {
  page: number;
  pageSize: number;
  sortBy?: string;
  sortOrder?: 'ASC' | 'DESC';
}

export interface PaginatedResponse<T> {
  data: T[];
  totalRecords: number;
  totalPages: number;
  currentPage: number;
  pageSize: number;
}

export interface OperationResult {
  success: boolean;
  message: string;
  data?: any;
  errors?: string[];
}

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
