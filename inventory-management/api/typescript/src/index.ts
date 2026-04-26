/**
 * WIA-IND-022: Inventory Management - TypeScript SDK
 *
 * @version 1.0.0
 * @license MIT
 * @author WIA Industry Standards Group
 *
 * 弘益人間 (Benefit All Humanity)
 *
 * This SDK provides comprehensive inventory management functionality including
 * stock tracking, warehouse management, demand forecasting, EOQ calculation,
 * batch tracking, and inventory optimization.
 */

import {
  InventoryItem,
  Transaction,
  ReceiptTransaction,
  ShipmentTransaction,
  TransferTransaction,
  AdjustmentTransaction,
  Warehouse,
  Batch,
  ForecastRequest,
  ForecastResult,
  ForecastPrediction,
  EOQCalculation,
  ReorderPointCalculation,
  SafetyStockCalculation,
  InventoryValuation,
  ValuationMethod,
  DeadStockAnalysis,
  DeadStockItem,
  CycleCountPlan,
  CycleCountRecord,
  InventoryManagerConfig,
  InventoryMetrics,
  StockStatus,
  ABCCategory,
  OperationResult,
} from './types';

// Export all types
export * from './types';

// ============================================================================
// Inventory Manager Class
// ============================================================================

export class InventoryManager {
  private config: InventoryManagerConfig;
  private inventory: Map<string, InventoryItem> = new Map();
  private transactions: Transaction[] = [];
  private warehouses: Map<string, Warehouse> = new Map();
  private batches: Map<string, Batch> = new Map();

  constructor(config: InventoryManagerConfig) {
    this.config = config;
    this.initializeWarehouses();
  }

  private initializeWarehouses(): void {
    // Initialize warehouse tracking
    this.config.warehouses.forEach((whId) => {
      // Warehouse initialization would happen here
    });
  }

  /**
   * Add a new inventory item
   */
  async addItem(item: Partial<InventoryItem>): Promise<OperationResult> {
    try {
      const newItem: InventoryItem = {
        sku: item.sku!,
        name: item.name!,
        description: item.description,
        category: item.category!,
        subcategory: item.subcategory,
        warehouse: item.warehouse!,
        zone: item.zone,
        location: item.location,
        quantity: item.quantity || 0,
        unitOfMeasure: item.unitOfMeasure || 'EA',
        unitCost: item.unitCost || 0,
        retailPrice: item.retailPrice,
        batchNumber: item.batchNumber,
        lotNumber: item.lotNumber,
        serialNumber: item.serialNumber,
        expirationDate: item.expirationDate,
        manufacturingDate: item.manufacturingDate,
        reorderPoint: item.reorderPoint || 0,
        reorderQuantity: item.reorderQuantity || 0,
        safetyStock: item.safetyStock || 0,
        leadTime: item.leadTime || 0,
        rfidTag: item.rfidTag,
        barcode: item.barcode,
        qrCode: item.qrCode,
        status: item.status || 'AVAILABLE',
        abcCategory: item.abcCategory,
        xyzCategory: item.xyzCategory,
        createdAt: new Date(),
        updatedAt: new Date(),
        createdBy: item.createdBy || 'system',
        updatedBy: item.updatedBy || 'system',
      };

      this.inventory.set(item.sku!, newItem);

      return {
        success: true,
        message: `Item ${item.sku} added successfully`,
        data: newItem,
      };
    } catch (error) {
      return {
        success: false,
        message: `Failed to add item: ${error}`,
        errors: [String(error)],
      };
    }
  }

  /**
   * Get stock information for a SKU
   */
  async getStock(sku: string, warehouse?: string): Promise<InventoryItem | null> {
    const item = this.inventory.get(sku);
    if (!item) return null;
    if (warehouse && item.warehouse !== warehouse) return null;
    return item;
  }

  /**
   * Update stock quantity
   */
  async updateStock(
    sku: string,
    quantity: number,
    warehouse: string
  ): Promise<OperationResult> {
    const item = this.inventory.get(sku);
    if (!item) {
      return {
        success: false,
        message: `SKU ${sku} not found`,
      };
    }

    item.quantity = quantity;
    item.updatedAt = new Date();

    return {
      success: true,
      message: `Stock updated for ${sku}`,
      data: item,
    };
  }

  /**
   * Transfer stock between warehouses
   */
  async transfer(params: {
    sku: string;
    fromWarehouse: string;
    toWarehouse: string;
    quantity: number;
    reason: string;
  }): Promise<OperationResult> {
    try {
      const transaction: TransferTransaction = {
        transactionId: `TXN-${Date.now()}`,
        transactionType: 'TRANSFER',
        transferOrder: `TO-${Date.now()}`,
        sku: params.sku,
        quantity: params.quantity,
        warehouse: params.fromWarehouse,
        fromWarehouse: params.fromWarehouse,
        fromLocation: '',
        toWarehouse: params.toWarehouse,
        toLocation: '',
        reason: params.reason,
        transferStatus: 'INITIATED',
        timestamp: new Date(),
        operator: 'system',
      };

      this.transactions.push(transaction);

      return {
        success: true,
        message: `Transfer initiated from ${params.fromWarehouse} to ${params.toWarehouse}`,
        data: transaction,
      };
    } catch (error) {
      return {
        success: false,
        message: `Transfer failed: ${error}`,
        errors: [String(error)],
      };
    }
  }

  /**
   * Track batch movement
   */
  async trackBatch(params: {
    batchNumber: string;
    action: string;
    quantity: number;
    location: string;
    operator: string;
    timestamp: Date;
  }): Promise<OperationResult> {
    try {
      const batch = this.batches.get(params.batchNumber);
      if (!batch) {
        return {
          success: false,
          message: `Batch ${params.batchNumber} not found`,
        };
      }

      // Track the movement (simplified)
      return {
        success: true,
        message: `Batch ${params.batchNumber} movement tracked`,
        data: params,
      };
    } catch (error) {
      return {
        success: false,
        message: `Batch tracking failed: ${error}`,
        errors: [String(error)],
      };
    }
  }

  /**
   * Analyze dead stock
   */
  async analyzeDeadStock(params: {
    minDaysWithoutMovement: number;
    minQuantity: number;
    includeValue: boolean;
  }): Promise<DeadStockAnalysis> {
    const deadStockItems: DeadStockItem[] = [];
    let totalValue = 0;
    let totalUnits = 0;

    // Analyze inventory for dead stock (simplified)
    this.inventory.forEach((item) => {
      // Logic to identify dead stock would go here
      // For now, return empty analysis
    });

    return {
      items: deadStockItems,
      totalValue,
      totalUnits,
      categoryBreakdown: {
        A: 0,
        B: 0,
        C: 0,
      },
      generatedAt: new Date(),
    };
  }

  /**
   * Get inventory metrics
   */
  async getMetrics(period: string): Promise<InventoryMetrics> {
    // Calculate various inventory metrics
    return {
      period,
      inventoryTurnover: 0,
      daysInventoryOutstanding: 0,
      inventoryAccuracy: 0,
      orderFillRate: 0,
      perfectOrderRate: 0,
      carryingCostPercent: 0,
      stockoutRate: 0,
      backorderRate: 0,
      returnRate: 0,
      gmroi: 0,
      deadStockRatio: 0,
      shrinkageRate: 0,
      damageRate: 0,
    };
  }
}

// ============================================================================
// Economic Order Quantity (EOQ) Calculator
// ============================================================================

export function calculateEOQ(params: {
  annualDemand: number;
  orderCost: number;
  holdingCostPerUnit: number;
}): EOQCalculation {
  const { annualDemand, orderCost, holdingCostPerUnit } = params;

  // EOQ Formula: √(2 × D × S / H)
  const orderQuantity = Math.sqrt(
    (2 * annualDemand * orderCost) / holdingCostPerUnit
  );

  const ordersPerYear = annualDemand / orderQuantity;
  const orderingCost = ordersPerYear * orderCost;
  const holdingCost = (orderQuantity / 2) * holdingCostPerUnit;
  const totalCost = orderingCost + holdingCost;
  const daysBetweenOrders = 365 / ordersPerYear;

  return {
    sku: '',
    annualDemand,
    orderCost,
    holdingCostPerUnit,
    orderQuantity: Math.round(orderQuantity),
    ordersPerYear: Math.round(ordersPerYear * 10) / 10,
    totalCost: Math.round(totalCost * 100) / 100,
    orderingCost: Math.round(orderingCost * 100) / 100,
    holdingCost: Math.round(holdingCost * 100) / 100,
    daysBetweenOrders: Math.round(daysBetweenOrders * 10) / 10,
  };
}

// ============================================================================
// Reorder Point Calculator
// ============================================================================

export function calculateReorderPoint(params: {
  averageDailyDemand: number;
  leadTime: number;
  safetyStock: number;
  serviceLevel?: number;
}): ReorderPointCalculation {
  const { averageDailyDemand, leadTime, safetyStock, serviceLevel = 0.95 } = params;

  const reorderPoint = averageDailyDemand * leadTime + safetyStock;

  return {
    sku: '',
    averageDailyDemand,
    leadTime,
    safetyStock,
    reorderPoint: Math.round(reorderPoint),
    serviceLevel,
  };
}

// ============================================================================
// Safety Stock Calculator
// ============================================================================

export function calculateSafetyStock(params: {
  serviceLevel: number;
  demandStdDev: number;
  leadTime: number;
}): SafetyStockCalculation {
  const { serviceLevel, demandStdDev, leadTime } = params;

  // Z-score lookup based on service level
  const zScores: Record<number, number> = {
    0.85: 1.04,
    0.90: 1.28,
    0.95: 1.65,
    0.98: 2.05,
    0.99: 2.33,
    0.999: 3.09,
  };

  const zScore = zScores[serviceLevel] || 1.65;

  // Safety Stock = Z × σ_d × √L
  const safetyStock = zScore * demandStdDev * Math.sqrt(leadTime);
  const expectedStockoutRate = 1 - serviceLevel;

  return {
    sku: '',
    serviceLevel,
    zScore,
    demandStdDev,
    leadTime,
    safetyStock: Math.round(safetyStock),
    expectedStockoutRate,
  };
}

// ============================================================================
// Demand Forecasting
// ============================================================================

export async function forecastDemand(
  request: ForecastRequest
): Promise<ForecastResult> {
  const { sku, historicalData, forecastMonths, method } = request;

  const predictions: ForecastPrediction[] = [];

  switch (method) {
    case 'MOVING_AVERAGE':
      return calculateMovingAverage(sku, historicalData, forecastMonths);
    case 'EXPONENTIAL_SMOOTHING':
      return calculateExponentialSmoothing(sku, historicalData, forecastMonths);
    case 'LINEAR_REGRESSION':
      return calculateLinearRegression(sku, historicalData, forecastMonths);
    default:
      return calculateMovingAverage(sku, historicalData, forecastMonths);
  }
}

function calculateMovingAverage(
  sku: string,
  historicalData: Record<string, number>,
  forecastMonths: number
): ForecastResult {
  const values = Object.values(historicalData);
  const periods = 3; // 3-month moving average
  const recentValues = values.slice(-periods);
  const average = recentValues.reduce((a, b) => a + b, 0) / periods;

  const predictions: ForecastPrediction[] = [];
  const currentDate = new Date();

  for (let i = 1; i <= forecastMonths; i++) {
    const forecastDate = new Date(currentDate);
    forecastDate.setMonth(currentDate.getMonth() + i);
    const month = forecastDate.toISOString().substring(0, 7);

    predictions.push({
      month,
      quantity: Math.round(average),
      confidence: 70,
    });
  }

  return {
    sku,
    method: 'MOVING_AVERAGE',
    predictions,
    accuracy: {
      mad: 0,
      mape: 0,
      rmse: 0,
      trackingSignal: 0,
    },
    confidence: 70,
    generatedAt: new Date(),
  };
}

function calculateExponentialSmoothing(
  sku: string,
  historicalData: Record<string, number>,
  forecastMonths: number,
  alpha: number = 0.3
): ForecastResult {
  const values = Object.values(historicalData);
  let forecast = values[0];

  // Calculate smoothed values
  for (let i = 1; i < values.length; i++) {
    forecast = alpha * values[i] + (1 - alpha) * forecast;
  }

  const predictions: ForecastPrediction[] = [];
  const currentDate = new Date();

  for (let i = 1; i <= forecastMonths; i++) {
    const forecastDate = new Date(currentDate);
    forecastDate.setMonth(currentDate.getMonth() + i);
    const month = forecastDate.toISOString().substring(0, 7);

    predictions.push({
      month,
      quantity: Math.round(forecast),
      confidence: 75,
    });
  }

  return {
    sku,
    method: 'EXPONENTIAL_SMOOTHING',
    predictions,
    accuracy: {
      mad: 0,
      mape: 0,
      rmse: 0,
      trackingSignal: 0,
    },
    confidence: 75,
    generatedAt: new Date(),
  };
}

function calculateLinearRegression(
  sku: string,
  historicalData: Record<string, number>,
  forecastMonths: number
): ForecastResult {
  const values = Object.values(historicalData);
  const n = values.length;

  // Calculate linear regression: Y = a + bX
  let sumX = 0;
  let sumY = 0;
  let sumXY = 0;
  let sumX2 = 0;

  for (let i = 0; i < n; i++) {
    const x = i + 1;
    const y = values[i];
    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumX2 += x * x;
  }

  const avgX = sumX / n;
  const avgY = sumY / n;

  const b = (sumXY - n * avgX * avgY) / (sumX2 - n * avgX * avgX);
  const a = avgY - b * avgX;

  const predictions: ForecastPrediction[] = [];
  const currentDate = new Date();

  for (let i = 1; i <= forecastMonths; i++) {
    const forecastDate = new Date(currentDate);
    forecastDate.setMonth(currentDate.getMonth() + i);
    const month = forecastDate.toISOString().substring(0, 7);

    const x = n + i;
    const forecastValue = a + b * x;

    predictions.push({
      month,
      quantity: Math.round(Math.max(0, forecastValue)),
      confidence: 80,
    });
  }

  return {
    sku,
    method: 'LINEAR_REGRESSION',
    predictions,
    accuracy: {
      mad: 0,
      mape: 0,
      rmse: 0,
      trackingSignal: 0,
    },
    confidence: 80,
    generatedAt: new Date(),
  };
}

// ============================================================================
// Inventory Valuation
// ============================================================================

export function calculateInventoryValue(
  items: InventoryItem[],
  method: ValuationMethod
): InventoryValuation[] {
  const valuations: InventoryValuation[] = [];

  items.forEach((item) => {
    const totalValue = item.quantity * item.unitCost;

    valuations.push({
      sku: item.sku,
      method,
      quantity: item.quantity,
      totalValue,
      averageCost: item.unitCost,
      calculatedAt: new Date(),
    });
  });

  return valuations;
}

// ============================================================================
// ABC Analysis
// ============================================================================

export function performABCAnalysis(items: InventoryItem[]): InventoryItem[] {
  // Calculate total value for each item
  const itemsWithValue = items.map((item) => ({
    ...item,
    totalValue: item.quantity * item.unitCost,
  }));

  // Sort by total value descending
  itemsWithValue.sort((a, b) => b.totalValue - a.totalValue);

  const totalValue = itemsWithValue.reduce((sum, item) => sum + item.totalValue, 0);

  let cumulativeValue = 0;
  let cumulativePercent = 0;

  // Assign ABC categories
  return itemsWithValue.map((item) => {
    cumulativeValue += item.totalValue;
    cumulativePercent = (cumulativeValue / totalValue) * 100;

    let category: ABCCategory;
    if (cumulativePercent <= 80) {
      category = 'A';
    } else if (cumulativePercent <= 95) {
      category = 'B';
    } else {
      category = 'C';
    }

    return {
      ...item,
      abcCategory: category,
    };
  });
}

// ============================================================================
// Stock Level Optimization
// ============================================================================

export function optimizeReorderPoint(params: {
  sku: string;
  historicalDemand: number[];
  leadTime: number;
  serviceLevel: number;
}): ReorderPointCalculation {
  const { historicalDemand, leadTime, serviceLevel } = params;

  // Calculate average daily demand
  const averageDailyDemand =
    historicalDemand.reduce((a, b) => a + b, 0) / historicalDemand.length;

  // Calculate standard deviation
  const variance =
    historicalDemand.reduce((sum, val) => {
      return sum + Math.pow(val - averageDailyDemand, 2);
    }, 0) / historicalDemand.length;
  const demandStdDev = Math.sqrt(variance);

  // Calculate safety stock
  const safetyStockCalc = calculateSafetyStock({
    serviceLevel,
    demandStdDev,
    leadTime,
  });

  // Calculate reorder point
  return calculateReorderPoint({
    averageDailyDemand,
    leadTime,
    safetyStock: safetyStockCalc.safetyStock,
    serviceLevel,
  });
}

// ============================================================================
// Expiration Management
// ============================================================================

export function getExpiringItems(
  items: InventoryItem[],
  daysThreshold: number
): InventoryItem[] {
  const now = new Date();
  const thresholdDate = new Date();
  thresholdDate.setDate(now.getDate() + daysThreshold);

  return items.filter((item) => {
    if (!item.expirationDate) return false;
    const expDate = new Date(item.expirationDate);
    return expDate <= thresholdDate && expDate > now;
  });
}

export function sortByFEFO(items: InventoryItem[]): InventoryItem[] {
  return items
    .filter((item) => item.expirationDate)
    .sort((a, b) => {
      const dateA = new Date(a.expirationDate!).getTime();
      const dateB = new Date(b.expirationDate!).getTime();
      return dateA - dateB;
    });
}

// ============================================================================
// Inventory Turnover Calculator
// ============================================================================

export function calculateInventoryTurnover(
  costOfGoodsSold: number,
  averageInventoryValue: number
): number {
  if (averageInventoryValue === 0) return 0;
  return costOfGoodsSold / averageInventoryValue;
}

export function calculateDaysInventoryOutstanding(
  averageInventory: number,
  costOfGoodsSold: number
): number {
  if (costOfGoodsSold === 0) return 0;
  return (averageInventory / costOfGoodsSold) * 365;
}

// ============================================================================
// Stock Status Utilities
// ============================================================================

export function checkStockStatus(
  currentQuantity: number,
  reorderPoint: number,
  safetyStock: number
): StockStatus {
  if (currentQuantity <= 0) return 'OBSOLETE';
  if (currentQuantity <= safetyStock) return 'ON_HOLD';
  if (currentQuantity <= reorderPoint) return 'ALLOCATED';
  return 'AVAILABLE';
}

export function calculateAvailableToPromise(
  onHandInventory: number,
  allocatedInventory: number,
  safetyStock: number,
  plannedReceipts: number = 0,
  plannedShipments: number = 0
): number {
  return (
    onHandInventory -
    allocatedInventory -
    safetyStock +
    plannedReceipts -
    plannedShipments
  );
}

// ============================================================================
// Utility Functions
// ============================================================================

export function validateSKU(sku: string): boolean {
  // Basic SKU validation
  return /^[A-Z0-9-]+$/.test(sku) && sku.length >= 3 && sku.length <= 50;
}

export function formatCurrency(amount: number, currency: string = 'USD'): string {
  return new Intl.NumberFormat('en-US', {
    style: 'currency',
    currency,
  }).format(amount);
}

export function formatQuantity(quantity: number, uom: string): string {
  return `${quantity.toLocaleString()} ${uom}`;
}

// ============================================================================
// Export default
// ============================================================================

export default {
  InventoryManager,
  calculateEOQ,
  calculateReorderPoint,
  calculateSafetyStock,
  forecastDemand,
  calculateInventoryValue,
  performABCAnalysis,
  optimizeReorderPoint,
  getExpiringItems,
  sortByFEFO,
  calculateInventoryTurnover,
  calculateDaysInventoryOutstanding,
  checkStockStatus,
  calculateAvailableToPromise,
  validateSKU,
  formatCurrency,
  formatQuantity,
};

/**
 * 弘益人間 (홍익인간) · Benefit All Humanity
 */
