# 📦 WIA-IND-022: Inventory Management Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-022
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND / Industry
> **Color:** Amber (#F59E0B)

---

## 🌟 Overview

The WIA-IND-022 standard defines comprehensive inventory management systems including stock tracking, warehouse management, barcode/RFID integration, demand forecasting, reorder optimization, multi-location inventory, batch/lot tracking, expiration management, inventory valuation, and dead stock management.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to optimize supply chains worldwide, reducing waste, improving efficiency, and ensuring product availability for all.

## 🎯 Key Features

- **Real-time Stock Tracking**: Live inventory counts across all locations
- **Warehouse Management**: Multi-warehouse and zone-based organization
- **Barcode & RFID**: Automated scanning and RFID tag integration
- **Demand Forecasting**: AI-powered prediction of future inventory needs
- **Reorder Point Optimization**: Automatic reordering based on EOQ models
- **Multi-location Inventory**: Centralized control across warehouses
- **Batch & Lot Tracking**: Full traceability for regulatory compliance
- **Expiration Management**: FEFO/FIFO handling for perishables
- **Inventory Valuation**: FIFO, LIFO, weighted average methods
- **Dead Stock Management**: Identify and manage slow-moving inventory
- **Cycle Counting**: Continuous accuracy verification
- **Transfer Management**: Inter-warehouse transfer tracking

## 📊 Core Concepts

### 1. Inventory Control Methods

```
Stock Management Approaches:
- Perpetual Inventory: Real-time tracking with every transaction
- Periodic Inventory: Regular cycle counts at intervals
- Just-in-Time (JIT): Minimal stock, frequent replenishment
- Economic Order Quantity (EOQ): Optimized order quantities
- ABC Analysis: Categorize items by value and turnover
```

### 2. Warehouse Zones

```
Typical Warehouse Layout:
1. Receiving Zone: Incoming shipments and inspection
2. Storage Zones: Bulk, rack, bin, and floor storage
3. Picking Zones: High-frequency items for quick access
4. Packing Zone: Order preparation and packaging
5. Shipping Zone: Outbound staging and dispatch
6. Quarantine Zone: Returns, damages, and holds
```

### 3. Performance Metrics

| Metric | Target | Industry Average | World Class |
|--------|--------|------------------|-------------|
| Inventory Accuracy | >99% | 95% | 99.9% |
| Inventory Turnover Ratio | 8-12x | 6x | 15x+ |
| Order Fill Rate | >98% | 90% | 99.5% |
| Days Inventory Outstanding | 30-45 | 60 | <30 |
| Carrying Cost % | <20% | 25% | <15% |
| Dead Stock % | <5% | 10-15% | <2% |
| Picking Accuracy | >99.5% | 98% | 99.9% |
| Cycle Count Accuracy | >99% | 95% | 99.8% |

## 🔧 Components

### TypeScript SDK

```typescript
import {
  InventoryManager,
  calculateEOQ,
  forecastDemand,
  optimizeReorderPoint,
  trackBatchLot,
  calculateInventoryValue,
  analyzeDeadStock
} from '@wia/ind-022';

// Initialize inventory manager
const inventory = new InventoryManager({
  warehouses: ['WH-001', 'WH-002', 'WH-003'],
  defaultValuationMethod: 'FIFO',
  enableRFID: true,
  enableDemandForecasting: true
});

// Add inventory item
await inventory.addItem({
  sku: 'PROD-12345',
  name: 'Widget A',
  category: 'Electronics',
  warehouse: 'WH-001',
  zone: 'A-01-03',
  quantity: 500,
  unitCost: 25.50,
  reorderPoint: 100,
  reorderQuantity: 250,
  batchNumber: 'BATCH-2025-001',
  expirationDate: '2026-12-31',
  rfidTag: 'RFID-ABC123'
});

// Calculate Economic Order Quantity
const eoq = calculateEOQ({
  annualDemand: 10000,
  orderCost: 50,
  holdingCostPerUnit: 5
});

console.log(`Optimal Order Quantity: ${eoq.orderQuantity} units`);
console.log(`Total Annual Cost: $${eoq.totalCost}`);
console.log(`Number of Orders: ${eoq.ordersPerYear}`);

// Forecast demand using historical data
const forecast = await forecastDemand({
  sku: 'PROD-12345',
  historicalData: {
    '2025-01': 850,
    '2025-02': 920,
    '2025-03': 880,
    '2025-04': 1100,
    '2025-05': 1050,
    '2025-06': 1200
  },
  forecastMonths: 3,
  method: 'exponential-smoothing'
});

console.log('Demand Forecast:');
forecast.predictions.forEach(p => {
  console.log(`${p.month}: ${p.quantity} units (confidence: ${p.confidence}%)`);
});

// Track batch movement
await inventory.trackBatch({
  batchNumber: 'BATCH-2025-001',
  action: 'PICK',
  quantity: 50,
  location: 'WH-001/A-01-03',
  operator: 'EMP-123',
  timestamp: new Date()
});

// Analyze dead stock
const deadStock = await inventory.analyzeDeadStock({
  minDaysWithoutMovement: 90,
  minQuantity: 10,
  includeValue: true
});

console.log(`Dead Stock Items: ${deadStock.items.length}`);
console.log(`Total Dead Stock Value: $${deadStock.totalValue}`);
```

### CLI Tool

```bash
# Add inventory item
wia-ind-022 add-item \
  --sku PROD-12345 \
  --name "Widget A" \
  --warehouse WH-001 \
  --quantity 500 \
  --cost 25.50

# Check stock levels
wia-ind-022 check-stock --sku PROD-12345 --all-warehouses

# Calculate reorder point
wia-ind-022 calculate-reorder \
  --sku PROD-12345 \
  --lead-time 7 \
  --daily-demand 50 \
  --safety-stock 100

# Forecast demand
wia-ind-022 forecast-demand \
  --sku PROD-12345 \
  --months 3 \
  --method exponential-smoothing

# Transfer between warehouses
wia-ind-022 transfer \
  --sku PROD-12345 \
  --from WH-001 \
  --to WH-002 \
  --quantity 100

# Scan barcode/RFID
wia-ind-022 scan --type barcode --code 1234567890123

# Check expiring items
wia-ind-022 check-expiration --days 30

# Analyze dead stock
wia-ind-022 analyze-dead-stock --threshold 90

# Generate inventory report
wia-ind-022 report --type valuation --method FIFO

# Perform cycle count
wia-ind-022 cycle-count --zone A-01 --warehouse WH-001
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-022-v1.0.md](./spec/WIA-IND-022-v1.0.md) | Complete inventory management specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-022.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/inventory-management

# Run installation script
./install.sh

# Verify installation
wia-ind-022 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-022

# Or yarn
yarn add @wia/ind-022
```

```typescript
import { InventoryManager, calculateEOQ } from '@wia/ind-022';

// Initialize
const inventory = new InventoryManager({
  warehouses: ['MAIN', 'BACKUP'],
  defaultValuationMethod: 'FIFO'
});

// Add product
await inventory.addItem({
  sku: 'WIDGET-001',
  name: 'Premium Widget',
  quantity: 1000,
  unitCost: 19.99,
  warehouse: 'MAIN',
  zone: 'A-12-05'
});

// Check stock
const stock = await inventory.getStock('WIDGET-001');
console.log(`Available: ${stock.quantity} units`);
console.log(`Value: $${stock.totalValue}`);
console.log(`Location: ${stock.warehouse}/${stock.zone}`);

// Calculate optimal order quantity
const eoq = calculateEOQ({
  annualDemand: 50000,
  orderCost: 100,
  holdingCostPerUnit: 4
});

console.log(`Order ${eoq.orderQuantity} units, ${eoq.ordersPerYear} times per year`);

// Transfer stock
await inventory.transfer({
  sku: 'WIDGET-001',
  fromWarehouse: 'MAIN',
  toWarehouse: 'BACKUP',
  quantity: 200,
  reason: 'Rebalancing'
});
```

## 🔬 Technical Specifications

### Inventory Valuation Methods

1. **FIFO (First In, First Out)**
   - Oldest inventory sold first
   - Higher profit in rising price environments
   - More realistic physical flow

2. **LIFO (Last In, First Out)**
   - Newest inventory sold first
   - Tax benefits in inflationary periods
   - Not allowed under IFRS

3. **Weighted Average Cost**
   - Average cost of all units
   - Smooths price fluctuations
   - Simpler calculations

4. **Specific Identification**
   - Track each item individually
   - Used for unique/high-value items
   - Most accurate but complex

### Reorder Point Calculation

```
Reorder Point = (Lead Time × Daily Demand) + Safety Stock

Safety Stock = Z × σ × √L
  Z = Service level Z-score (e.g., 1.65 for 95%)
  σ = Standard deviation of daily demand
  L = Lead time in days

Economic Order Quantity (EOQ):
EOQ = √(2 × D × S / H)
  D = Annual demand
  S = Order cost per order
  H = Holding cost per unit per year
```

### ABC Analysis Categories

- **A-items**: 20% of SKUs, 80% of value → Weekly review
- **B-items**: 30% of SKUs, 15% of value → Monthly review
- **C-items**: 50% of SKUs, 5% of value → Quarterly review

### Warehouse Storage Types

1. **Pallet Racking**: High-density storage
2. **Bin Shelving**: Small parts organization
3. **Floor Storage**: Bulk items
4. **Cold Storage**: Temperature-controlled
5. **Hazmat Storage**: Regulated materials
6. **High-Security**: Valuable items

## ⚠️ Implementation Considerations

1. **Accuracy Requirements**: Implement cycle counting programs
2. **RFID vs Barcode**: Consider cost vs automation benefits
3. **Perishable Goods**: Use FEFO (First Expired, First Out)
4. **Lot Traceability**: Critical for recalls and compliance
5. **Multi-location**: Centralize visibility, decentralize execution
6. **Integration**: Connect with ERP, WMS, and POS systems
7. **Mobile Devices**: Enable warehouse mobility
8. **Real-time Updates**: Minimize inventory blind spots

## 🌐 WIA Integration

This standard integrates with:
- **WIA-SUPPLY-CHAIN**: End-to-end supply chain management
- **WIA-WAREHOUSE**: Advanced warehouse operations
- **WIA-ERP**: Enterprise resource planning integration
- **WIA-BARCODE**: Barcode and RFID standards
- **WIA-ANALYTICS**: Inventory analytics and BI
- **WIA-IOT**: Smart sensors and tracking devices
- **WIA-AI**: Demand forecasting and optimization

## 📖 Use Cases

1. **Retail**: Multi-store inventory synchronization
2. **E-commerce**: Real-time stock availability
3. **Manufacturing**: Raw material and WIP tracking
4. **Food & Beverage**: Expiration and batch tracking
5. **Pharmaceuticals**: Lot traceability and compliance
6. **Automotive**: Parts inventory management
7. **Healthcare**: Medical supply management
8. **Distribution**: Multi-warehouse operations
9. **Agriculture**: Harvest and storage tracking
10. **Fashion**: Size/color variant management

## 🎓 Best Practices

### Inventory Accuracy
- Conduct regular cycle counts
- Investigate and correct discrepancies immediately
- Train staff on proper procedures
- Use technology to minimize manual errors

### Stock Optimization
- Set appropriate reorder points
- Review and adjust safety stock levels
- Implement ABC analysis
- Monitor slow-moving inventory

### Warehouse Efficiency
- Optimize warehouse layout
- Use location-based storage
- Implement wave picking
- Cross-dock when possible

### Technology Integration
- Integrate with accounting systems
- Connect to e-commerce platforms
- Use mobile scanning devices
- Implement automated alerts

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
