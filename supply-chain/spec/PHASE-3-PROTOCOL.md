# PHASE 3 — Protocol

> Supply-chain operational protocols: shipment tracking, blockchain
> traceability, end-to-end supply-chain visibility, demand planning,
> and logistics optimization.

## 10. Shipment Tracking

### 10.1 Tracking Modes

#### 10.1.1 Air Freight

- **Tracking Events**: Booked, received, departed, in-transit, arrived, customs, delivered
- **Update Frequency**: Every 6 hours minimum
- **ETA Accuracy**: ±2 hours
- **Typical Duration**: 1-5 days international

#### 10.1.2 Ocean Freight

- **Tracking Events**: Booked, loaded, departed port, in-transit, arrived port, discharged, customs, delivered
- **Update Frequency**: Daily
- **ETA Accuracy**: ±1 day
- **Typical Duration**: 15-45 days international

#### 10.1.3 Road Transport

- **Tracking Events**: Picked up, in-transit, rest stop, refueling, delivered
- **Update Frequency**: Real-time (GPS)
- **ETA Accuracy**: ±30 minutes
- **Typical Duration**: 1-7 days domestic

#### 10.1.4 Rail Transport

- **Tracking Events**: Loaded, departed, in-transit, arrived, unloaded
- **Update Frequency**: Every 12 hours
- **ETA Accuracy**: ±4 hours
- **Typical Duration**: 5-15 days

### 10.2 Real-time Tracking

#### 10.2.1 GPS Tracking

```json
{
  "shipmentId": "SHIP-2025-001234",
  "currentLocation": {
    "latitude": 22.3080,
    "longitude": 113.9185,
    "altitude": 5.2,
    "accuracy": 10,
    "speed": 65,
    "heading": 285
  },
  "timestamp": "2025-12-27T14:30:00Z",
  "nextCheckpoint": {
    "name": "Los Angeles Airport",
    "eta": "2025-12-28T08:00:00Z",
    "distance": 11250
  }
}
```

#### 10.2.2 IoT Sensors

**Temperature Monitoring** (Cold Chain):
```json
{
  "sensorId": "TEMP-5678",
  "shipmentId": "SHIP-2025-001234",
  "readings": [{
    "timestamp": "2025-12-27T14:30:00Z",
    "temperature": 2.5,
    "unit": "C",
    "status": "normal",
    "alert": false
  }],
  "thresholds": {
    "min": 2.0,
    "max": 8.0,
    "alertDelay": 300
  }
}
```

**Shock/Vibration Monitoring**:
```json
{
  "sensorId": "SHOCK-9012",
  "shipmentId": "SHIP-2025-001234",
  "readings": [{
    "timestamp": "2025-12-27T14:30:00Z",
    "force": 2.5,
    "unit": "G",
    "duration": 50,
    "axis": "vertical",
    "alert": false
  }],
  "threshold": {
    "max": 5.0,
    "duration": 100
  }
}
```

### 10.3 Exception Management

#### 10.3.1 Exception Types

- **Delay**: Shipment behind schedule
- **Damage**: Package damage detected
- **Lost**: Package missing or unaccounted
- **Customs Hold**: Held by customs
- **Weather**: Delayed due to weather
- **Mechanical**: Vehicle breakdown
- **Security**: Security incident
- **Documentation**: Missing or incorrect documents

#### 10.3.2 Exception Handling

```
Exception Detected
    ↓
Auto-notification (Email/SMS/Webhook)
    ↓
Impact Assessment (Delivery date, cost)
    ↓
Mitigation Plan (Expedite, reroute, replace)
    ↓
Stakeholder Communication
    ↓
Resolution Tracking
    ↓
Post-incident Review
```

---


## 11. Blockchain Traceability

### 11.1 Architecture

```
┌─────────────────────────────────────────────────┐
│         Supply Chain Applications               │
├─────────────────────────────────────────────────┤
│              WIA-IND-023 SDK                    │
├─────────────────────────────────────────────────┤
│          Smart Contract Layer                   │
│  (Product Registry, Transfer, Verification)     │
├─────────────────────────────────────────────────┤
│         Blockchain Network                      │
│  (Ethereum / Polygon / Hyperledger)            │
└─────────────────────────────────────────────────┘
```

### 11.2 Smart Contracts

#### 11.2.1 Product Registry Contract

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract ProductRegistry {
    struct Product {
        string sku;
        string serialNumber;
        address manufacturer;
        uint256 manufactureDate;
        string origin;
        bool isAuthentic;
    }

    mapping(bytes32 => Product) public products;
    mapping(bytes32 => Checkpoint[]) public journey;

    struct Checkpoint {
        string stage;
        uint256 timestamp;
        string location;
        address verifiedBy;
        string metadata;
    }

    event ProductRegistered(bytes32 indexed productId, string sku);
    event CheckpointAdded(bytes32 indexed productId, string stage);

    function registerProduct(
        string memory _sku,
        string memory _serialNumber,
        string memory _origin
    ) public returns (bytes32) {
        bytes32 productId = keccak256(abi.encodePacked(_sku, _serialNumber));

        products[productId] = Product({
            sku: _sku,
            serialNumber: _serialNumber,
            manufacturer: msg.sender,
            manufactureDate: block.timestamp,
            origin: _origin,
            isAuthentic: true
        });

        emit ProductRegistered(productId, _sku);
        return productId;
    }

    function addCheckpoint(
        bytes32 _productId,
        string memory _stage,
        string memory _location,
        string memory _metadata
    ) public {
        require(products[_productId].isAuthentic, "Product not found");

        journey[_productId].push(Checkpoint({
            stage: _stage,
            timestamp: block.timestamp,
            location: _location,
            verifiedBy: msg.sender,
            metadata: _metadata
        }));

        emit CheckpointAdded(_productId, _stage);
    }

    function verifyProduct(bytes32 _productId) public view returns (
        bool isAuthentic,
        string memory sku,
        address manufacturer,
        uint256 checkpoints
    ) {
        Product memory product = products[_productId];
        return (
            product.isAuthentic,
            product.sku,
            product.manufacturer,
            journey[_productId].length
        );
    }
}
```

### 11.3 Provenance Recording

#### 11.3.1 Checkpoint Stages

1. **Origin**: Raw material sourcing
2. **Manufacturing**: Production and assembly
3. **Quality Check**: Inspection and testing
4. **Packaging**: Final packaging
5. **Warehouse**: Storage and inventory
6. **Shipping**: Transportation
7. **Distribution**: Regional distribution
8. **Retail**: Point of sale
9. **Consumer**: End user delivery

#### 11.3.2 Data Structure

```json
{
  "checkpoint": {
    "id": "cp_12345",
    "productId": "0x7f8c...",
    "stage": "manufacturing",
    "timestamp": "2025-12-27T10:00:00Z",
    "location": {
      "name": "Factory Floor 3",
      "facility": "Shenzhen Plant",
      "address": {
        "city": "Shenzhen",
        "country": "China",
        "coordinates": {
          "latitude": 22.5431,
          "longitude": 114.0579
        }
      }
    },
    "verifiedBy": "0x1234...",
    "data": {
      "batchNumber": "BATCH-2025-001",
      "operator": "OP-5678",
      "equipment": "LINE-A-03",
      "qualityScore": 98.5,
      "temperature": 22.5,
      "humidity": 45
    },
    "certifications": ["ISO9001", "IATF16949"],
    "blockchain": {
      "network": "polygon",
      "txHash": "0xabc...",
      "blockNumber": 45678901,
      "gasUsed": 125000,
      "confirmations": 12
    },
    "attachments": [
      "ipfs://Qm.../quality-report.pdf",
      "ipfs://Qm.../inspection-photos.zip"
    ]
  }
}
```

### 11.4 Verification Process

#### 11.4.1 QR Code Verification

```
Consumer scans QR code on product
    ↓
Mobile app extracts product ID
    ↓
Query blockchain for product record
    ↓
Retrieve full journey history
    ↓
Verify authenticity (hash validation)
    ↓
Display provenance information
    ↓
Show certifications and origin
```

#### 11.4.2 API Verification

```bash
POST /api/v1/verify
{
  "sku": "CHIP-A100",
  "serialNumber": "SN-12345",
  "blockchainHash": "0x7f8c..."
}

Response:
{
  "isAuthentic": true,
  "confidence": 0.99,
  "product": {
    "sku": "CHIP-A100",
    "name": "Industrial Microchip A100",
    "manufacturer": "Quality Tech Inc.",
    "manufactureDate": "2025-11-15"
  },
  "origin": {
    "country": "Taiwan",
    "facility": "Hsinchu Fab 5"
  },
  "journey": [
    {
      "stage": "manufacturing",
      "date": "2025-11-15",
      "location": "Hsinchu, Taiwan",
      "verified": true
    },
    {
      "stage": "quality_check",
      "date": "2025-11-16",
      "location": "QC Lab, Taiwan",
      "verified": true
    },
    ...
  ],
  "certifications": ["ISO9001", "RoHS", "CE"],
  "carbonFootprint": 1.2,
  "blockchainVerification": {
    "network": "polygon",
    "txHash": "0x7f8c...",
    "verified": true,
    "checkpoints": 7
  }
}
```

---


## 12. Supply Chain Visibility

### 12.1 Multi-Tier Visibility

#### 12.1.1 Network Mapping

```
                    [Raw Material Suppliers]
                            Tier 3
                              ↓
                    [Component Manufacturers]
                            Tier 2
                              ↓
                    [Sub-Assembly Suppliers]
                            Tier 1
                              ↓
                        [OEM/Buyer]
                              ↓
                        [Distributors]
                              ↓
                          [Retailers]
                              ↓
                        [End Consumers]
```

#### 12.1.2 Visibility Levels

**Level 1** (Direct Suppliers):
- Real-time order status
- Production schedules
- Quality metrics
- Delivery tracking
- Invoice status

**Level 2** (Sub-tier Suppliers):
- Order acknowledgement
- Estimated delivery
- Capacity constraints
- Quality certifications

**Level 3+** (Deep tier):
- Supplier identification
- Geographic location
- Risk factors
- Certifications

### 12.2 Dashboard Metrics

#### 12.2.1 Executive Dashboard

- **Order Value**: Total PO value by period
- **Supplier Count**: Active suppliers by tier
- **On-Time Delivery**: OTD% trend
- **Quality Performance**: Defect rate trend
- **Risk Exposure**: High-risk suppliers
- **Sustainability**: Carbon footprint
- **Cost Savings**: Year-over-year comparison

#### 12.2.2 Operational Dashboard

- **Open Orders**: Count by status
- **Shipments in Transit**: Real-time map
- **Delays**: Shipments behind schedule
- **Exceptions**: Active issues
- **Inventory Levels**: Current stock by SKU
- **Incoming**: Expected receipts next 7 days
- **Backorders**: Outstanding orders

---


## 14. Demand Planning

### 14.1 Forecasting Methods

#### 14.1.1 Time Series Methods

**Moving Average**:
```
MA(t) = (D(t-1) + D(t-2) + ... + D(t-n)) / n
```

**Exponential Smoothing**:
```
F(t) = α × D(t-1) + (1-α) × F(t-1)
```
Where α = smoothing constant (0-1)

**Holt-Winters (Seasonal)**:
```
Level: L(t) = α × (D(t)/S(t-s)) + (1-α) × (L(t-1) + T(t-1))
Trend: T(t) = β × (L(t) - L(t-1)) + (1-β) × T(t-1)
Season: S(t) = γ × (D(t)/L(t)) + (1-γ) × S(t-s)
Forecast: F(t+m) = (L(t) + m×T(t)) × S(t+m-s)
```

#### 14.1.2 Causal Methods

**Regression Analysis**:
```
Demand = β0 + β1×Price + β2×Marketing + β3×Economy + ε
```

**Machine Learning**:
- Random Forest
- Gradient Boosting
- Neural Networks (LSTM, GRU)
- Prophet (Facebook's algorithm)

### 14.2 Forecast Accuracy Metrics

**Mean Absolute Percentage Error (MAPE)**:
```
MAPE = (100/n) × Σ|Actual - Forecast| / Actual
```

**Root Mean Squared Error (RMSE)**:
```
RMSE = √(Σ(Actual - Forecast)² / n)
```

**Bias**:
```
Bias = Σ(Actual - Forecast) / n
```

**Tracking Signal**:
```
TS = Cumulative Error / MAD
```
Where MAD = Mean Absolute Deviation

### 14.3 Inventory Optimization

#### 14.3.1 Economic Order Quantity (EOQ)

```
EOQ = √((2 × D × S) / H)

Where:
D = Annual demand
S = Order cost per order
H = Holding cost per unit per year
```

#### 14.3.2 Reorder Point (ROP)

```
ROP = (Average Daily Demand × Lead Time) + Safety Stock

Safety Stock = Z × σ × √Lead Time

Where:
Z = Service level factor (e.g., 1.65 for 95% service level)
σ = Standard deviation of demand
```

#### 14.3.3 ABC Analysis

**Class A** (20% of items, 80% of value):
- Tight inventory control
- Frequent review
- Accurate forecasting
- Low safety stock

**Class B** (30% of items, 15% of value):
- Moderate control
- Regular review
- Standard forecasting

**Class C** (50% of items, 5% of value):
- Simple controls
- Periodic review
- High safety stock
- Bulk orders

---


## 15. Logistics Optimization

### 15.1 Route Optimization

#### 15.1.1 Vehicle Routing Problem (VRP)

```
Minimize: Total Distance (or Cost or Time)

Subject to:
- Each customer visited exactly once
- Vehicle capacity not exceeded
- Time windows respected
- Maximum route duration not exceeded
```

**Algorithms**:
- Clarke-Wright Savings
- Genetic Algorithm
- Simulated Annealing
- Ant Colony Optimization
- Google OR-Tools

#### 15.1.2 Multi-Objective Optimization

```
Objective Function =
  w1 × Cost +
  w2 × Time +
  w3 × Carbon Emissions +
  w4 × Reliability

Where w1 + w2 + w3 + w4 = 1
```

### 15.2 Mode Selection

#### 15.2.1 Cost Comparison

| Mode | Cost/kg/km | Speed | Carbon | Reliability |
|------|-----------|-------|--------|-------------|
| Air | $0.50-2.00 | Fast | High | Very High |
| Ocean | $0.01-0.05 | Slow | Low | Medium |
| Road | $0.10-0.30 | Medium | Medium | High |
| Rail | $0.05-0.15 | Medium | Low | Medium |

#### 15.2.2 Selection Criteria

**Urgent (Express)**:
- Mode: Air
- Priority: Speed
- Cost: High
- Use cases: Emergency parts, perishables, high-value

**Standard**:
- Mode: Road/Rail
- Priority: Balanced
- Cost: Medium
- Use cases: Regular shipments, domestic

**Economy**:
- Mode: Ocean
- Priority: Cost
- Cost: Low
- Use cases: Bulk commodities, non-urgent

### 15.3 Load Planning

#### 15.3.1 Container Optimization

**20ft Container**: 33 CBM, 28,200 kg max
**40ft Container**: 67 CBM, 28,800 kg max
**40ft HC**: 76 CBM, 28,600 kg max

**Bin Packing Algorithm**:
```
For each item:
  Find smallest bin with sufficient space
  If no bin found:
    Open new bin
  Place item in bin
  Update remaining capacity
```

#### 15.3.2 Weight Distribution

```
Center of Gravity = Σ(Weight × Distance) / Total Weight

Requirements:
- Max weight per axle
- Balanced left-right
- Heavy items on bottom
- Fragile items protected
```

---


