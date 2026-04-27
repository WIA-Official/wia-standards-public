# PHASE 4 — Integration

> Supply-chain integration with adjacent systems: risk management,
> sustainability tracking, security and privacy, integration
> patterns with ERP/WMS/TMS, compliance and certification, and
> the appendices with example workflows and sample code.

## 13. Risk Management

### 13.1 Risk Categories

#### 13.1.1 Supplier Risk

**Financial Risk**:
- Credit rating
- Payment history
- Financial ratios
- Bankruptcy probability

**Operational Risk**:
- Capacity utilization
- Quality history
- Delivery performance
- Technology capability

**Compliance Risk**:
- Regulatory violations
- Certification expiry
- Audit findings
- Legal disputes

#### 13.1.2 Geographic Risk

**Geopolitical Risk**:
- Political stability index
- Trade restrictions
- Sanctions
- Conflicts

**Natural Disaster Risk**:
- Earthquake zones
- Hurricane/typhoon paths
- Flood plains
- Wildfire areas

**Infrastructure Risk**:
- Port congestion
- Road conditions
- Power stability
- Internet connectivity

#### 13.1.3 Demand Risk

**Market Risk**:
- Demand volatility
- Seasonality
- Competition
- Economic conditions

**Forecast Risk**:
- Forecast accuracy
- Lead time variability
- Bullwhip effect
- Obsolescence

### 13.2 Risk Scoring Algorithm

```python
def calculate_risk_score(supplier_id):
    # Get risk factors
    financial_score = get_financial_score(supplier_id)      # 0-100
    operational_score = get_operational_score(supplier_id)  # 0-100
    compliance_score = get_compliance_score(supplier_id)    # 0-100
    geo_risk = get_geopolitical_risk(supplier_id)          # 0-100

    # Weights
    weights = {
        'financial': 0.30,
        'operational': 0.25,
        'compliance': 0.20,
        'geopolitical': 0.15,
        'concentration': 0.10
    }

    # Calculate weighted score (inverse - higher score = lower risk)
    risk_score = 100 - (
        financial_score * weights['financial'] +
        operational_score * weights['operational'] +
        compliance_score * weights['compliance'] +
        geo_risk * weights['geopolitical'] +
        concentration_score * weights['concentration']
    )

    # Classify risk level
    if risk_score < 20:
        level = 'LOW'
    elif risk_score < 40:
        level = 'MEDIUM'
    elif risk_score < 60:
        level = 'HIGH'
    else:
        level = 'CRITICAL'

    return {
        'score': risk_score,
        'level': level,
        'factors': {
            'financial': financial_score,
            'operational': operational_score,
            'compliance': compliance_score,
            'geopolitical': geo_risk
        }
    }
```

### 13.3 Mitigation Strategies

#### 13.3.1 Supplier Diversification

- **Dual Sourcing**: Two suppliers for critical items
- **Multi Sourcing**: Multiple suppliers for high-volume items
- **Geographic Diversification**: Suppliers in different regions
- **Vertical Integration**: Bring critical capabilities in-house

#### 13.3.2 Inventory Strategies

- **Safety Stock**: Buffer inventory for critical items
- **Strategic Inventory**: Pre-positioning for long lead items
- **Vendor Managed Inventory**: Supplier-owned stock on-site
- **Consignment**: Supplier inventory until consumption

#### 13.3.3 Contractual Protections

- **Force Majeure Clauses**: Protection for unforeseeable events
- **Performance Bonds**: Financial guarantee of performance
- **Penalty Clauses**: Penalties for non-performance
- **Alternative Supply**: Right to source elsewhere if needed

---


## 16. Sustainability Tracking

### 16.1 Carbon Footprint Calculation

#### 16.1.1 Scope 1 (Direct Emissions)

```
Scope 1 = Σ(Fuel Consumption × Emission Factor)

Emission Factors (kg CO2/liter):
- Diesel: 2.68
- Gasoline: 2.31
- Natural Gas: 2.03 kg/m³
```

#### 16.1.2 Scope 2 (Indirect Energy)

```
Scope 2 = Electricity Consumption (kWh) × Grid Emission Factor

Grid Emission Factors (kg CO2/kWh):
- China: 0.555
- USA: 0.417
- EU: 0.296
- Renewables: 0.000
```

#### 16.1.3 Scope 3 (Value Chain)

```
Scope 3 = Σ(Activity Data × Emission Factor)

Categories:
1. Purchased goods and services
2. Capital goods
3. Fuel and energy
4. Upstream transportation
5. Waste generated
6. Business travel
7. Employee commuting
8. Upstream leased assets
9. Downstream transportation
10. Processing of sold products
11. Use of sold products
12. End-of-life treatment
13. Downstream leased assets
14. Franchises
15. Investments
```

#### 16.1.4 Transportation Emissions

```
Transport CO2 =
  (Weight in tonnes × Distance in km × Emission Factor) /
  Load Factor

Emission Factors (g CO2/tonne-km):
- Air: 500
- Ocean: 15
- Road: 62
- Rail: 28
```

### 16.2 ESG Scoring

#### 16.2.1 Environmental Score (E)

**Metrics** (Weight):
- Carbon footprint (25%)
- Renewable energy use (20%)
- Waste reduction (15%)
- Water consumption (15%)
- Recycled materials (15%)
- Biodiversity impact (10%)

**Calculation**:
```
E-Score = Σ(Metric Score × Weight)

Score ranges:
90-100: Leader
70-89: Good
50-69: Moderate
0-49: Laggard
```

#### 16.2.2 Social Score (S)

**Metrics** (Weight):
- Labor practices (30%)
- Health & safety (25%)
- Diversity & inclusion (20%)
- Community impact (15%)
- Human rights (10%)

#### 16.2.3 Governance Score (G)

**Metrics** (Weight):
- Compliance (30%)
- Ethics & transparency (25%)
- Board diversity (20%)
- Risk management (15%)
- Certifications (10%)

#### 16.2.4 Overall ESG Score

```
ESG Score =
  (E-Score × 0.40) +
  (S-Score × 0.30) +
  (G-Score × 0.30)
```

### 16.3 Circular Economy

#### 16.3.1 Product Lifecycle

```
Design → Manufacture → Use → Collect →
Recycle/Refurbish → Reuse → [back to Use]
```

#### 16.3.2 Circularity Metrics

**Material Circularity Indicator (MCI)**:
```
MCI = (Virgin Material / Total Material) × 100

Target: < 20% (80%+ recycled content)
```

**Product Utilization Rate**:
```
Utilization = Actual Use Time / Expected Lifetime

Target: > 80%
```

**Recovery Rate**:
```
Recovery Rate = (Recycled + Reused) / Total End-of-Life

Target: > 90%
```

---


## 18. Security and Privacy

### 18.1 Data Security

#### 18.1.1 Encryption

- **In Transit**: TLS 1.3
- **At Rest**: AES-256
- **Database**: Field-level encryption for PII
- **Blockchain**: Public/private key cryptography

#### 18.1.2 Access Control

**Role-Based Access Control (RBAC)**:
- Admin: Full access
- Procurement Manager: Create/approve POs
- Warehouse: Receive shipments
- Finance: View/approve invoices
- Supplier: View own orders only
- Auditor: Read-only access

### 18.2 Data Privacy

#### 18.2.1 Compliance

- **GDPR**: EU data protection
- **CCPA**: California privacy
- **LGPD**: Brazil data protection
- **PIPL**: China privacy law

#### 18.2.2 Data Retention

- **Transactional Data**: 7 years
- **Communication**: 3 years
- **Audit Logs**: 10 years
- **Personal Data**: As required by law or until consent withdrawn

### 18.3 Audit Trail

```json
{
  "auditId": "AUDIT-12345",
  "timestamp": "2025-12-27T14:30:00Z",
  "userId": "user_123",
  "action": "order.approved",
  "resourceType": "PurchaseOrder",
  "resourceId": "ORD-2025-5678",
  "changes": {
    "status": {
      "from": "pending_approval",
      "to": "approved"
    },
    "approvedBy": {
      "from": null,
      "to": "user_123"
    }
  },
  "ipAddress": "203.0.113.42",
  "userAgent": "Mozilla/5.0...",
  "metadata": {
    "approvalReason": "Budget approved",
    "comments": "Expedite delivery"
  }
}
```

---


## 19. Integration Patterns

### 19.1 ERP Integration

#### 19.1.1 SAP Integration

```xml
<!-- IDoc ORDERS05 -->
<ORDERS05>
  <IDOC BEGIN="1">
    <EDI_DC40>
      <DOCNUM>1234567890</DOCNUM>
      <MESTYP>ORDERS</MESTYP>
    </EDI_DC40>
    <E1EDK01>
      <CURCY>USD</CURCY>
      <BELNR>PO-2025-001234</BELNR>
    </E1EDK01>
    <E1EDP01>
      <POSEX>00010</POSEX>
      <MENGE>1000</MENGE>
      <MATNR>CHIP-A100</MATNR>
    </E1EDP01>
  </IDOC>
</ORDERS05>
```

#### 19.1.2 Oracle EBS Integration

```sql
-- Order Interface Table
INSERT INTO PO_HEADERS_INTERFACE (
  interface_header_id,
  batch_id,
  org_id,
  vendor_id,
  vendor_site_id,
  currency_code,
  approved_flag
) VALUES (
  po_headers_interface_s.nextval,
  :batch_id,
  :org_id,
  :vendor_id,
  :vendor_site_id,
  'USD',
  'Y'
);
```

### 19.2 WMS Integration

#### 19.2.1 ASN (Advanced Shipping Notice)

```json
{
  "asnId": "ASN-2025-001234",
  "shipmentId": "SHIP-2025-001234",
  "orderId": "ORD-2025-5678",
  "expectedArrival": "2025-12-28T10:00:00Z",
  "carrier": "DHL",
  "trackingNumber": "1234567890",
  "packages": [{
    "packageId": "PKG-001",
    "weight": 50,
    "dimensions": { },
    "contents": [{
      "sku": "CHIP-A100",
      "quantity": 1000,
      "lotNumber": "LOT-2025-001",
      "expiryDate": "2027-12-31"
    }]
  }]
}
```

### 19.3 TMS Integration

#### 19.3.1 Shipment Booking

```json
{
  "bookingRequest": {
    "orderId": "ORD-2025-5678",
    "origin": {
      "address": { },
      "readyDate": "2025-12-27"
    },
    "destination": {
      "address": { },
      "requiredDate": "2025-12-30"
    },
    "cargo": [{
      "packageType": "box",
      "quantity": 10,
      "weight": 50,
      "dimensions": { }
    }],
    "serviceLevel": "express",
    "requirements": {
      "temperatureControl": true,
      "insurance": true,
      "signature": true
    }
  }
}
```

---


## 20. Compliance and Certification

### 20.1 WIA Certification

#### 20.1.1 Certification Levels

**Level 1 - Basic Compliance**:
- Implement core data models
- REST API integration
- Basic shipment tracking
- Standard reporting

**Level 2 - Advanced Integration**:
- Multi-tier visibility
- Real-time tracking
- Risk management
- Demand forecasting

**Level 3 - Excellence**:
- Blockchain traceability
- AI-powered optimization
- Carbon accounting
- Full ESG compliance

#### 20.1.2 Certification Process

1. **Application**: Submit certification request
2. **Documentation**: Provide implementation details
3. **Testing**: Pass conformance tests
4. **Audit**: On-site or remote audit
5. **Certification**: Receive WIA certificate
6. **Renewal**: Annual recertification

### 20.2 Conformance Testing

#### 20.2.1 API Conformance

```bash
# Run conformance test suite
wia-ind-023 test conformance --api-url https://your-api.com

Tests:
✓ Authentication (OAuth 2.0)
✓ Supplier CRUD operations
✓ Purchase order workflow
✓ Shipment tracking
✓ Blockchain verification
✓ Risk assessment
✓ Carbon calculation
✓ Webhook delivery
✓ Error handling
✓ Rate limiting

Result: 10/10 tests passed
Certification: Level 3 (Excellence)
```

### 20.3 Industry Standards

#### 20.3.1 Compliance Matrix

| Standard | Requirement | WIA-IND-023 Support |
|----------|------------|-------------------|
| ISO 28000 | Supply chain security | ✓ Full |
| ISO 9001 | Quality management | ✓ Full |
| ISO 14001 | Environmental mgmt | ✓ Full |
| GS1 | Global standards | ✓ Partial |
| EDIFACT | EDI messages | ✓ Full |
| GHG Protocol | Carbon accounting | ✓ Full |
| INCOTERMS 2020 | Trade terms | ✓ Full |

---


## Appendix A: Example Workflows

### A.1 Complete Order Workflow

```
1. Demand Planning
   └─> Generate forecast for SKU-A
   └─> Inventory recommendation: Order 3000 units

2. Purchase Requisition
   └─> Create requisition REQ-001
   └─> Manager approval
   └─> Procurement processing

3. Supplier Selection
   └─> Query available suppliers
   └─> Calculate risk scores
   └─> Select Supplier SUP-5678 (Risk: LOW, Cost: Best)

4. Purchase Order
   └─> Create PO-2025-001234
   └─> Director approval (value > $100k)
   └─> Send PO to supplier via EDI

5. Order Acknowledgement
   └─> Supplier confirms: 3000 units, delivery Dec 30
   └─> Record blockchain checkpoint: Order Confirmed

6. Production
   └─> Supplier updates: In Production
   └─> Record checkpoint: Manufacturing Stage
   └─> Quality check passed
   └─> Record checkpoint: QC Approved

7. Shipment
   └─> Create shipment SHIP-2025-001234
   └─> Book carrier: DHL Express Air
   └─> Generate ASN
   └─> Record checkpoint: Shipped

8. Tracking
   └─> Real-time GPS tracking
   └─> Temperature monitoring (cold chain)
   └─> ETA updates
   └─> Customs clearance

9. Delivery
   └─> Arrived at warehouse
   └─> Physical inspection
   └─> Quantity verification: 3000 units ✓
   └─> Quality check: Passed ✓
   └─> Record checkpoint: Received

10. Invoice & Payment
    └─> Match invoice to PO
    └─> 3-way match: PO, Receipt, Invoice ✓
    └─> Payment scheduled: NET30
    └─> Update supplier performance metrics

11. Blockchain Finalization
    └─> Record final checkpoint: Delivered
    └─> Close product journey
    └─> Generate authenticity certificate
    └─> Calculate carbon footprint: 280.5 kg CO2e
```

### A.2 Risk Mitigation Workflow

```
1. Risk Detection
   └─> Weekly supplier risk scan
   └─> SUP-5678 risk increased: 15 → 35 (MEDIUM)
   └─> Trigger: Geopolitical risk +20 points

2. Impact Analysis
   └─> Active orders: 5 POs, $500k value
   └─> Critical items: 3 SKUs with no alternative
   └─> Projected delivery impact: +10 days

3. Mitigation Planning
   └─> Strategy 1: Diversify to SUP-ALT-001 (cost +5%)
   └─> Strategy 2: Increase safety stock +15%
   └─> Strategy 3: Expedite current orders

4. Stakeholder Communication
   └─> Alert procurement team
   └─> Notify affected departments
   └─> Email to management

5. Implementation
   └─> Approve alternate supplier SUP-ALT-001
   └─> Split next order 60/40
   └─> Increase safety stock for critical SKUs
   └─> Expedite 2 urgent orders to air freight

6. Monitoring
   └─> Daily risk score updates
   └─> Track geopolitical developments
   └─> Monitor alternate supplier performance

7. Review
   └─> 30-day review: Risk decreased to 25
   └─> Supplier performance maintained
   └─> Continue dual-source strategy
```

---

## Appendix B: Sample Code

### B.1 TypeScript SDK Usage

```typescript
import { SupplyChainSDK } from '@wia/ind-023';

const sdk = new SupplyChainSDK({
  apiKey: process.env.WIA_API_KEY!,
  blockchain: {
    network: 'polygon',
    contractAddress: '0x...'
  }
});

async function main() {
  // 1. Create purchase order
  const po = await sdk.createPurchaseOrder({
    supplierId: 'SUP-5678',
    items: [{
      sku: 'CHIP-A100',
      quantity: 1000,
      unitPrice: 45.50
    }],
    deliveryDate: '2025-12-30'
  });

  console.log(`PO Created: ${po.poNumber}`);

  // 2. Track shipment
  const tracking = await sdk.trackShipment('SHIP-2025-001234');
  console.log(`Status: ${tracking.status}`);
  console.log(`ETA: ${tracking.eta}`);

  // 3. Verify on blockchain
  const verification = await sdk.verifyProvenance(
    'CHIP-A100',
    '0x7f8c...'
  );
  console.log(`Authentic: ${verification.isAuthentic}`);
  console.log(`Journey: ${verification.journey.length} checkpoints`);

  // 4. Calculate risk
  const risk = await sdk.calculateRiskScore('SUP-5678');
  console.log(`Risk: ${risk.riskLevel} (${risk.riskScore})`);

  // 5. Demand forecast
  const forecast = await sdk.generateForecast({
    sku: 'CHIP-A100',
    period: 90
  });
  console.log(`90-day forecast generated`);

  // 6. Carbon footprint
  const carbon = await sdk.calculateCarbonFootprint({
    shipmentId: 'SHIP-2025-001234'
  });
  console.log(`Carbon: ${carbon.totalKg} kg CO2e`);
}

main();
```

---

## Appendix C: Glossary

**3PL**: Third-Party Logistics Provider
**4PL**: Fourth-Party Logistics Provider
**ASN**: Advanced Shipping Notice
**BOL**: Bill of Lading
**COGS**: Cost of Goods Sold
**CTC**: Cash-to-Cash Cycle
**DDP**: Delivered Duty Paid
**DIO**: Days Inventory Outstanding
**DPO**: Days Payable Outstanding
**DSO**: Days Sales Outstanding
**EDI**: Electronic Data Interchange
**EOQ**: Economic Order Quantity
**ERP**: Enterprise Resource Planning
**ESG**: Environmental, Social, Governance
**ETA**: Estimated Time of Arrival
**EXW**: Ex Works
**FCA**: Free Carrier
**FOB**: Free On Board
**HS Code**: Harmonized System Code
**Incoterms**: International Commercial Terms
**JIT**: Just-In-Time
**KPI**: Key Performance Indicator
**LCL**: Less than Container Load
**MAPE**: Mean Absolute Percentage Error
**MOQ**: Minimum Order Quantity
**MRP**: Material Requirements Planning
**OTD**: On-Time Delivery
**PPM**: Parts Per Million
**RFP**: Request for Proposal
**RFQ**: Request for Quotation
**RMA**: Return Merchandise Authorization
**RMSE**: Root Mean Squared Error
**ROI**: Return on Investment
**ROP**: Reorder Point
**SKU**: Stock Keeping Unit
**SLA**: Service Level Agreement
**TMS**: Transportation Management System
**UOM**: Unit of Measure
**VMI**: Vendor Managed Inventory
**WMS**: Warehouse Management System

---

## Appendix D: References

1. **ISO 28000:2007** - Specification for security management systems for the supply chain
2. **ISO 9001:2015** - Quality management systems — Requirements
3. **GHG Protocol** - Corporate Accounting and Reporting Standard
4. **INCOTERMS 2020** - ICC official rules for the interpretation of trade terms
5. **GS1 General Specifications** - Global standards for supply chain
6. **WIA Standards Portal** - https://wiastandards.com

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-12-27 | WIA Industry Standards Group | Initial release |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

