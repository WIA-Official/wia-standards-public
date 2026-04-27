# PHASE 2 — API

> Supply-chain API surface: supplier management, procurement
> automation, order management, and the formal API specification
> that consumers code against.

## 7. Supplier Management

### 7.1 Supplier Onboarding

#### 7.1.1 Registration Process

1. **Initial Contact**: Supplier submits registration form
2. **Document Collection**: Collect required documents
   - Business license
   - Tax registration
   - Bank details
   - Insurance certificates
   - Quality certifications
   - Safety certifications
3. **Due Diligence**: Verify supplier information
   - Credit check
   - Reference verification
   - Facility audit (if required)
   - Compliance verification
4. **Risk Assessment**: Evaluate supplier risk profile
5. **Approval**: Final approval by procurement team
6. **Onboarding**: System setup and training

#### 7.1.2 Required Certifications

**Quality Certifications:**
- ISO 9001 (Quality Management)
- ISO/TS 16949 (Automotive)
- AS9100 (Aerospace)
- ISO 13485 (Medical Devices)

**Environmental Certifications:**
- ISO 14001 (Environmental Management)
- ISO 50001 (Energy Management)
- FSC (Forest Stewardship Council)

**Social Certifications:**
- SA8000 (Social Accountability)
- BSCI (Business Social Compliance)
- Fair Trade

**Safety Certifications:**
- ISO 45001 (Occupational Health & Safety)
- OHSAS 18001

**Security Certifications:**
- ISO 28000 (Supply Chain Security)
- ISO 27001 (Information Security)

### 7.2 Supplier Evaluation

#### 7.2.1 Performance Metrics

**Quality Metrics:**
- Defect rate (PPM - parts per million)
- First pass yield
- Return rate
- Warranty claims
- Customer complaints

**Delivery Metrics:**
- On-time delivery rate
- Lead time compliance
- Order fill rate
- Schedule adherence
- Emergency response time

**Cost Metrics:**
- Price competitiveness
- Cost reduction initiatives
- Payment terms compliance
- Currency stability
- Total cost of ownership

**Innovation Metrics:**
- New product development
- Process improvements
- Technology adoption
- Sustainability initiatives
- Collaboration level

**Compliance Metrics:**
- Audit results
- Certification status
- Legal compliance
- Ethical standards
- Environmental compliance

#### 7.2.2 Scorecard Formula

```
Overall Score =
  (Quality × 30%) +
  (Delivery × 25%) +
  (Cost × 20%) +
  (Innovation × 15%) +
  (Compliance × 10%)

Rating Scale:
  4.5 - 5.0: Excellent (Preferred Supplier)
  4.0 - 4.4: Good (Approved Supplier)
  3.0 - 3.9: Acceptable (Conditional Approval)
  2.0 - 2.9: Poor (Improvement Required)
  0.0 - 1.9: Unacceptable (Probation/Termination)
```

### 7.3 Supplier Development

#### 7.3.1 Improvement Programs

- **Quality Improvement**: Six Sigma, Kaizen, Lean Manufacturing
- **Capacity Building**: Training, technology transfer
- **Sustainability**: Carbon reduction, waste minimization
- **Innovation**: Joint product development, process optimization
- **Compliance**: Audit support, certification assistance

#### 7.3.2 Collaboration Models

- **Strategic Partnership**: Long-term collaboration with shared goals
- **Preferred Supplier**: Priority status with volume commitments
- **Approved Supplier**: Standard relationship with regular monitoring
- **Conditional Supplier**: Probationary status with improvement plan
- **Blacklisted**: Terminated relationship due to severe issues

---


## 8. Procurement Automation

### 8.1 Purchase Requisition

#### 8.1.1 Requisition Creation

```json
{
  "requisitionId": "REQ-2025-001234",
  "requestedBy": "user_id",
  "department": "Manufacturing",
  "priority": "normal|urgent|critical",
  "items": [{
    "materialId": "MAT-5678",
    "description": "Industrial Motor 5HP",
    "quantity": 10,
    "uom": "EA",
    "estimatedCost": 1200.00,
    "requiredDate": "2025-12-30",
    "purpose": "Production line maintenance",
    "accountCode": "5000-123-456"
  }],
  "justification": "string",
  "attachments": ["url"],
  "createdAt": "2025-12-27T10:30:00Z"
}
```

#### 8.1.2 Approval Workflow

```
Requisition Created
    ↓
Manager Approval (< $10,000)
    ↓
Department Head Approval (< $50,000)
    ↓
Director Approval (< $100,000)
    ↓
VP/CFO Approval (≥ $100,000)
    ↓
Procurement Processing
    ↓
PO Generation
```

### 8.2 Supplier Selection

#### 8.2.1 RFQ Process

1. **RFQ Creation**: Define requirements and specifications
2. **Supplier Shortlist**: Select qualified suppliers
3. **RFQ Distribution**: Send RFQ to selected suppliers
4. **Bid Collection**: Receive and log supplier bids
5. **Bid Evaluation**: Compare bids across criteria
6. **Negotiation**: Negotiate with top candidates
7. **Award**: Select winning supplier
8. **PO Generation**: Create purchase order

#### 8.2.2 Evaluation Criteria

```
Total Score =
  (Price × 40%) +
  (Quality × 25%) +
  (Delivery × 20%) +
  (Service × 10%) +
  (Sustainability × 5%)
```

### 8.3 Contract Management

#### 8.3.1 Contract Types

- **Blanket PO**: Open-ended order with release schedule
- **Framework Agreement**: Master agreement with call-offs
- **Spot Buy**: One-time purchase
- **Long-term Contract**: Multi-year agreement
- **Consignment**: Supplier-owned inventory on-site

#### 8.3.2 Contract Terms

- **Pricing**: Fixed, variable, tiered, cost-plus
- **Payment**: NET30, NET60, advance payment, progress payments
- **Incoterms**: FOB, CIF, DDP, EXW, FCA
- **Warranties**: Standard, extended, as-is
- **SLA**: Response time, resolution time, uptime
- **Penalties**: Late delivery, quality issues, breach
- **Termination**: Notice period, conditions, penalties

---


## 9. Order Management

### 9.1 Order Lifecycle

```
Draft → Pending Approval → Approved → Sent → Acknowledged →
In Production → Ready to Ship → Shipped → Delivered →
Received → Quality Check → Accepted → Completed
```

### 9.2 Order Status Definitions

- **Draft**: Order created but not submitted
- **Pending Approval**: Awaiting authorization
- **Approved**: Approved and ready to send
- **Sent**: Transmitted to supplier
- **Acknowledged**: Supplier confirmed receipt
- **In Production**: Manufacturing in progress
- **Ready to Ship**: Completed and awaiting pickup
- **Shipped**: In transit to destination
- **Delivered**: Arrived at destination
- **Received**: Physically received and logged
- **Quality Check**: Under inspection
- **Accepted**: Passed inspection, ready for use
- **Completed**: Closed and archived
- **Cancelled**: Order cancelled
- **Disputed**: Issue under resolution

### 9.3 Change Management

#### 9.3.1 Change Order Process

1. **Change Request**: Initiate change (qty, date, specs)
2. **Impact Analysis**: Assess cost and schedule impact
3. **Supplier Approval**: Get supplier confirmation
4. **Internal Approval**: Get management approval
5. **PO Amendment**: Update purchase order
6. **Confirmation**: Supplier acknowledges change
7. **Implementation**: Execute changed order

#### 9.3.2 Cancellation Policy

- **Before Acknowledgement**: Full refund, no penalty
- **After Acknowledgement**: Cancellation fee applies
- **In Production**: Restocking fee + work completed
- **Shipped**: Return shipping + restocking fee

---


## 17. API Specifications

### 17.1 Authentication

#### 17.1.1 API Key Authentication

```http
GET /api/v1/suppliers
Authorization: Bearer <API_KEY>
```

#### 17.1.2 OAuth 2.0

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials
&client_id=<CLIENT_ID>
&client_secret=<CLIENT_SECRET>

Response:
{
  "access_token": "eyJhbG...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

### 17.2 REST API Endpoints

#### 17.2.1 Suppliers

```http
# List suppliers
GET /api/v1/suppliers?page=1&limit=50&status=active

# Get supplier
GET /api/v1/suppliers/{supplierId}

# Create supplier
POST /api/v1/suppliers
Content-Type: application/json
{
  "name": "Supplier Name",
  "tier": 1,
  "address": { },
  ...
}

# Update supplier
PATCH /api/v1/suppliers/{supplierId}

# Get supplier performance
GET /api/v1/suppliers/{supplierId}/performance?period=2025-Q1
```

#### 17.2.2 Purchase Orders

```http
# Create PO
POST /api/v1/orders
{
  "supplierId": "SUP-5678",
  "items": [{
    "sku": "CHIP-A100",
    "quantity": 1000,
    "unitPrice": 45.50
  }],
  ...
}

# Get PO
GET /api/v1/orders/{orderId}

# Update PO status
PATCH /api/v1/orders/{orderId}/status
{
  "status": "approved",
  "approvedBy": "user_123"
}

# List POs
GET /api/v1/orders?status=open&supplierId=SUP-5678
```

#### 17.2.3 Shipments

```http
# Track shipment
GET /api/v1/shipments/{shipmentId}

# Get location
GET /api/v1/shipments/{shipmentId}/location

# List shipments
GET /api/v1/shipments?status=in_transit&carrier=DHL
```

#### 17.2.4 Blockchain Verification

```http
# Verify product
POST /api/v1/verify
{
  "sku": "CHIP-A100",
  "serialNumber": "SN-12345",
  "blockchainHash": "0x7f8c..."
}

# Get provenance
GET /api/v1/provenance/{sku}/{serialNumber}

# Record checkpoint
POST /api/v1/provenance/checkpoint
{
  "sku": "CHIP-A100",
  "serialNumber": "SN-12345",
  "stage": "quality_check",
  "location": "QC Lab",
  "data": { }
}
```

### 17.3 Webhooks

#### 17.3.1 Event Types

- `order.created`
- `order.approved`
- `order.shipped`
- `order.delivered`
- `shipment.in_transit`
- `shipment.delayed`
- `shipment.exception`
- `supplier.risk_change`
- `inventory.low_stock`

#### 17.3.2 Webhook Payload

```json
{
  "event": "shipment.delayed",
  "timestamp": "2025-12-27T14:30:00Z",
  "data": {
    "shipmentId": "SHIP-2025-001234",
    "orderId": "ORD-2025-5678",
    "originalEta": "2025-12-28T10:00:00Z",
    "newEta": "2025-12-29T15:00:00Z",
    "reason": "Weather delay at origin",
    "impact": "24 hour delay"
  },
  "signature": "sha256=..."
}
```

---


