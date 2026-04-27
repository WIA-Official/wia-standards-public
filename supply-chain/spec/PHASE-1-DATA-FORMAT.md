# PHASE 1 — Data Format

> Supply-chain canonical data models: supplier, purchase order,
> shipment, lot, traceability record, and the architecture map
> that hosts the procurement, logistics, and traceability
> pipelines.

## Abstract

The WIA-IND-023 Supply Chain Standard defines a comprehensive framework for managing end-to-end supply chain operations across global networks. This standard provides data models, APIs, protocols, and best practices for supplier management, procurement automation, order tracking, blockchain-based traceability, risk assessment, demand planning, logistics optimization, and sustainability tracking.

**弘益人間 (Benefit All Humanity)** - This standard aims to create transparent, efficient, and sustainable supply chains that benefit all stakeholders from raw material suppliers to end consumers.

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Architecture Overview](#5-architecture-overview)
6. [Data Models](#6-data-models)
7. [Supplier Management](#7-supplier-management)
8. [Procurement Automation](#8-procurement-automation)
9. [Order Management](#9-order-management)
10. [Shipment Tracking](#10-shipment-tracking)
11. [Blockchain Traceability](#11-blockchain-traceability)
12. [Supply Chain Visibility](#12-supply-chain-visibility)
13. [Risk Management](#13-risk-management)
14. [Demand Planning](#14-demand-planning)
15. [Logistics Optimization](#15-logistics-optimization)
16. [Sustainability Tracking](#16-sustainability-tracking)
17. [API Specifications](#17-api-specifications)
18. [Security and Privacy](#18-security-and-privacy)
19. [Integration Patterns](#19-integration-patterns)
20. [Compliance and Certification](#20-compliance-and-certification)

---


## 1. Introduction

### 1.1 Purpose

The WIA-IND-023 standard provides a unified framework for supply chain management that enables:

- **Transparency**: Complete visibility across all tiers of the supply chain
- **Automation**: Streamlined procurement and order processing
- **Traceability**: Immutable product provenance using blockchain technology
- **Optimization**: AI-powered demand forecasting and route optimization
- **Sustainability**: Comprehensive carbon footprint and ESG tracking
- **Risk Mitigation**: Proactive identification and management of supply chain risks

### 1.2 Design Principles

1. **Interoperability**: Compatible with existing ERP, WMS, and TMS systems
2. **Scalability**: Support for small businesses to global enterprises
3. **Modularity**: Adopt individual components or full stack
4. **Real-time**: Live tracking and instant notifications
5. **Data-driven**: Analytics and AI for predictive insights
6. **Security**: End-to-end encryption and access control
7. **Sustainability**: Built-in carbon accounting and ESG compliance

### 1.3 Benefits

**For Manufacturers:**
- Reduced lead times and inventory costs
- Improved supplier relationships
- Better demand forecasting
- Enhanced quality control

**For Suppliers:**
- Streamlined order processing
- Predictable demand patterns
- Performance visibility
- Fair evaluation metrics

**For Distributors:**
- Optimized routing and warehousing
- Real-time inventory visibility
- Reduced shipping costs
- Improved delivery accuracy

**For Retailers:**
- Product authenticity verification
- Faster restocking
- Better customer experience
- Sustainability transparency

**For Consumers:**
- Product provenance verification
- Ethical sourcing confidence
- Sustainability information
- Quality assurance

---


## 2. Scope

### 2.1 In Scope

This standard covers:

- Supplier onboarding, evaluation, and management
- Purchase order creation, approval, and tracking
- Multi-tier supplier visibility
- Real-time shipment tracking across all modes
- Blockchain-based product provenance
- Supplier risk assessment and mitigation
- Demand forecasting and inventory optimization
- Route optimization and cost reduction
- Carbon footprint calculation and ESG scoring
- API specifications for system integration
- Data formats and exchange protocols

### 2.2 Out of Scope

The following are not covered by this standard:

- Internal manufacturing operations (see WIA-MFG standards)
- Warehouse management systems (see WIA-WMS standards)
- Financial accounting and payments (see WIA-FIN standards)
- Human resources management (see WIA-HR standards)
- Product design and development (see WIA-PLM standards)

---


## 3. Normative References

The following standards and specifications are referenced in this document:

- **ISO 28000** - Supply chain security management
- **ISO 9001** - Quality management systems
- **ISO 14001** - Environmental management systems
- **GS1 Standards** - Global supply chain standards
- **EDIFACT** - Electronic data interchange
- **GHG Protocol** - Greenhouse gas accounting
- **WIA-BLOCKCHAIN** - Blockchain traceability standard
- **WIA-API** - API design and security standard
- **WIA-IOT** - Internet of Things integration standard
- **INCOTERMS 2020** - International commercial terms
- **HS Code** - Harmonized commodity description and coding system

---


## 4. Terms and Definitions

### 4.1 Supply Chain Terms

**Supply Chain**: The network of organizations involved in producing and delivering a product from raw materials to end customer.

**Tier 1 Supplier**: Direct supplier to the manufacturer or buyer.

**Tier 2 Supplier**: Supplier to Tier 1 supplier (indirect supplier).

**Lead Time**: Time between order placement and delivery.

**Safety Stock**: Extra inventory held to prevent stockouts.

**Reorder Point**: Inventory level triggering new purchase order.

**Economic Order Quantity (EOQ)**: Optimal order quantity minimizing total costs.

**Fill Rate**: Percentage of customer demand met from available stock.

**Perfect Order**: Order delivered complete, on-time, damage-free, with correct documentation.

**Cash-to-Cash Cycle**: Time from paying suppliers to receiving payment from customers.

### 4.2 Logistics Terms

**Incoterms**: International commercial terms defining shipping responsibilities.

**Bill of Lading (BOL)**: Document detailing shipment contents and terms.

**Customs Broker**: Agent facilitating customs clearance.

**Drayage**: Short-distance transport of goods.

**Cross-Docking**: Direct transfer from inbound to outbound without warehousing.

**Last Mile**: Final delivery leg to end destination.

**Freight Forwarder**: Company arranging storage and shipping on behalf of shippers.

### 4.3 Risk Terms

**Supply Chain Risk**: Potential disruption to supply chain operations.

**Single Source Risk**: Dependency on one supplier for critical items.

**Geopolitical Risk**: Risk from political instability or trade restrictions.

**Force Majeure**: Unforeseeable circumstances preventing contract fulfillment.

**Business Continuity Plan (BCP)**: Strategy for maintaining operations during disruptions.

### 4.4 Sustainability Terms

**Carbon Footprint**: Total greenhouse gas emissions caused by an activity.

**Scope 1/2/3 Emissions**: Direct, indirect energy, and value chain emissions.

**ESG**: Environmental, Social, and Governance factors.

**Circular Economy**: Economic system aimed at eliminating waste.

**Life Cycle Assessment (LCA)**: Environmental impact analysis across product lifecycle.

---


## 5. Architecture Overview

### 5.1 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   WIA Supply Chain Platform                  │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Supplier   │  │ Procurement  │  │    Order     │      │
│  │  Management  │  │  Automation  │  │  Management  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Shipment   │  │  Blockchain  │  │   Supply     │      │
│  │   Tracking   │  │ Traceability │  │ Chain Viz    │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                               │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │     Risk     │  │    Demand    │  │  Logistics   │      │
│  │  Management  │  │   Planning   │  │ Optimization │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                               │
│  ┌──────────────────────────────────────────────────┐       │
│  │        Sustainability & ESG Tracking             │       │
│  └──────────────────────────────────────────────────┘       │
│                                                               │
├─────────────────────────────────────────────────────────────┤
│                      Integration Layer                        │
│   REST API │ GraphQL │ Webhooks │ Event Streams             │
├─────────────────────────────────────────────────────────────┤
│                       Data Layer                              │
│   RDBMS │ Document DB │ Time Series │ Blockchain            │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Component Responsibilities

#### Supplier Management
- Supplier onboarding and qualification
- Performance tracking and scorecarding
- Certification management
- Contract management
- Supplier risk profiling

#### Procurement Automation
- Automated RFQ/RFP generation
- Bid comparison and evaluation
- Approval workflow automation
- Contract generation
- Purchase order automation

#### Order Management
- Order creation and validation
- Multi-tier order visibility
- Order status tracking
- Change order management
- Invoice reconciliation

#### Shipment Tracking
- Real-time location tracking
- Multi-modal transport support
- ETA prediction
- Exception management
- Delivery confirmation

#### Blockchain Traceability
- Product provenance recording
- Authenticity verification
- Tamper-proof audit trails
- Smart contract integration
- NFT-based certificates

#### Supply Chain Visibility
- Multi-tier network mapping
- Real-time status dashboards
- Bottleneck identification
- Collaboration tools
- Document sharing

#### Risk Management
- Risk factor identification
- Risk scoring and prioritization
- Mitigation strategy generation
- Alternative supplier recommendations
- Continuous monitoring

#### Demand Planning
- Historical trend analysis
- Seasonal pattern detection
- Machine learning forecasting
- What-if scenario modeling
- Collaborative planning

#### Logistics Optimization
- Route optimization
- Load planning
- Carrier selection
- Cost optimization
- Carbon-efficient routing

#### Sustainability Tracking
- Carbon footprint calculation
- ESG metric collection
- Compliance monitoring
- Sustainability reporting
- Circular economy tracking

---


## 6. Data Models

### 6.1 Core Entities

#### 6.1.1 Supplier

```json
{
  "id": "string",
  "name": "string",
  "legalName": "string",
  "tier": 1-5,
  "status": "active|inactive|pending|suspended|blacklisted",
  "address": {
    "street": "string",
    "city": "string",
    "state": "string",
    "postalCode": "string",
    "country": "string",
    "countryCode": "string",
    "coordinates": {
      "latitude": "number",
      "longitude": "number"
    }
  },
  "contacts": [{
    "name": "string",
    "email": "string",
    "phone": "string",
    "role": "string",
    "isPrimary": "boolean"
  }],
  "rating": "number (0-5)",
  "certifications": ["ISO9001", "ISO14001", ...],
  "financialScore": "number (0-100)",
  "esgScore": "number (0-100)",
  "riskScore": "number (0-100)",
  "categories": ["string"],
  "paymentTerms": "string",
  "leadTimeDays": "number",
  "moq": "number",
  "currency": "string",
  "contractStart": "ISO 8601 datetime",
  "contractEnd": "ISO 8601 datetime",
  "createdAt": "ISO 8601 datetime",
  "updatedAt": "ISO 8601 datetime"
}
```

#### 6.1.2 Purchase Order

```json
{
  "id": "string",
  "poNumber": "string",
  "supplier": {
    "id": "string",
    "name": "string",
    "address": { }
  },
  "buyer": {
    "id": "string",
    "name": "string",
    "address": { }
  },
  "status": "draft|pending_approval|approved|sent|acknowledged|in_production|ready_to_ship|shipped|delivered|completed|cancelled|disputed",
  "items": [{
    "lineNumber": "number",
    "sku": "string",
    "description": "string",
    "quantity": "number",
    "uom": "string",
    "unitPrice": "number",
    "lineTotal": "number",
    "requestedDate": "ISO 8601 datetime",
    "confirmedDate": "ISO 8601 datetime",
    "tax": "number",
    "discount": "number",
    "category": "string",
    "hsCode": "string",
    "countryOfOrigin": "string",
    "blockchainHash": "string"
  }],
  "subtotal": "number",
  "tax": "number",
  "shipping": "number",
  "total": "number",
  "currency": "string",
  "paymentTerms": "string",
  "deliveryAddress": { },
  "requestedDeliveryDate": "ISO 8601 datetime",
  "confirmedDeliveryDate": "ISO 8601 datetime",
  "incoterms": "string",
  "notes": "string",
  "createdBy": "string",
  "approvedBy": "string",
  "createdAt": "ISO 8601 datetime",
  "updatedAt": "ISO 8601 datetime",
  "shipmentIds": ["string"]
}
```

#### 6.1.3 Shipment

```json
{
  "id": "string",
  "trackingNumber": "string",
  "purchaseOrderId": "string",
  "carrier": {
    "id": "string",
    "name": "string",
    "service": "string"
  },
  "mode": "air|sea|road|rail|multimodal",
  "status": "pending|picked_up|in_transit|customs_clearance|out_for_delivery|delivered|exception|returned|cancelled",
  "origin": {
    "address": { },
    "departureDate": "ISO 8601 datetime"
  },
  "destination": {
    "address": { },
    "arrivalDate": "ISO 8601 datetime"
  },
  "currentLocation": {
    "name": "string",
    "coordinates": { },
    "timestamp": "ISO 8601 datetime"
  },
  "eta": "ISO 8601 datetime",
  "actualDelivery": "ISO 8601 datetime",
  "events": [{
    "timestamp": "ISO 8601 datetime",
    "type": "string",
    "location": { },
    "description": "string",
    "handledBy": "string",
    "temperature": "number",
    "humidity": "number",
    "damageNoted": "boolean",
    "notes": "string"
  }],
  "packages": [{
    "packageId": "string",
    "weight": "number",
    "weightUnit": "string",
    "dimensions": {
      "length": "number",
      "width": "number",
      "height": "number",
      "unit": "string"
    },
    "contents": "string"
  }],
  "customs": {
    "declarationNumber": "string",
    "value": "number",
    "currency": "string",
    "cleared": "boolean",
    "clearedDate": "ISO 8601 datetime"
  },
  "insurance": {
    "provider": "string",
    "value": "number",
    "currency": "string",
    "policyNumber": "string"
  },
  "temperatureControl": {
    "required": "boolean",
    "minTemp": "number",
    "maxTemp": "number",
    "unit": "C|F",
    "currentTemp": "number"
  },
  "createdAt": "ISO 8601 datetime",
  "updatedAt": "ISO 8601 datetime"
}
```

#### 6.1.4 Product Provenance

```json
{
  "sku": "string",
  "serialNumber": "string",
  "network": "ethereum|polygon|hyperledger|private",
  "contractAddress": "string",
  "tokenId": "string",
  "origin": {
    "manufacturer": "string",
    "location": { },
    "date": "ISO 8601 datetime"
  },
  "journey": [{
    "id": "string",
    "stage": "origin|manufacturing|quality_check|packaging|warehouse|shipping|distribution|retail|consumer",
    "timestamp": "ISO 8601 datetime",
    "location": { },
    "txHash": "string",
    "blockNumber": "number",
    "data": { },
    "verifiedBy": "string",
    "certifications": ["string"]
  }],
  "currentOwner": "string",
  "isAuthentic": "boolean",
  "verifiedAt": "ISO 8601 datetime",
  "carbonFootprint": "number",
  "sustainabilityCerts": ["string"]
}
```

---


