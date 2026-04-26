# WIA-AGRI-016: Food Traceability Standard
## PHASE 4: Blockchain & Global Network Integration

**Version:** 1.0
**Status:** Stable
**Last Updated:** 2025-12-26

---

## 1. Overview

PHASE 4 represents the pinnacle of food traceability, implementing blockchain-based immutable records, global multi-enterprise networks, AI-powered optimization, consumer engagement platforms, and automated regulatory compliance. This phase creates a trustless, transparent, and globally interoperable food traceability ecosystem.

### 1.1 Objectives

- Implement blockchain for immutable traceability records
- Create global multi-enterprise traceability networks
- Deploy AI-powered supply chain optimization
- Enable direct consumer engagement
- Automate regulatory compliance and reporting

### 1.2 Scope

PHASE 4 covers:
- Ethereum/Hyperledger blockchain integration
- W3C Verifiable Credentials for certifications
- Decentralized identifiers (DIDs) for all participants
- Global traceability data exchange
- Consumer mobile apps and QR experiences
- Automated regulatory submission
- Carbon footprint and sustainability tracking

---

## 2. Blockchain Integration

### 2.1 Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Application Layer                      │
│  (APIs, Mobile Apps, Web Portals, Analytics)            │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│              Traceability Middleware                     │
│  (Event Processing, Data Validation, Access Control)    │
└────────────────────┬────────────────────────────────────┘
                     │
          ┌──────────┴──────────┐
          ▼                      ▼
┌──────────────────┐   ┌──────────────────┐
│  Off-Chain DB    │   │   Blockchain     │
│  (PostgreSQL,    │   │   (Ethereum,     │
│   MongoDB)       │   │    Hyperledger)  │
│                  │   │                  │
│ • Sensor data    │   │ • Batch hashes   │
│ • Full events    │   │ • Certifications │
│ • Analytics      │   │ • Ownership      │
└──────────────────┘   └──────────────────┘
```

### 2.2 Smart Contract: Batch Registry

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract BatchRegistry {
    struct Batch {
        string batchId;
        bytes32 dataHash;
        address registeredBy;
        uint256 timestamp;
        string ipfsUrl;
        bool exists;
    }

    mapping(string => Batch) public batches;
    mapping(string => bytes32[]) public batchEventHashes;

    event BatchRegistered(
        string indexed batchId,
        bytes32 dataHash,
        address registeredBy,
        uint256 timestamp
    );

    event EventRecorded(
        string indexed batchId,
        bytes32 eventHash,
        uint256 timestamp
    );

    function registerBatch(
        string memory batchId,
        bytes32 dataHash,
        string memory ipfsUrl
    ) public {
        require(!batches[batchId].exists, "Batch already registered");

        batches[batchId] = Batch({
            batchId: batchId,
            dataHash: dataHash,
            registeredBy: msg.sender,
            timestamp: block.timestamp,
            ipfsUrl: ipfsUrl,
            exists: true
        });

        emit BatchRegistered(batchId, dataHash, msg.sender, block.timestamp);
    }

    function recordEvent(
        string memory batchId,
        bytes32 eventHash
    ) public {
        require(batches[batchId].exists, "Batch not found");

        batchEventHashes[batchId].push(eventHash);

        emit EventRecorded(batchId, eventHash, block.timestamp);
    }

    function verifyBatch(
        string memory batchId,
        bytes32 dataHash
    ) public view returns (bool) {
        return batches[batchId].exists &&
               batches[batchId].dataHash == dataHash;
    }

    function getBatchEvents(
        string memory batchId
    ) public view returns (bytes32[] memory) {
        return batchEventHashes[batchId];
    }
}
```

### 2.3 Blockchain Transaction Flow

```javascript
const { ethers } = require('ethers');

class BlockchainTraceability {
  constructor(contractAddress, privateKey, rpcUrl) {
    this.provider = new ethers.JsonRpcProvider(rpcUrl);
    this.wallet = new ethers.Wallet(privateKey, this.provider);
    this.contract = new ethers.Contract(
      contractAddress,
      BatchRegistryABI,
      this.wallet
    );
  }

  async registerBatch(batchData) {
    // Upload full data to IPFS
    const ipfsHash = await this.uploadToIPFS(batchData);

    // Create data hash for on-chain verification
    const dataHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(batchData))
    );

    // Submit to blockchain
    const tx = await this.contract.registerBatch(
      batchData.batchId,
      dataHash,
      `ipfs://${ipfsHash}`
    );

    const receipt = await tx.wait();

    return {
      batchId: batchData.batchId,
      transactionHash: receipt.hash,
      blockNumber: receipt.blockNumber,
      ipfsHash: ipfsHash,
      dataHash: dataHash,
      gasUsed: receipt.gasUsed.toString()
    };
  }

  async recordEvent(batchId, eventData) {
    const eventHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(eventData))
    );

    const tx = await this.contract.recordEvent(batchId, eventHash);
    const receipt = await tx.wait();

    // Store full event data off-chain
    await this.storeOffChain(eventData);

    return {
      eventId: eventData.eventId,
      batchId: batchId,
      transactionHash: receipt.hash,
      eventHash: eventHash
    };
  }

  async verifyBatch(batchId, batchData) {
    const dataHash = ethers.keccak256(
      ethers.toUtf8Bytes(JSON.stringify(batchData))
    );

    const isValid = await this.contract.verifyBatch(batchId, dataHash);

    return {
      batchId: batchId,
      verified: isValid,
      dataHash: dataHash
    };
  }
}
```

---

## 3. Verifiable Credentials

### 3.1 W3C Verifiable Credential Format

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://w3id.org/traceability/v1"
  ],
  "type": ["VerifiableCredential", "OrganicCertificationCredential"],
  "id": "urn:uuid:a1234567-89ab-cdef-0123-456789abcdef",
  "issuer": {
    "id": "did:web:usda.gov",
    "name": "USDA Organic Certification Program"
  },
  "issuanceDate": "2025-01-15T00:00:00Z",
  "expirationDate": "2027-01-15T23:59:59Z",
  "credentialSubject": {
    "id": "did:web:abcfarm.com",
    "name": "ABC Organic Farm",
    "organizationType": "Agricultural Producer",
    "certifications": [
      {
        "type": "USDA Organic",
        "certificationNumber": "ORG-123456",
        "scope": ["Fresh Produce", "Apples", "Pears"],
        "certifiedSince": "2020-01-15",
        "inspectionDate": "2024-12-01",
        "inspector": {
          "name": "Jane Inspector",
          "license": "USDA-INSP-9876"
        }
      }
    ],
    "facilities": [
      {
        "gln": "1234567890123",
        "address": "123 Farm Road, Wenatchee, WA 98801, USA",
        "latitude": 47.7511,
        "longitude": -120.7401
      }
    ]
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T08:00:00Z",
    "verificationMethod": "did:web:usda.gov#key-1",
    "proofPurpose": "assertionMethod",
    "proofValue": "z3FXQjecWRftXMC...base64EncodedSignature..."
  }
}
```

### 3.2 Decentralized Identifiers (DIDs)

```json
{
  "@context": "https://www.w3.org/ns/did/v1",
  "id": "did:web:abcfarm.com",
  "verificationMethod": [
    {
      "id": "did:web:abcfarm.com#key-1",
      "type": "Ed25519VerificationKey2020",
      "controller": "did:web:abcfarm.com",
      "publicKeyMultibase": "z6MkpTHR8VNsBxYAAWHut2Geadd9jSwuBV8xRoAnwWsdvktH"
    }
  ],
  "authentication": ["did:web:abcfarm.com#key-1"],
  "assertionMethod": ["did:web:abcfarm.com#key-1"],
  "service": [
    {
      "id": "did:web:abcfarm.com#traceability",
      "type": "TraceabilityService",
      "serviceEndpoint": "https://api.abcfarm.com/traceability"
    },
    {
      "id": "did:web:abcfarm.com#credentials",
      "type": "CredentialRegistry",
      "serviceEndpoint": "https://api.abcfarm.com/credentials"
    }
  ]
}
```

---

## 4. Global Multi-Enterprise Network

### 4.1 Network Architecture

```
┌─────────────────────────────────────────────────────────┐
│              WIA Global Traceability Network             │
└─────────────────────────────────────────────────────────┘
                          │
         ┌────────────────┼────────────────┐
         ▼                ▼                ▼
    ┌─────────┐      ┌─────────┐      ┌─────────┐
    │ Farmers │      │Processor│      │Retailers│
    │ Network │      │ Network │      │ Network │
    └────┬────┘      └────┬────┘      └────┬────┘
         │                │                │
         └────────────────┼────────────────┘
                          │
              ┌───────────┴───────────┐
              ▼                       ▼
        ┌──────────┐            ┌──────────┐
        │Blockchain│            │ IPFS/    │
        │ Network  │            │ Storage  │
        └──────────┘            └──────────┘
```

### 4.2 Enterprise Node Configuration

```yaml
# WIA Traceability Node Configuration
node:
  id: "node-abcfarm-001"
  type: "producer"
  organization:
    name: "ABC Organic Farm"
    did: "did:web:abcfarm.com"
    gln: "1234567890123"

network:
  consortium: "WIA-Global-Traceability"
  blockchain:
    type: "hyperledger-fabric"
    channel: "food-traceability-channel"
    orderer: "orderer.wia-network.org:7050"
    peers:
      - "peer0.producers.wia-network.org:7051"
      - "peer1.producers.wia-network.org:7051"

  api:
    endpoint: "https://api.abcfarm.com/traceability/v1"
    authentication: "oauth2"
    rateLimit: 1000  # requests per minute

credentials:
  issuer: true
  verifier: true
  supportedTypes:
    - "OrganicCertificationCredential"
    - "HarvestCredential"
    - "BatchOriginCredential"

dataSharing:
  defaultPolicy: "permissioned"
  publicFields:
    - "batchId"
    - "productName"
    - "harvestDate"
    - "certifications"
  privateFields:
    - "pricing"
    - "customerList"
    - "proprietaryProcesses"

compliance:
  regions: ["US", "EU", "KR"]
  regulations:
    - "FDA-FSMA"
    - "EU-178-2002"
    - "KR-Agricultural-Products-Quality-Control-Act"
```

### 4.3 Cross-Enterprise Event Exchange

```javascript
// Secure event sharing between enterprises
class EnterpriseEventExchange {
  async shareEvent(event, recipientDID) {
    // Determine what to share based on policy
    const sharedData = this.filterByPolicy(event, recipientDID);

    // Encrypt sensitive fields
    const encrypted = await this.encryptForRecipient(
      sharedData.privateFields,
      recipientDID
    );

    // Create signed message
    const message = {
      type: "TraceabilityEventShare",
      sender: this.organizationDID,
      recipient: recipientDID,
      timestamp: new Date().toISOString(),
      event: {
        ...sharedData.publicFields,
        encryptedData: encrypted
      },
      proof: await this.signMessage(sharedData)
    };

    // Send via network
    await this.network.send(recipientDID, message);

    // Record hash on blockchain
    await this.recordOnBlockchain({
      eventId: event.eventId,
      hash: this.hashMessage(message),
      sender: this.organizationDID,
      recipient: recipientDID
    });

    return message;
  }

  async receiveEvent(message) {
    // Verify signature
    const isValid = await this.verifySignature(
      message,
      message.sender
    );

    if (!isValid) {
      throw new Error('Invalid signature');
    }

    // Decrypt private data
    const decrypted = await this.decryptData(message.event.encryptedData);

    // Store in local database
    await this.storeEvent({
      ...message.event,
      ...decrypted,
      receivedFrom: message.sender,
      receivedAt: new Date().toISOString()
    });

    return { success: true };
  }
}
```

---

## 5. AI-Powered Supply Chain Optimization

### 5.1 Route Optimization

```python
# AI-based route optimization for minimal quality degradation
import numpy as np
from scipy.optimize import minimize

class SupplyChainOptimizer:
    def __init__(self, product_type, quality_model):
        self.product_type = product_type
        self.quality_model = quality_model

    def optimize_route(self, origin, destinations, constraints):
        """
        Optimize delivery route considering:
        - Transit time
        - Temperature exposure
        - Quality degradation
        - Cost
        """

        def objective(route_params):
            # Unpack parameters
            stops = route_params[:len(destinations)]
            transit_times = route_params[len(destinations):]

            total_cost = 0
            quality_loss = 0

            # Calculate quality degradation
            for i, (stop, time) in enumerate(zip(stops, transit_times)):
                temp_exposure = self.estimate_temperature(stop, time)
                quality_loss += self.quality_model.predict_degradation(
                    time, temp_exposure
                )

                # Add transportation cost
                if i == 0:
                    distance = self.distance(origin, stop)
                else:
                    distance = self.distance(stops[i-1], stop)

                total_cost += distance * constraints['cost_per_km']

            # Penalize quality loss
            total_cost += quality_loss * constraints['quality_penalty']

            return total_cost

        # Optimize
        initial_guess = self.generate_initial_route(origin, destinations)
        result = minimize(
            objective,
            initial_guess,
            method='SLSQP',
            constraints=self.build_constraints(constraints)
        )

        return {
            'optimal_route': result.x[:len(destinations)],
            'transit_times': result.x[len(destinations):],
            'total_cost': result.fun,
            'estimated_quality_retention': self.calculate_quality(result.x)
        }
```

### 5.2 Demand Forecasting

```javascript
// ML-based demand forecasting
class DemandForecaster {
  constructor(model) {
    this.model = model; // TensorFlow.js model
  }

  async forecastDemand(productId, days = 30) {
    // Gather features
    const features = await this.collectFeatures(productId);

    // Features include:
    // - Historical sales (last 90 days)
    // - Seasonality (day of week, month)
    // - Weather forecasts
    // - Promotional calendar
    // - Economic indicators
    // - Social media sentiment

    const input = this.preprocessFeatures(features);

    // Run prediction
    const predictions = await this.model.predict(input);

    return {
      productId: productId,
      forecastPeriod: days,
      predictions: predictions.map((pred, idx) => ({
        date: this.addDays(new Date(), idx),
        expectedDemand: Math.round(pred.demand),
        confidence: pred.confidence,
        recommendedInventory: Math.round(pred.demand * 1.2) // 20% safety stock
      })),
      insights: this.generateInsights(predictions)
    };
  }

  generateInsights(predictions) {
    // Identify trends and patterns
    const avgDemand = predictions.reduce((sum, p) => sum + p.demand, 0) / predictions.length;

    const peakDay = predictions.reduce((max, p) =>
      p.demand > max.demand ? p : max
    );

    return {
      averageDailyDemand: Math.round(avgDemand),
      peakDemandDay: peakDay.date,
      peakDemandValue: Math.round(peakDay.demand),
      trendDirection: this.calculateTrend(predictions),
      recommendations: [
        "Increase production by 15% for peak demand period",
        "Schedule harvest to align with demand forecast",
        "Coordinate with retailers for promotional alignment"
      ]
    };
  }
}
```

---

## 6. Consumer Engagement Platform

### 6.1 Mobile App Features

```javascript
// Consumer mobile app API
class ConsumerTraceabilityApp {
  // Scan QR code to view product history
  async scanProduct(qrData) {
    const batchId = this.extractBatchId(qrData);

    // Fetch traceability data
    const trace = await this.api.get(`/trace/${batchId}`);

    // Verify on blockchain
    const verification = await this.verifyOnBlockchain(batchId, trace);

    return {
      product: {
        name: trace.product.name,
        brand: trace.product.brand,
        imageUrl: trace.product.imageUrl,
        batchId: batchId
      },
      origin: {
        farm: trace.origin.name,
        location: `${trace.origin.city}, ${trace.origin.country}`,
        harvestDate: trace.origin.harvestDate,
        farmer: trace.origin.farmer,
        farmerStory: trace.origin.story,
        photo: trace.origin.photoUrl
      },
      journey: trace.events.map(e => ({
        date: e.timestamp,
        location: e.location.name,
        event: this.formatEventType(e.eventType),
        icon: this.getEventIcon(e.eventType)
      })),
      certifications: trace.certifications.map(c => ({
        name: c.type,
        icon: c.iconUrl,
        verified: c.blockchainVerified
      })),
      sustainability: {
        carbonFootprint: trace.sustainability.co2kg,
        waterUsage: trace.sustainability.waterLiters,
        localSourcing: trace.sustainability.localPercent,
        rating: trace.sustainability.overallRating
      },
      blockchainVerification: {
        verified: verification.success,
        transactionHash: verification.txHash,
        network: "Ethereum Mainnet",
        verifiedAt: new Date().toISOString()
      }
    };
  }

  // Rate and review product
  async submitReview(batchId, review) {
    // Verify purchase (via loyalty program or receipt scan)
    const purchaseVerified = await this.verifyPurchase(batchId, review.userId);

    if (!purchaseVerified) {
      return { error: "Purchase verification required" };
    }

    const reviewData = {
      batchId: batchId,
      userId: review.userId,
      rating: review.rating, // 1-5 stars
      comment: review.comment,
      categories: review.categories, // freshness, taste, packaging
      timestamp: new Date().toISOString(),
      verified: true
    };

    // Store review
    await this.api.post('/reviews', reviewData);

    // Notify producer (if opted in)
    await this.notifyProducer(batchId, reviewData);

    return { success: true, reviewId: reviewData.id };
  }

  // Report quality issue
  async reportIssue(batchId, issue) {
    const report = {
      batchId: batchId,
      userId: issue.userId,
      issueType: issue.type, // quality, safety, packaging
      severity: issue.severity, // low, medium, high, critical
      description: issue.description,
      photos: issue.photos,
      location: await this.getLocation(),
      timestamp: new Date().toISOString()
    };

    // Immediate escalation for safety issues
    if (issue.severity === 'critical') {
      await this.escalateToFoodSafety(report);
    }

    // Store and route to appropriate team
    await this.api.post('/quality-reports', report);

    return {
      success: true,
      reportId: report.id,
      message: "Thank you for your report. We take quality seriously and will investigate."
    };
  }
}
```

### 6.2 Augmented Reality Experience

```javascript
// AR visualization of product journey
class ARTraceabilityExperience {
  async initializeAR(batchId) {
    const trace = await this.getTraceData(batchId);

    // Create 3D journey visualization
    const arScene = {
      type: "TraceabilityJourney",
      nodes: trace.events.map((event, idx) => ({
        id: `node-${idx}`,
        position: this.calculateARPosition(idx, trace.events.length),
        model: this.getLocationModel(event.location.type),
        label: event.location.name,
        date: event.timestamp,
        animation: "fade-in",
        delay: idx * 500
      })),
      connections: this.createConnections(trace.events),
      environment: "farm-to-table",
      backgroundAudio: "gentle-ambience"
    };

    return arScene;
  }
}
```

---

## 7. Automated Regulatory Compliance

### 7.1 FDA FSMA Compliance Report

```javascript
// Automatically generate FDA FSMA compliance reports
async function generateFSMAReport(batchId, incidentType) {
  const trace = await getCompleteTrace(batchId);

  const report = {
    reportType: "FDA FSMA 204",
    batchId: batchId,
    generatedAt: new Date().toISOString(),

    // Critical Tracking Events (CTEs)
    criticalTrackingEvents: trace.events.filter(e =>
      ['harvesting', 'cooling', 'initial_packing', 'shipping', 'receiving', 'transformation'].includes(e.eventType)
    ).map(e => ({
      eventType: e.eventType,
      timestamp: e.timestamp,
      location: {
        gln: e.location.gln,
        address: e.location.address
      },
      quantity: e.quantity,
      traceabilityLotCode: e.batchId
    })),

    // Key Data Elements (KDEs)
    keyDataElements: {
      productDescription: trace.product.name,
      productQuantity: trace.quantity.value,
      productUnit: trace.quantity.unit,
      locationDescription: trace.origin.name,
      originReference: trace.origin.gln,
      harvestDate: trace.origin.harvestDate,
      packingCode: trace.origin.packingCode
    },

    // Traceability Lot Code
    traceabilityLotCode: batchId,

    // 24-hour recall readiness
    recallReadiness: {
      immediateSuppliers: await getImmediateSuppliers(batchId),
      immediateRecipients: await getImmediateRecipients(batchId),
      currentLocations: await getCurrentLocations(batchId),
      estimatedRecoveryTime: "< 2 hours"
    },

    // Digital records
    recordsAvailable: true,
    recordFormat: "JSON + Blockchain",
    recordRetention: "Minimum 2 years",

    // Blockchain proof
    blockchainVerification: {
      network: "Ethereum",
      contractAddress: "0x...",
      transactionHash: trace.blockchainTx,
      immutable: true
    }
  };

  // Submit to FDA FSMA portal (if required)
  if (incidentType === 'recall') {
    await submitToFDA(report);
  }

  return report;
}
```

### 7.2 EU Regulation 178/2002 Compliance

```json
{
  "complianceReport": {
    "regulation": "EU 178/2002",
    "article": "Article 18 - Traceability",
    "batchId": "01234567890128.LOT2025001",
    "product": "Organic Apples",
    "operator": {
      "name": "ABC Organic Farm",
      "registrationNumber": "EU-ORG-123456",
      "address": "123 Farm Road, Wenatchee, WA, USA"
    },
    "oneStepBack": {
      "suppliers": [
        {
          "name": "Seedling Supplier Co",
          "productSupplied": "Organic Apple Seedlings",
          "batchNumber": "SEED-2023-100",
          "deliveryDate": "2023-03-15"
        }
      ]
    },
    "oneStepForward": {
      "recipients": [
        {
          "name": "EuroFresh Distributors",
          "gln": "5432109876543",
          "shipmentDate": "2025-12-10",
          "quantity": "500 kg",
          "destination": "Berlin, Germany"
        }
      ]
    },
    "recordsRetention": "5 years",
    "complianceStatus": "Compliant",
    "lastAudit": "2025-06-15",
    "nextAudit": "2026-06-15"
  }
}
```

---

## 8. Sustainability Tracking

### 8.1 Carbon Footprint Calculation

```javascript
// Calculate carbon footprint for entire supply chain
class CarbonFootprintCalculator {
  async calculateFootprint(batchId) {
    const trace = await getCompleteTrace(batchId);

    let totalCO2 = 0;

    // Farming phase
    const farmingEmissions = this.calculateFarmingEmissions({
      area: trace.origin.fieldArea,
      inputs: trace.origin.inputs, // fertilizer, water, etc.
      equipment: trace.origin.equipmentUsed
    });
    totalCO2 += farmingEmissions;

    // Processing phase
    const processingEmissions = this.calculateProcessingEmissions({
      energy: trace.processing.energyUsed,
      water: trace.processing.waterUsed,
      waste: trace.processing.wasteGenerated
    });
    totalCO2 += processingEmissions;

    // Transportation
    trace.events.filter(e => e.eventType === 'shipping').forEach(shipment => {
      const distance = this.calculateDistance(shipment.origin, shipment.destination);
      const mode = shipment.transportMode; // truck, ship, air

      const transportEmissions = this.calculateTransportEmissions(
        distance,
        mode,
        shipment.quantity
      );

      totalCO2 += transportEmissions;
    });

    // Packaging
    const packagingEmissions = this.calculatePackagingEmissions(
      trace.packaging.materials,
      trace.packaging.weight
    );
    totalCO2 += packagingEmissions;

    return {
      batchId: batchId,
      totalCO2kg: Math.round(totalCO2 * 100) / 100,
      breakdown: {
        farming: farmingEmissions,
        processing: processingEmissions,
        transportation: totalCO2 - farmingEmissions - processingEmissions - packagingEmissions,
        packaging: packagingEmissions
      },
      perUnit: (totalCO2 / trace.quantity.value).toFixed(3),
      benchmark: this.getBenchmark(trace.product.category),
      rating: this.rateFootprint(totalCO2, trace.product.category)
    };
  }
}
```

---

## 9. Implementation Checklist

- [ ] Deploy blockchain infrastructure (Ethereum/Hyperledger)
- [ ] Implement smart contracts for batch registry
- [ ] Set up IPFS nodes for distributed storage
- [ ] Create DID infrastructure for all participants
- [ ] Develop W3C Verifiable Credential system
- [ ] Build global multi-enterprise network
- [ ] Deploy AI optimization models
- [ ] Launch consumer mobile app (iOS/Android)
- [ ] Implement AR experiences
- [ ] Configure automated regulatory reporting
- [ ] Integrate sustainability tracking
- [ ] Conduct network security audits
- [ ] Train all stakeholders on new systems
- [ ] Perform end-to-end testing
- [ ] Go live with phased rollout

---

## 10. Future Enhancements

### 10.1 Roadmap

- **Q1 2026:** Quantum-resistant cryptography
- **Q2 2026:** Integration with satellite imagery for farm monitoring
- **Q3 2026:** DNA-based product authentication
- **Q4 2026:** Global regulatory harmonization platform
- **2027+:** Fully autonomous supply chain orchestration

---

## 11. Conclusion

PHASE 4 represents the complete realization of the WIA-AGRI-016 Food Traceability Standard, creating a transparent, trustless, and globally interoperable ecosystem that benefits all stakeholders from farmers to consumers while ensuring food safety, quality, and sustainability.

---

**Document Control:**
- **Author:** WIA Standards Committee
- **Review Cycle:** Annual
- **Next Review:** 2026-12-26

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity


## Annex E — Implementation Notes for PHASE-4

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-4. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-4 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-4. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P4-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
