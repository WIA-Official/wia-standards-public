# Chapter 7: System Integration

## Overview

This chapter provides guidance on integrating the WIA KYC/AML Standard into existing technology ecosystems, including core banking systems, RegTech platforms, identity verification providers, and compliance tools.

---

## Integration Architecture Patterns

### 1. Hub-and-Spoke Model

```
                    ┌─────────────────┐
                    │   WIA KYC/AML   │
                    │  Central Hub    │
                    └────────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
   ┌────▼────┐         ┌────▼────┐         ┌────▼────┐
   │  Core   │         │Identity │         │Screening│
   │ Banking │         │ Verify  │         │ Service │
   └─────────┘         └─────────┘         └─────────┘
```

**Use Case:** Financial institution with multiple vendor services
**Benefits:**
- Single integration point for each system
- Centralized data management
- Simplified vendor changes
- Consistent data formats

**Implementation:**
```typescript
// Hub configuration
const kycHub = new WIAKYCHub({
  coreBanking: new CoreBankingAdapter('bank-system'),
  identityProvider: new IdentityAdapter('jumio'),
  screeningProvider: new ScreeningAdapter('dow-jones'),
  transactionMonitoring: new MonitoringAdapter('actimize')
});

// Unified workflow
async function onboardCustomer(application) {
  // Create customer in hub
  const customer = await kycHub.customers.create(application);

  // Hub orchestrates all integrations
  const verification = await kycHub.identity.verify(customer.id);
  const screening = await kycHub.screening.perform(customer.id);
  const risk = await kycHub.risk.assess(customer.id);

  // Hub syncs to core banking
  await kycHub.sync.toCoreBanking(customer.id);

  return customer;
}
```

### 2. Microservices Architecture

```
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  Customer    │    │ Verification │    │  Screening   │
│  Service     │    │  Service     │    │  Service     │
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       └───────────────────┼───────────────────┘
                           │
                    ┌──────▼───────┐
                    │ Event Bus    │
                    │ (Kafka/RabbitMQ)
                    └──────────────┘
```

**Use Case:** Cloud-native financial institutions
**Benefits:**
- Scalability
- Independent deployment
- Fault isolation
- Technology diversity

**Implementation:**
```typescript
// Customer service publishes events
class CustomerService {
  async createCustomer(data) {
    const customer = await this.db.customers.create(data);

    await this.eventBus.publish('customer.created', {
      customerId: customer.id,
      type: customer.type,
      riskIndicators: this.extractRiskIndicators(customer)
    });

    return customer;
  }
}

// Verification service subscribes
class VerificationService {
  constructor() {
    this.eventBus.subscribe('customer.created', this.handleNewCustomer);
  }

  async handleNewCustomer(event) {
    const verification = await this.performVerification(event.customerId);

    await this.eventBus.publish('verification.completed', {
      customerId: event.customerId,
      result: verification.result,
      confidenceScore: verification.confidenceScore
    });
  }
}

// Screening service subscribes
class ScreeningService {
  constructor() {
    this.eventBus.subscribe('customer.created', this.handleNewCustomer);
    this.eventBus.subscribe('verification.completed', this.checkIfReady);
  }

  async handleNewCustomer(event) {
    await this.performScreening(event.customerId);
  }
}
```

### 3. API Gateway Pattern

```
         ┌─────────────────┐
         │   API Gateway   │
         │  (Kong/Apigee)  │
         └────────┬────────┘
                  │
    ┌─────────────┼─────────────┐
    │             │             │
┌───▼───┐   ┌────▼────┐   ┌───▼───┐
│  KYC  │   │   AML   │   │ Case  │
│  API  │   │   API   │   │ Mgmt  │
└───────┘   └─────────┘   └───────┘
```

**Use Case:** Multiple client applications (web, mobile, partner integrations)
**Benefits:**
- Unified entry point
- Authentication/authorization
- Rate limiting
- Request routing

---

## Core Banking System Integration

### Integration Points

```
┌─────────────────────────────────────────┐
│        Core Banking System              │
│                                         │
│  ┌─────────────┐    ┌─────────────┐   │
│  │   Customer  │    │   Account   │   │
│  │   Master    │◄───┤   Opening   │   │
│  │             │    │             │   │
│  └──────┬──────┘    └─────────────┘   │
│         │                              │
└─────────┼──────────────────────────────┘
          │
          ▼
   ┌──────────────┐
   │  WIA KYC/AML │
   │   Adapter    │
   └──────┬───────┘
          │
   ┌──────▼───────┐
   │  WIA API     │
   └──────────────┘
```

### Data Synchronization

**Bidirectional Sync Strategy:**

**From Core Banking to KYC/AML:**
```json
{
  "syncType": "customer_update",
  "timestamp": "2025-01-09T10:30:00Z",
  "source": "core_banking",
  "data": {
    "customerId": "CUST-789012",
    "updates": {
      "contactInfo": {
        "phone": "+1-555-9999"
      },
      "addresses": [
        {
          "type": "residential",
          "street": "456 New Address St",
          "city": "Seattle",
          "state": "WA",
          "postalCode": "98101",
          "country": "USA"
        }
      ]
    }
  }
}
```

**From KYC/AML to Core Banking:**
```json
{
  "syncType": "risk_assessment_update",
  "timestamp": "2025-01-09T10:35:00Z",
  "source": "kyc_aml_system",
  "data": {
    "customerId": "CUST-789012",
    "riskProfile": {
      "category": "medium",
      "score": 45,
      "assessmentDate": "2025-01-09T10:30:00Z",
      "nextReviewDate": "2026-01-09"
    },
    "transactionLimits": {
      "dailyWithdrawal": 10000,
      "monthlyInternationalWire": 100000
    },
    "flags": {
      "enhancedMonitoring": true,
      "manualApprovalRequired": false
    }
  }
}
```

### Integration Patterns

#### 1. Real-Time API Integration

```typescript
// Core banking triggers KYC on account opening
class AccountOpeningService {
  async openAccount(customerId: string, accountType: string) {
    // Check KYC status
    const kycStatus = await wiaKYC.customers.get(customerId);

    if (kycStatus.status !== 'verified') {
      throw new Error('KYC verification required');
    }

    if (kycStatus.riskProfile.category === 'high') {
      // Require manual approval
      return this.createPendingAccount(customerId, accountType);
    }

    // Proceed with account opening
    const account = await this.createAccount(customerId, accountType);

    // Apply risk-based limits
    await this.applyLimits(account.id, kycStatus.riskProfile);

    return account;
  }
}
```

#### 2. Batch Synchronization

```typescript
// Nightly batch sync for updates
class BatchSyncService {
  async syncDailyUpdates() {
    const date = new Date();

    // Get customers updated in core banking
    const updatedCustomers = await coreBanking.getUpdatedCustomers({
      since: this.getLastSyncTime(),
      until: date
    });

    // Sync to KYC system
    for (const customer of updatedCustomers) {
      try {
        await wiaKYC.customers.update(customer.id, customer.changes);
        await this.logSync(customer.id, 'success');
      } catch (error) {
        await this.logSync(customer.id, 'failed', error);
        await this.queueForRetry(customer.id);
      }
    }

    // Get risk updates from KYC system
    const riskUpdates = await wiaKYC.risk.getUpdatedAssessments({
      since: this.getLastSyncTime(),
      until: date
    });

    // Sync to core banking
    for (const update of riskUpdates) {
      await coreBanking.updateCustomerRisk(update.customerId, update.riskProfile);
      await coreBanking.updateTransactionLimits(update.customerId, update.limits);
    }

    this.setLastSyncTime(date);
  }
}
```

#### 3. Event-Driven Integration

```typescript
// Core banking emits events, KYC system reacts
coreBanking.on('customer.address_changed', async (event) => {
  // Update address in KYC system
  await wiaKYC.customers.update(event.customerId, {
    addresses: event.newAddress
  });

  // Trigger re-verification if significant change
  if (event.countryChanged) {
    await wiaKYC.identity.reverify(event.customerId);
  }

  // Re-assess risk due to geographic change
  await wiaKYC.risk.reassess(event.customerId);
});

// KYC system emits events, core banking reacts
wiaKYC.on('risk.category_changed', async (event) => {
  // Update risk in core banking
  await coreBanking.updateCustomerRisk(
    event.customerId,
    event.newCategory
  );

  // Adjust limits based on new risk
  if (event.newCategory === 'high') {
    await coreBanking.reduceTransactionLimits(event.customerId);
  }

  // Flag for relationship manager review
  if (event.direction === 'increased') {
    await coreBanking.createReviewTask(event.customerId);
  }
});
```

---

## Identity Verification Provider Integration

### Supported Providers

| Provider | Document Verification | Biometric | Database Checks |
|----------|----------------------|-----------|-----------------|
| Jumio | ✅ | ✅ | ✅ |
| Onfido | ✅ | ✅ | ✅ |
| Trulioo | ❌ | ❌ | ✅ |
| IDology | ❌ | ❌ | ✅ |
| LexisNexis | ❌ | ❌ | ✅ |
| Experian | ❌ | ❌ | ✅ |

### Provider Adapter Pattern

```typescript
// Standard interface
interface IdentityVerificationProvider {
  verifyDocument(customerId: string, document: Document): Promise<VerificationResult>;
  verifyBiometric(customerId: string, biometric: Biometric): Promise<BiometricResult>;
  checkDatabase(customerId: string, data: PersonalData): Promise<DatabaseResult>;
}

// Jumio adapter
class JumioAdapter implements IdentityVerificationProvider {
  private client: JumioClient;

  async verifyDocument(customerId: string, document: Document) {
    // Call Jumio API
    const jumioResult = await this.client.initiateNetverify({
      customerInternalReference: customerId,
      userReference: customerId,
      successUrl: `${this.callbackUrl}/success`,
      errorUrl: `${this.callbackUrl}/error`
    });

    // Transform to WIA format
    return this.transformJumioResult(jumioResult);
  }

  private transformJumioResult(jumioResult: any): VerificationResult {
    return {
      verificationId: jumioResult.transactionReference,
      status: this.mapStatus(jumioResult.status),
      overallResult: jumioResult.verificationStatus === 'APPROVED_VERIFIED' ? 'pass' : 'fail',
      confidenceScore: jumioResult.similarity / 100,
      documentVerification: {
        authenticity: jumioResult.documentVerification.status,
        dataExtraction: jumioResult.extractedData,
        // ... map other fields
      }
    };
  }
}

// Onfido adapter
class OnfidoAdapter implements IdentityVerificationProvider {
  private client: OnfidoClient;

  async verifyDocument(customerId: string, document: Document) {
    // Call Onfido API
    const applicant = await this.client.applicant.create({
      first_name: document.firstName,
      last_name: document.lastName,
      email: document.email
    });

    const check = await this.client.check.create({
      applicant_id: applicant.id,
      report_names: ['document', 'facial_similarity_photo']
    });

    // Transform to WIA format
    return this.transformOnfidoResult(check);
  }

  // ... similar transformation logic
}

// Factory to select provider
class IdentityVerificationFactory {
  static createProvider(providerName: string): IdentityVerificationProvider {
    switch (providerName) {
      case 'jumio':
        return new JumioAdapter(config.jumio);
      case 'onfido':
        return new OnfidoAdapter(config.onfido);
      default:
        throw new Error(`Unknown provider: ${providerName}`);
    }
  }
}

// Usage
const provider = IdentityVerificationFactory.createProvider('jumio');
const result = await provider.verifyDocument(customerId, document);
```

### Webhook Handling

```typescript
// Unified webhook handler
class IdentityWebhookHandler {
  async handleWebhook(provider: string, payload: any, signature: string) {
    // Verify webhook signature
    if (!this.verifySignature(provider, payload, signature)) {
      throw new Error('Invalid webhook signature');
    }

    // Route to appropriate handler
    switch (provider) {
      case 'jumio':
        return this.handleJumioWebhook(payload);
      case 'onfido':
        return this.handleOnfidoWebhook(payload);
      default:
        throw new Error(`Unknown provider: ${provider}`);
    }
  }

  private async handleJumioWebhook(payload: any) {
    const verificationId = payload.transactionReference;
    const customerId = payload.customerInternalReference;

    // Update verification status in WIA system
    await wiaKYC.identity.updateVerification(verificationId, {
      status: this.mapJumioStatus(payload.status),
      result: payload.verificationStatus,
      completedAt: new Date(payload.timestamp)
    });

    // Trigger next steps if verification complete
    if (payload.status === 'DONE') {
      await this.triggerNextSteps(customerId, verificationId);
    }
  }

  private async triggerNextSteps(customerId: string, verificationId: string) {
    const verification = await wiaKYC.identity.getVerification(verificationId);

    if (verification.overallResult === 'pass') {
      // Proceed with screening
      await wiaKYC.screening.perform(customerId);
      // Assess risk
      await wiaKYC.risk.assess(customerId);
    } else {
      // Handle verification failure
      await wiaKYC.customers.updateStatus(customerId, 'verification_failed');
      await this.notifyCustomer(customerId, 'verification_failed');
    }
  }
}
```

---

## Screening Provider Integration

### Provider Comparison

| Provider | Sanctions | PEP | Adverse Media | Coverage | Update Frequency |
|----------|-----------|-----|---------------|----------|------------------|
| Dow Jones | ✅ | ✅ | ✅ | Global | Real-time |
| Refinitiv (World-Check) | ✅ | ✅ | ✅ | Global | Daily |
| LexisNexis | ✅ | ✅ | ✅ | Global | Daily |
| ComplyAdvantage | ✅ | ✅ | ✅ | Global | Real-time |

### Screening Adapter

```typescript
interface ScreeningProvider {
  screen(customer: Customer, options: ScreeningOptions): Promise<ScreeningResult>;
  getWatchlists(): Promise<Watchlist[]>;
  enableContinuousMonitoring(customerId: string): Promise<void>;
}

class DowJonesAdapter implements ScreeningProvider {
  async screen(customer: Customer, options: ScreeningOptions) {
    const searchRequest = {
      searchTerm: customer.personalInfo.fullName,
      dateOfBirth: customer.personalInfo.dateOfBirth,
      country: customer.personalInfo.nationality[0],
      // ... other fields
    };

    const response = await this.client.search(searchRequest);

    return this.transformResults(response);
  }

  private transformResults(djResponse: any): ScreeningResult {
    return {
      screeningId: djResponse.caseId,
      executedAt: new Date(),
      sanctionsScreening: {
        status: this.hasMatches(djResponse, 'SANCTION') ? 'match' : 'no_match',
        matches: this.extractMatches(djResponse, 'SANCTION')
      },
      pepScreening: {
        status: this.hasMatches(djResponse, 'PEP') ? 'match' : 'no_match',
        matches: this.extractMatches(djResponse, 'PEP')
      },
      adverseMediaScreening: {
        status: this.hasMatches(djResponse, 'ADVERSE_MEDIA') ? 'match' : 'no_match',
        articles: this.extractAdverseMedia(djResponse)
      }
    };
  }
}

// Multi-provider screening (for redundancy or coverage)
class MultiProviderScreening {
  private providers: ScreeningProvider[];

  async screenWithConsensus(customer: Customer) {
    // Screen with all providers in parallel
    const results = await Promise.all(
      this.providers.map(p => p.screen(customer, {}))
    );

    // Aggregate results
    return this.aggregateResults(results);
  }

  private aggregateResults(results: ScreeningResult[]): ScreeningResult {
    // Combine matches from all providers
    const allMatches = results.flatMap(r => r.sanctionsScreening.matches || []);

    // Deduplicate
    const uniqueMatches = this.deduplicateMatches(allMatches);

    // Return consolidated result
    return {
      screeningId: `MULTI-${Date.now()}`,
      executedAt: new Date(),
      sanctionsScreening: {
        status: uniqueMatches.length > 0 ? 'match' : 'no_match',
        matches: uniqueMatches
      },
      // ... other fields
    };
  }
}
```

---

## Transaction Monitoring System Integration

### Integration with Existing TMS

```typescript
// Adapter for existing TMS (e.g., Actimize, FICO, SAS)
class TransactionMonitoringAdapter {
  // Push transactions from core banking to TMS
  async sendTransaction(transaction: Transaction) {
    // Transform to WIA format
    const wiaTransaction = this.transformToWIA(transaction);

    // Send to WIA TM system
    await wiaKYC.monitoring.submitTransaction(wiaTransaction);

    // Also send to legacy TMS during transition period
    if (this.isTransitionMode()) {
      await this.legacyTMS.sendTransaction(transaction);
    }
  }

  // Receive alerts from TMS
  async handleAlert(alert: any) {
    // Transform alert to WIA format
    const wiaAlert = this.transformAlertToWIA(alert);

    // Create case in WIA system
    const case = await wiaKYC.cases.create({
      caseType: 'suspicious_activity_investigation',
      customerId: wiaAlert.customerId,
      trigger: {
        type: 'transaction_monitoring_alert',
        sourceId: wiaAlert.alertId
      }
    });

    return case;
  }

  // Sync disposition back to TMS
  async syncDisposition(caseId: string, disposition: string) {
    const case = await wiaKYC.cases.get(caseId);

    // Update legacy TMS
    await this.legacyTMS.updateAlertStatus(
      case.trigger.sourceId,
      this.mapDispositionToTMS(disposition)
    );
  }
}
```

---

## Document Management Integration

### Document Storage Options

**1. Cloud Storage (AWS S3, Azure Blob, GCP Storage)**

```typescript
class CloudDocumentStorage {
  private s3: S3Client;

  async storeDocument(customerId: string, document: File): Promise<string> {
    // Encrypt document
    const encrypted = await this.encrypt(document);

    // Generate secure file name
    const fileName = `${customerId}/${document.type}/${uuidv4()}.enc`;

    // Upload to S3
    await this.s3.putObject({
      Bucket: 'kyc-documents',
      Key: fileName,
      Body: encrypted,
      ServerSideEncryption: 'AES256',
      Metadata: {
        customerId,
        documentType: document.type,
        uploadedAt: new Date().toISOString()
      }
    });

    // Return secure URL
    return `s3://kyc-documents/${fileName}`;
  }

  async retrieveDocument(documentUrl: string): Promise<Buffer> {
    // Parse S3 URL
    const { bucket, key } = this.parseS3Url(documentUrl);

    // Download from S3
    const response = await this.s3.getObject({
      Bucket: bucket,
      Key: key
    });

    const encrypted = await response.Body.transformToByteArray();

    // Decrypt
    return this.decrypt(encrypted);
  }
}
```

**2. DMS Integration (FileNet, Documentum, SharePoint)**

```typescript
class DMSAdapter {
  async storeDocument(customerId: string, document: File): Promise<string> {
    // Create folder structure if not exists
    await this.ensureFolder(`/Customers/${customerId}/KYC`);

    // Store in DMS
    const docId = await this.dmsClient.createDocument({
      folder: `/Customers/${customerId}/KYC`,
      fileName: `${document.type}_${Date.now()}.pdf`,
      content: document.buffer,
      metadata: {
        customerId,
        documentType: document.type,
        uploadedAt: new Date(),
        classification: 'confidential'
      }
    });

    return docId;
  }
}
```

---

## Audit and Compliance Tools Integration

### Audit Trail Export

```typescript
class AuditTrailExporter {
  async exportForRegulator(dateRange: DateRange, format: 'csv' | 'xlsx' | 'json') {
    // Fetch all relevant activities
    const activities = await wiaKYC.audit.getActivities({
      from: dateRange.start,
      to: dateRange.end,
      includeTypes: [
        'customer.created',
        'verification.completed',
        'screening.performed',
        'risk.assessed',
        'alert.generated',
        'case.created',
        'sar.filed'
      ]
    });

    // Format for export
    const formatted = activities.map(activity => ({
      timestamp: activity.timestamp,
      activityType: activity.type,
      customerId: activity.customerId,
      performedBy: activity.performedBy,
      details: JSON.stringify(activity.details),
      ipAddress: activity.ipAddress
    }));

    // Export in requested format
    switch (format) {
      case 'csv':
        return this.convertToCSV(formatted);
      case 'xlsx':
        return this.convertToExcel(formatted);
      case 'json':
        return JSON.stringify(formatted, null, 2);
    }
  }
}
```

### Regulatory Reporting Integration

```typescript
class RegulatoryReportingService {
  // Generate SAR in FinCEN format
  async generateFinCENSAR(caseId: string): Promise<string> {
    const case = await wiaKYC.cases.get(caseId);
    const sar = await wiaKYC.cases.getSAR(caseId);

    // Transform to FinCEN XML format
    const fincenXML = this.transformToFinCENFormat(sar);

    // Validate against FinCEN schema
    await this.validateFinCENXML(fincenXML);

    // Submit to FinCEN BSA E-Filing System
    const confirmationNumber = await this.submitToFinCEN(fincenXML);

    // Update case with confirmation
    await wiaKYC.cases.update(caseId, {
      'resolution.sarConfirmation': confirmationNumber
    });

    return confirmationNumber;
  }

  // Generate CTR report
  async generateCTR(transactions: Transaction[]): Promise<CTRReport> {
    // Aggregate transactions
    const totalAmount = transactions.reduce((sum, t) => sum + t.amount, 0);

    // Create CTR
    const ctr = {
      reportDate: new Date(),
      transactions,
      totalAmount,
      customer: await this.getCustomerInfo(transactions[0].customerId),
      institution: this.getInstitutionInfo()
    };

    // Submit to regulator
    await this.submitCTR(ctr);

    return ctr;
  }
}
```

---

## Key Takeaways

1. 🏗️ **Multiple architecture patterns** - Hub-and-spoke, microservices, API gateway
2. 🔄 **Bidirectional sync** with core banking systems
3. 🔌 **Provider adapters** for identity verification and screening
4. 📊 **Transaction monitoring** integration patterns
5. 📁 **Document storage** options (cloud, DMS)
6. 📋 **Audit and reporting** export capabilities
7. 🌐 **Event-driven** and real-time integration support

---

**Previous**: [← Chapter 6 - Protocol](06-protocol.md) | **Next**: [Chapter 8 - Implementation →](08-implementation.md)

---

© 2025 WIA Standards Committee
弘益人間 (홍익인간) - Benefit All Humanity
