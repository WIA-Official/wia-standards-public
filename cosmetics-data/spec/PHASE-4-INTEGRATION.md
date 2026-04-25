# WIA-IND-005: Phase 4 - Integration Specification

**Version:** 1.0
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity

## Overview

Phase 4 defines integration patterns for connecting WIA-IND-005 with enterprise systems (ERP, PLM), regulatory databases (FDA, EU CosIng), supply chain platforms, and consumer-facing applications. This specification enables seamless adoption across the cosmetics ecosystem.

## Enterprise System Integration

### ERP Integration

#### SAP S/4HANA Integration

**Integration Architecture:**

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  SAP S/4HANA    │────▶│  WIA Connector   │────▶│  WIA-IND-005    │
│  (ERP System)   │◀────│  (Middleware)    │◀────│  API            │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

**Integration Points:**

1. **Material Master (MM)**
   - Sync ingredient data with SAP material records
   - Update safety ratings and regulatory status
   - Link INCI names to SAP material numbers

2. **Quality Management (QM)**
   - Import CoA data into QM inspection lots
   - Trigger alerts for out-of-specification materials
   - Track batch quality history

3. **Product Lifecycle Management (PLM)**
   - Sync formulation data
   - Manage formulation versioning
   - Track regulatory approvals

**Configuration Example:**

```json
{
  "integration": {
    "type": "SAP_S4HANA",
    "connection": {
      "host": "sap.acmecosmetics.com",
      "port": 443,
      "protocol": "OData V4",
      "authentication": {
        "type": "OAuth2",
        "client_id": "wia_connector",
        "token_endpoint": "https://sap.acmecosmetics.com/oauth/token"
      }
    },
    "mappings": {
      "inci_to_material": {
        "source": "WIA.inci_name",
        "target": "SAP.MARA.MATNR",
        "transformation": "lookup_material_master"
      },
      "safety_rating": {
        "source": "WIA.safety.ewg_score",
        "target": "SAP.MARA.ZZ_SAFETY_SCORE",
        "custom_field": true
      }
    },
    "sync_frequency": "hourly",
    "philosophy": "弘益人間 - Seamless enterprise integration"
  }
}
```

#### Oracle ERP Cloud Integration

**Integration Method:** REST API with Oracle Integration Cloud (OIC)

```javascript
// Oracle Integration Cloud Flow
const oicFlow = {
  trigger: "Scheduled (Daily at 2:00 AM)",
  actions: [
    {
      type: "InvokeWIAAPI",
      endpoint: "https://api.wia-standards.org/v1/cosmetics/ingredients/updates",
      params: { since: "lastSyncTimestamp" }
    },
    {
      type: "TransformData",
      mapping: "WIA_to_Oracle_Item_Master"
    },
    {
      type: "InvokeOracleERP",
      operation: "UpdateItems",
      endpoint: "/fscmRestApi/resources/11.13.18.05/items"
    }
  ]
};
```

### PLM Integration

#### Siemens Teamcenter Integration

**Use Case:** Formulation Management

```xml
<!-- Teamcenter Item Revision with WIA Data -->
<ItemRevision id="FORMULA-HYD-SERUM-001-A">
  <Properties>
    <Property name="wia_product_id">PROD-12345</Property>
    <Property name="wia_safety_score">9.2</Property>
    <Property name="wia_compliant_markets">US,EU,JP,KR,CN</Property>
  </Properties>
  <Attachments>
    <Attachment type="WIA_Safety_Assessment" ref="SAFETY-12345.json"/>
    <Attachment type="WIA_Regulatory_Report" ref="REG-12345.pdf"/>
  </Attachments>
  <Relations>
    <Relation type="WIA_Ingredient_BOM">
      <Item ref="ING-AQUA"/>
      <Item ref="ING-GLYCERIN"/>
      <Item ref="ING-NIACINAMIDE"/>
    </Relation>
  </Relations>
</ItemRevision>
```

## Regulatory Database Integration

### EU CosIng Integration

**Integration Type:** Automated Synchronization

```python
# Python Integration Script
from wia_cosmetics import RegulatorySync

sync_engine = RegulatorySync(
    wia_api_key="your_api_key",
    target_database="EU_COSING"
)

# Daily sync of ingredient updates
updates = sync_engine.fetch_cosing_updates(since="2025-01-01")

for update in updates:
    # Update WIA database
    sync_engine.update_wia_ingredient(
        inci_name=update['inci_name'],
        eu_status=update['status'],
        restrictions=update['restrictions'],
        max_concentration=update['max_concentration']
    )

    # Notify affected manufacturers
    sync_engine.notify_stakeholders(
        ingredient=update['inci_name'],
        change_type=update['change_type'],
        effective_date=update['effective_date']
    )
```

**Data Mapping:**

| CosIng Field | WIA-IND-005 Field | Transformation |
|--------------|-------------------|----------------|
| INCI Name | inci_name | Direct mapping |
| CAS No | cas_number | Direct mapping |
| Function | function.primary | Enum conversion |
| Restrictions | regulatory.eu_approved | Boolean logic |
| Max Conc | regulatory.max_concentration.leave_on | Percentage conversion |

### FDA VCRP Integration

**Integration Method:** API Integration with FDA VCRP System

```json
{
  "fda_vcrp_registration": {
    "establishment": {
      "fei_number": "1234567890",
      "name": "Acme Cosmetics Manufacturing",
      "address": "123 Industrial Blvd, Los Angeles, CA 90001"
    },
    "product": {
      "brand_name": "Hydrating Serum",
      "formulation_code": "PROD-12345",
      "wia_reference": "https://api.wia-standards.org/v1/cosmetics/products/PROD-12345",
      "ingredients": [
        {
          "inci": "Aqua",
          "vcrp_code": "VCRP-AQUA-001",
          "percentage_range": "60-70%"
        }
      ],
      "product_category": "Skin Care - Moisturizer",
      "intended_use": "Leave-on facial moisturizer"
    },
    "wia_integration": {
      "auto_update": true,
      "sync_frequency": "monthly",
      "notify_changes": true
    }
  }
}
```

### NMPA (China) Integration

**Registration Process Integration:**

```typescript
// TypeScript Integration for NMPA Registration
interface NMPARegistration {
  productId: string;
  productType: 'ordinary' | 'special_use';
  wiaData: {
    ingredients: Ingredient[];
    safetyAssessment: SafetyAssessment;
    formulation: Formulation;
  };
  nmpaRequirements: {
    animalTestingRequired: boolean;
    clinicalTrialRequired: boolean;
    efficacyTestingRequired: boolean;
  };
}

async function submitNMPARegistration(registration: NMPARegistration) {
  // Generate NMPA-compliant documentation from WIA data
  const nmpaDocuments = await generateNMPADocuments(registration.wiaData);

  // Submit to NMPA system
  const submission = await nmpaApi.submitRegistration({
    productId: registration.productId,
    documents: nmpaDocuments,
    attachments: [
      'safety_assessment_report.pdf',
      'ingredient_specifications.pdf',
      'manufacturing_gmp_certificate.pdf'
    ]
  });

  return submission;
}
```

## Supply Chain Integration

### Blockchain Integration

#### Ethereum Smart Contract

```solidity
// SPDX-License-Identifier: MIT
pragma solidity ^0.8.0;

contract CosmeticsSupplyChain {
    struct Ingredient {
        string inciName;
        string batchNumber;
        address supplier;
        uint256 timestamp;
        string wiaReference;
        bool certified;
    }

    mapping(bytes32 => Ingredient) public ingredients;

    event IngredientRegistered(
        bytes32 indexed ingredientHash,
        string inciName,
        string batchNumber,
        string wiaReference
    );

    function registerIngredient(
        string memory _inciName,
        string memory _batchNumber,
        string memory _wiaReference
    ) public returns (bytes32) {
        bytes32 ingredientHash = keccak256(
            abi.encodePacked(_inciName, _batchNumber, block.timestamp)
        );

        ingredients[ingredientHash] = Ingredient({
            inciName: _inciName,
            batchNumber: _batchNumber,
            supplier: msg.sender,
            timestamp: block.timestamp,
            wiaReference: _wiaReference,
            certified: true
        });

        emit IngredientRegistered(
            ingredientHash,
            _inciName,
            _batchNumber,
            _wiaReference
        );

        return ingredientHash;
    }

    // Philosophy: 弘益人間 - Transparent supply chain for all
}
```

#### VeChain Integration

```javascript
// VeChain Thor Integration
const { Framework } = require('@vechain/connex-framework');
const { Driver, SimpleNet } = require('@vechain/connex-driver');

async function trackBatch(batchData) {
  const driver = await Driver.connect(new SimpleNet('https://mainnet.veblocks.net'));
  const connex = new Framework(driver);

  // Create batch tracking transaction
  const clause = connex.thor.account(CONTRACT_ADDRESS).method({
    "inputs": [
      {"name": "batchNumber", "type": "string"},
      {"name": "productId", "type": "string"},
      {"name": "wiaReference", "type": "string"}
    ],
    "name": "trackBatch",
    "outputs": []
  }).asClause(
    batchData.batchNumber,
    batchData.productId,
    `https://api.wia-standards.org/v1/cosmetics/batches/${batchData.batchNumber}`
  );

  // Sign and send transaction
  const txResponse = await connex.vendor.sign('tx', [clause])
    .signer(SIGNER_ADDRESS)
    .request();

  return txResponse.txid;
}
```

### IoT Integration

#### Smart Packaging Integration

```json
{
  "iot_device": {
    "device_id": "IOT-PKG-12345",
    "device_type": "Smart NFC Tag",
    "product_id": "PROD-12345",
    "batch_number": "B2025-001234",
    "wia_endpoint": "https://api.wia-standards.org/v1/cosmetics/products/PROD-12345",
    "sensors": {
      "temperature": {
        "current": "22°C",
        "min": "18°C",
        "max": "25°C",
        "alerts": []
      },
      "humidity": {
        "current": "45%",
        "threshold": "60%"
      },
      "tampering": {
        "status": "sealed",
        "first_opened": null
      }
    },
    "consumer_scan_count": 0,
    "philosophy": "弘益人間 - Smart packaging for consumer safety"
  }
}
```

## Consumer Application Integration

### Mobile App SDK

#### iOS (Swift) SDK

```swift
import WIACosmetics

class ProductScanViewController: UIViewController {
    let wiaSDK = WIACosmetics.SDK(apiKey: "your_api_key")

    func scanProduct(qrCode: String) {
        wiaSDK.products.scan(qrCode: qrCode) { result in
            switch result {
            case .success(let product):
                self.displayProductInfo(product)
                self.checkAllergens(product, userProfile: self.userProfile)
            case .failure(let error):
                self.showError(error)
            }
        }
    }

    func checkAllergens(_ product: Product, userProfile: UserProfile) {
        let allergenCheck = wiaSDK.safety.checkAllergens(
            ingredients: product.ingredients,
            userAllergens: userProfile.allergens
        )

        if allergenCheck.hasAllergens {
            showAllergenAlert(allergenCheck.allergens)
        }
    }
}
```

#### Android (Kotlin) SDK

```kotlin
import org.wia.cosmetics.WIACosmetics
import org.wia.cosmetics.models.*

class ProductScanActivity : AppCompatActivity() {
    private val wiaSDK = WIACosmetics(apiKey = "your_api_key")

    suspend fun scanProduct(qrCode: String) {
        try {
            val product = wiaSDK.products.scan(qrCode)
            displayProductInfo(product)

            val safetyCheck = wiaSDK.safety.analyze(
                ingredients = product.ingredients,
                userProfile = getUserProfile()
            )

            if (!safetyCheck.isSafe) {
                showSafetyWarning(safetyCheck.concerns)
            }
        } catch (e: WIAException) {
            showError(e.message)
        }
    }
}
```

### Web Application Integration

#### React Integration

```typescript
import { useWIACosmetics } from '@wia/cosmetics-react';

function ProductDetailsPage({ productId }: { productId: string }) {
  const { product, loading, error } = useWIACosmetics({
    productId,
    includeIngredients: true,
    includeSafety: true
  });

  if (loading) return <LoadingSpinner />;
  if (error) return <ErrorMessage error={error} />;

  return (
    <div>
      <h1>{product.name}</h1>
      <SafetyRating score={product.safetyScore} />
      <IngredientList ingredients={product.ingredients} />
      <AllergenWarnings allergens={product.allergens} />
      <p>弘益人間 - Transparent product information for all</p>
    </div>
  );
}
```

## Integration Security

### API Gateway Configuration

```yaml
apiVersion: networking.istio.io/v1beta1
kind: Gateway
metadata:
  name: wia-cosmetics-gateway
spec:
  selector:
    istio: ingressgateway
  servers:
  - port:
      number: 443
      name: https
      protocol: HTTPS
    tls:
      mode: SIMPLE
      credentialName: wia-cosmetics-cert
    hosts:
    - "api.wia-standards.org"
---
apiVersion: security.istio.io/v1beta1
kind: AuthorizationPolicy
metadata:
  name: wia-api-authz
spec:
  selector:
    matchLabels:
      app: wia-cosmetics-api
  rules:
  - from:
    - source:
        requestPrincipals: ["*"]
    to:
    - operation:
        methods: ["GET", "POST"]
        paths: ["/v1/cosmetics/*"]
    when:
    - key: request.auth.claims[aud]
      values: ["wia-cosmetics-api"]
```

### Data Protection

#### Encryption at Rest

```json
{
  "encryption": {
    "provider": "AWS KMS",
    "algorithm": "AES-256-GCM",
    "key_rotation": "90 days",
    "data_classification": {
      "public": ["ingredient_names", "safety_ratings"],
      "confidential": ["formulations", "supplier_info"],
      "restricted": ["user_health_data", "api_keys"]
    }
  },
  "philosophy": "弘益人間 - Protecting sensitive data"
}
```

## Monitoring and Observability

### Integration Metrics

```yaml
metrics:
  - name: integration_requests_total
    type: counter
    labels: [integration_type, status, endpoint]

  - name: integration_latency_seconds
    type: histogram
    labels: [integration_type, endpoint]
    buckets: [0.1, 0.5, 1.0, 2.5, 5.0, 10.0]

  - name: integration_errors_total
    type: counter
    labels: [integration_type, error_type]

  - name: sync_last_success_timestamp
    type: gauge
    labels: [integration_type, source, destination]
```

### Health Checks

```http
GET /health/integrations
Authorization: Bearer {monitoring_token}

Response:
{
  "status": "healthy",
  "integrations": {
    "sap_s4hana": {
      "status": "connected",
      "last_sync": "2025-01-15T10:30:00Z",
      "sync_lag_seconds": 120
    },
    "eu_cosing": {
      "status": "connected",
      "last_sync": "2025-01-15T10:25:00Z",
      "sync_lag_seconds": 420
    },
    "blockchain_ethereum": {
      "status": "connected",
      "last_block": 12345678
    }
  },
  "philosophy": "弘益人間 - Reliable integrations for all"
}
```

## Philosophy Statement

**弘益人間 (Benefit All Humanity):** Integration specifications are designed to be vendor-neutral and technology-agnostic, preventing lock-in and promoting innovation. Whether integrating with enterprise SAP systems or building consumer mobile apps, the same principles apply: accessibility, transparency, and safety for all stakeholders. Open integration patterns enable even small manufacturers to compete globally.

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active
**Philosophy:** 弘益人間 - Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for cosmetics-data is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/cosmetics-data/api/` — TypeScript SDK skeleton
- `wia-standards/standards/cosmetics-data/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/cosmetics-data/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
