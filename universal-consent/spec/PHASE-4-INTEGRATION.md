# WIA-CORE-002 PHASE 4: Integration Specification

**Version:** 1.0  
**Status:** Stable  
**Last Updated:** January 2025

## Overview

Phase 4 provides comprehensive integration patterns for connecting WIA-CORE-002 to the broader WIA ecosystem, third-party platforms, cloud infrastructure, and regulatory frameworks. This phase enables organizations to achieve full ecosystem-wide consent management.

## WIA Family Integration

### 1. WIA-INTENT Integration

**Consent as User Intent:**

```json
{
  "intentId": "intent-abc123",
  "userId": "user-789012",
  "type": "privacy-preference",
  "expression": {
    "consentPreferences": {
      "marketing": "opt-out",
      "analytics": "opt-in-anonymous",
      "personalization": "opt-in-full",
      "dataSharing": "minimal"
    },
    "dataRetention": "minimal",
    "thirdPartySharing": "prohibited",
    "automatedDecisionMaking": "opt-out"
  },
  "appliesTo": ["all-services"],
  "wiaCore002ConsentId": "consent-550e8400...",
  "effectiveFrom": "2025-01-15T10:30:00Z"
}
```

### 2. WIA-OMNI-API Integration

**API Access Control with Consent:**

```http
GET /api/omni/user/789012/profile
Authorization: Bearer {token}
X-WIA-Consent-Check: required
X-WIA-Purpose: profile-access
```

**Response Headers:**
```
X-WIA-Consent-Status: verified
X-WIA-Consent-Id: consent-550e8400...
X-WIA-Purposes: profile-access,analytics
X-WIA-Expiry: 2026-01-15T10:30:00Z
```

**Response Body:**
```json
{
  "userId": "user-789012",
  "profile": {
    "name": "John Doe",
    "email": "john@example.com"
  },
  "consentCompliance": {
    "verified": true,
    "consentId": "consent-550e8400...",
    "purposes": ["profile-access", "analytics"],
    "dataCategories": ["contact-info", "preferences"]
  }
}
```

### 3. WIA-SOCIAL Integration

**Social Platform Consent Coordination:**

```json
{
  "socialAction": "share-profile",
  "fromUser": "user-789012",
  "toUser": "user-456789",
  "platform": "WIA-SOCIAL",
  "consentCheck": {
    "required": true,
    "purposes": ["social-sharing", "profile-visibility"],
    "consentId": "consent-550e8400...",
    "verified": true,
    "grantedAt": "2025-01-15T10:30:00Z"
  },
  "privacyLevel": "friends-only",
  "dataShared": ["name", "profile-picture"],
  "timestamp": "2025-06-20T14:30:00Z"
}
```

### 4. WIA-AIR-SHIELD Integration

**Enhanced Consent Security:**

```json
{
  "dataType": "consent-record",
  "securityLevel": "high",
  "encryption": {
    "algorithm": "AES-256-GCM",
    "keyManagement": "WIA-AIR-SHIELD-KMS",
    "keyId": "key-abc123"
  },
  "accessControl": {
    "readPermissions": ["user-self", "privacy-officer", "regulator"],
    "writePermissions": ["user-self", "authorized-admin"],
    "deletePermissions": ["user-self", "compliance-officer"],
    "auditAllAccess": true
  },
  "integrityProtection": {
    "hashAlgorithm": "SHA-256",
    "signatureRequired": true,
    "timestampRequired": true
  },
  "dataResidency": {
    "allowedRegions": ["EU", "EEA"],
    "encryptionAtRest": true,
    "encryptionInTransit": true
  }
}
```

## Third-Party Platform Integrations

### Consent Management Platforms (CMPs)

#### OneTrust Integration

```javascript
// Bidirectional sync
const oneTrust = new OneTrustConnector({
  apiKey: process.env.ONETRUST_KEY,
  wiaConsentSync: true
});

// Import from OneTrust
await oneTrust.importConsents({
  format: 'WIA-CORE-002',
  mapping: {
    'OneTrust.Marketing': 'marketing-email',
    'OneTrust.Analytics': 'analytics'
  }
});

// Export to OneTrust
await oneTrust.exportConsents({
  consentIds: ['consent-1', 'consent-2'],
  format: 'OneTrust-v3'
});
```

#### TrustArc Integration

```javascript
const trustArc = new TrustArcConnector({
  apiKey: process.env.TRUSTARC_KEY
});

// Real-time sync
trustArc.on('consent.updated', async (event) => {
  await wiaConsent.update({
    consentId: event.consentId,
    purposes: event.purposes
  });
});
```

### Marketing Automation Platforms

#### HubSpot Integration

```javascript
const hubspot = new HubSpotConnector({
  apiKey: process.env.HUBSPOT_KEY,
  consentSync: true
});

// When WIA consent changes, update HubSpot
wiaConsent.on('consent.updated', async (event) => {
  await hubspot.updateContactConsent({
    email: event.userId,
    purposes: event.purposes,
    source: 'WIA-CORE-002',
    legalBasis: event.legalBasis
  });
});

// Respect consent in campaigns
const campaign = await hubspot.createCampaign({
  name: 'Summer Sale 2025',
  consentCheck: {
    required: true,
    purposeId: 'marketing-email',
    standard: 'WIA-CORE-002'
  }
});
```

#### Salesforce Integration

```apex
// Apex trigger for consent synchronization
trigger ConsentSync on Contact (after update) {
    List<WIA_Consent__c> consentsToUpdate = new List<WIA_Consent__c>();
    
    for (Contact c : Trigger.new) {
        if (c.HasOptedOutOfEmail != Trigger.oldMap.get(c.Id).HasOptedOutOfEmail) {
            WIA_Consent__c consent = new WIA_Consent__c();
            consent.Contact__c = c.Id;
            consent.Purpose_Id__c = 'marketing-email';
            consent.Granted__c = !c.HasOptedOutOfEmail;
            consent.Updated_At__c = System.now();
            consentsToUpdate.add(consent);
        }
    }
    
    if (!consentsToUpdate.isEmpty()) {
        WIAConsentAPI.syncConsents(consentsToUpdate);
    }
}
```

### Customer Data Platforms (CDPs)

#### Segment Integration

```javascript
// Segment destination for WIA-CORE-002
analytics.ready(() => {
  // Attach consent to every track call
  const originalTrack = analytics.track;
  analytics.track = async function(...args) {
    const consent = await wiaConsent.verify({
      userId: analytics.user().id(),
      purposeId: 'analytics'
    });
    
    if (consent.isValid) {
      args[1] = args[1] || {};
      args[1].consent = {
        standard: 'WIA-CORE-002',
        consentId: consent.consentId,
        purposes: consent.purposes
      };
      return originalTrack.apply(this, args);
    } else {
      console.warn('Analytics blocked: no valid consent');
    }
  };
});
```

## Cloud Platform Integrations

### AWS Integration

#### Lambda Function for Consent Verification

```python
# AWS Lambda handler
import json
import boto3
from wia_consent import ConsentClient

consent_client = ConsentClient(
    api_key=os.environ['WIA_API_KEY']
)

def lambda_handler(event, context):
    user_id = event['userId']
    purpose_id = event['purposeId']
    
    # Verify consent
    consent = consent_client.verify(
        user_id=user_id,
        purpose_id=purpose_id
    )
    
    if not consent['isValid']:
        return {
            'statusCode': 403,
            'body': json.dumps({
                'error': 'Consent required for this operation',
                'purposeId': purpose_id
            })
        }
    
    # Proceed with data processing
    result = process_data(event['data'])
    
    return {
        'statusCode': 200,
        'body': json.dumps(result)
    }
```

#### AWS EventBridge Integration

```yaml
# EventBridge rule for consent events
Resources:
  ConsentEventRule:
    Type: AWS::Events::Rule
    Properties:
      Name: wia-consent-sync
      EventPattern:
        source:
          - wia.consent
        detail-type:
          - Consent Updated
          - Consent Revoked
      Targets:
        - Arn: !GetAtt ConsentSyncFunction.Arn
          Id: ConsentSyncTarget
```

### Azure Integration

#### Azure Policy for Consent-Based Access

```json
{
  "policyName": "require-consent-for-pii",
  "type": "Microsoft.Authorization/policyDefinitions",
  "properties": {
    "mode": "All",
    "parameters": {
      "wiaConsentVerification": {
        "type": "String",
        "metadata": {
          "displayName": "WIA Consent Verification Endpoint"
        }
      }
    },
    "policyRule": {
      "if": {
        "field": "tags.dataClassification",
        "equals": "PII"
      },
      "then": {
        "effect": "audit",
        "details": {
          "type": "Microsoft.Compliance/consentVerification",
          "existenceCondition": {
            "field": "Microsoft.Compliance/consentVerification/status",
            "equals": "verified"
          }
        }
      }
    }
  }
}
```

### Google Cloud Integration

#### Cloud Functions for Consent Management

```javascript
// Google Cloud Function
const functions = require('@google-cloud/functions-framework');
const {ConsentClient} = require('@wia/consent-sdk');

const consentClient = new ConsentClient({
  apiKey: process.env.WIA_API_KEY
});

functions.http('checkConsent', async (req, res) => {
  const {userId, purposeId} = req.body;
  
  const consent = await consentClient.verify({
    userId,
    purposeId
  });
  
  res.json({
    allowed: consent.isValid,
    consentId: consent.consentId,
    expiresAt: consent.expiresAt
  });
});
```

## Regulatory Compliance Mapping

### GDPR Compliance

| GDPR Article | WIA-CORE-002 Implementation |
|--------------|----------------------------|
| Article 7: Conditions for consent | `purposes[].granted`, `auditTrail` |
| Article 13: Information to be provided | `metadata.privacyPolicyUrl` |
| Article 15: Right of access | `GET /api/v1/users/{userId}/consents` |
| Article 16: Right to rectification | `PATCH /api/v1/consents/{id}` |
| Article 17: Right to erasure | `DELETE /api/v1/consents/{id}` |
| Article 18: Right to restriction | `status: "suspended"` |
| Article 20: Right to data portability | `GET /api/v1/users/{userId}/consents/export` |
| Article 21: Right to object | `POST /api/v1/consents/{id}/revoke` |
| Article 30: Records of processing | Complete consent record with audit trail |

### CCPA Compliance

| CCPA Right | WIA-CORE-002 Implementation |
|------------|----------------------------|
| Right to Know | Complete consent record retrieval |
| Right to Delete | Consent revocation and data deletion |
| Right to Opt-Out of Sale | `purposes[].granted = false` for "sale" purposes |
| Right to Non-Discrimination | No service denial based on consent |
| Authorized Agent | Delegated access via API keys |

## Migration Pathways

### From Legacy Systems

```javascript
// Migration script
const migration = {
  async migrateConsents() {
    // 1. Extract from legacy system
    const legacyConsents = await legacyDb.query(`
      SELECT user_id, marketing_opt_in, analytics_opt_in, created_at
      FROM user_preferences
    `);
    
    // 2. Transform to WIA-CORE-002 format
    const wiaConsents = legacyConsents.map(legacy => ({
      userId: legacy.user_id,
      purposes: [
        {
          purposeId: 'marketing-email',
          granted: legacy.marketing_opt_in === 1
        },
        {
          purposeId: 'analytics',
          granted: legacy.analytics_opt_in === 1
        }
      ],
      jurisdiction: 'US',
      legalBasis: 'consent',
      metadata: {
        source: 'legacy-migration',
        legacyCreatedAt: legacy.created_at
      }
    }));
    
    // 3. Validate and import
    for (const consent of wiaConsents) {
      const validation = await wiaConsent.validate(consent);
      if (validation.isValid) {
        await wiaConsent.create(consent);
      } else {
        console.error('Invalid consent:', validation.errors);
      }
    }
  }
};
```

## Certification Program

### Certification Levels

| Level | Requirements | Annual Fee |
|-------|-------------|-----------|
| Bronze | Phase 1 (Data Format) | $500 |
| Silver | Phases 1-2 (Data + API) | $2,000 |
| Gold | Phases 1-3 (Data + API + Protocol) | $5,000 |
| Platinum | All Phases + Ecosystem Integration | $10,000 |

### Certification Process

1. **Self-Assessment:** Complete questionnaire
2. **Automated Testing:** Run conformance test suite
3. **Documentation Review:** Submit required documentation
4. **Security Audit:** (Gold/Platinum only)
5. **Third-Party Verification:** (Gold/Platinum only)
6. **Certificate Issuance:** Receive digital certificate
7. **Annual Recertification:** Maintain compliance

### Conformance Testing

```bash
# Install test suite
npm install -g @wia/consent-conformance-tests

# Run tests
wia-consent-test \
  --api-url https://api.example.com \
  --api-key $WIA_API_KEY \
  --level platinum \
  --output results.json

# Results:
# ✓ Phase 1 Data Format: 45/45 tests passed
# ✓ Phase 2 API Interface: 78/78 tests passed
# ✓ Phase 3 Protocol: 34/34 tests passed
# ✓ Phase 4 Integration: 23/23 tests passed
# Overall: 180/180 (100%)
```

## Open Source Tools

WIA provides open-source tools to accelerate implementation:

- **wia-consent-validator** - JSON schema validator
- **wia-consent-sdk** - Official SDKs in 7+ languages
- **wia-consent-ui** - Pre-built React/Vue consent UIs
- **wia-consent-migrator** - Migration tools from legacy systems
- **wia-consent-simulator** - Testing and simulation environment

---

**Previous:** [PHASE 3: Protocol](PHASE-3-PROTOCOL.md)

© 2025 SmileStory Inc. / WIA · 弘益人間 (Benefit All Humanity)
