# WIA-DIGITAL-ERASURE PHASE 3 — Protocol Specification

**Standard:** WIA-DIGITAL-ERASURE
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** consolidated from `PHASE-2-API-INTERFACE.md` Code Examples + Security sections

---

## Code Examples

### 7.1 JavaScript/Node.js - Create Erasure Request

```javascript
const axios = require('axios');

const WIA_API_BASE = 'https://api.wia.live/digital-erasure/v1';
const ACCESS_TOKEN = 'your_access_token';

async function createErasureRequest() {
  try {
    const response = await axios.post(
      `${WIA_API_BASE}/erasure-requests`,
      {
        decedent: {
          deathCertificateId: 'DC-KR-2025-001',
          dateOfDeath: '2025-12-01T00:00:00Z'
        },
        executor: {
          name: 'Jane Doe',
          email: 'executor@example.com',
          authenticationDocument: 'base64_encoded_document'
        },
        requestType: 'complete_erasure',
        legalBasis: 'gdpr_article_17',
        targetAccounts: [
          {
            platform: 'facebook',
            accountIdentifier: 'user@example.com',
            requestedAction: 'permanent_deletion'
          }
        ],
        deletionMethod: {
          algorithm: 'crypto_shred',
          passes: 7,
          verificationRequired: true
        }
      },
      {
        headers: {
          'Authorization': `Bearer ${ACCESS_TOKEN}`,
          'Content-Type': 'application/json'
        }
      }
    );

    console.log('Erasure request created:', response.data.requestId);
    console.log('Status:', response.data.status);
    return response.data;
  } catch (error) {
    console.error('Error creating erasure request:', error.response?.data);
    throw error;
  }
}

createErasureRequest();
```

### 7.2 Python - Monitor Deletion Status

```python
import requests
import time

WIA_API_BASE = 'https://api.wia.live/digital-erasure/v1'
ACCESS_TOKEN = 'your_access_token'

def monitor_erasure_status(request_id):
    headers = {
        'Authorization': f'Bearer {ACCESS_TOKEN}',
        'Content-Type': 'application/json'
    }

    while True:
        response = requests.get(
            f'{WIA_API_BASE}/erasure-requests/{request_id}',
            headers=headers
        )

        if response.status_code == 200:
            data = response.json()
            status = data['status']
            percentage = data.get('completionPercentage', 0)

            print(f'Status: {status}, Progress: {percentage}%')

            if status in ['completed', 'failed', 'cancelled']:
                print(f'Final status: {status}')
                return data

            time.sleep(60)  # Check every minute
        else:
            print(f'Error: {response.json()}')
            break

# Monitor status
result = monitor_erasure_status('ER-2025-12-18-001')
print(f'Accounts completed: {result["accountsCompleted"]}/{result["accountsTotal"]}')
```

### 7.3 cURL - Scan Digital Footprint

```bash
curl -X POST https://api.wia.live/digital-erasure/v1/inventory/scan \
  -H "Authorization: Bearer your_access_token" \
  -H "Content-Type: application/json" \
  -d '{
    "decedent": {
      "email": "user@example.com",
      "fullName": "John Doe"
    },
    "scanOptions": {
      "deepScan": true,
      "platformCategories": ["social_media", "email_messaging"]
    }
  }'
```

### 7.4 Java - Get Verification Proof

```java
import java.net.http.*;
import java.net.URI;
import com.google.gson.Gson;

public class WIAErasureClient {
    private static final String API_BASE = "https://api.wia.live/digital-erasure/v1";
    private static final String ACCESS_TOKEN = "your_access_token";

    public static VerificationProof getVerificationProof(String accountId) throws Exception {
        HttpClient client = HttpClient.newHttpClient();

        HttpRequest request = HttpRequest.newBuilder()
            .uri(URI.create(API_BASE + "/verifications/" + accountId))
            .header("Authorization", "Bearer " + ACCESS_TOKEN)
            .header("Content-Type", "application/json")
            .GET()
            .build();

        HttpResponse<String> response = client.send(request,
            HttpResponse.BodyHandlers.ofString());

        if (response.statusCode() == 200) {
            Gson gson = new Gson();
            return gson.fromJson(response.body(), VerificationProof.class);
        } else {
            throw new RuntimeException("Verification failed: " + response.body());
        }
    }

    public static void main(String[] args) {
        try {
            VerificationProof proof = getVerificationProof("ACC-001");
            System.out.println("Verification Status: " + proof.verificationStatus);
            System.out.println("Platform: " + proof.platform);
            System.out.println("Deletion Time: " + proof.deletionTimestamp);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
```

### 7.5 Go - Webhook Handler

```go
package main

import (
    "crypto/hmac"
    "crypto/sha256"
    "encoding/hex"
    "encoding/json"
    "io/ioutil"
    "log"
    "net/http"
)

type WebhookPayload struct {
    EventID   string                 `json:"eventId"`
    EventType string                 `json:"eventType"`
    Timestamp string                 `json:"timestamp"`
    Data      map[string]interface{} `json:"data"`
}

const webhookSecret = "webhook_secret_key"

func verifySignature(payload []byte, signature string) bool {
    mac := hmac.New(sha256.New, []byte(webhookSecret))
    mac.Write(payload)
    expectedSignature := "sha256=" + hex.EncodeToString(mac.Sum(nil))
    return hmac.Equal([]byte(signature), []byte(expectedSignature))
}

func handleWebhook(w http.ResponseWriter, r *http.Request) {
    body, err := ioutil.ReadAll(r.Body)
    if err != nil {
        http.Error(w, "Error reading body", http.StatusBadRequest)
        return
    }

    signature := r.Header.Get("X-WIA-Signature")
    if !verifySignature(body, signature) {
        http.Error(w, "Invalid signature", http.StatusUnauthorized)
        return
    }

    var payload WebhookPayload
    if err := json.Unmarshal(body, &payload); err != nil {
        http.Error(w, "Invalid JSON", http.StatusBadRequest)
        return
    }

    log.Printf("Received webhook: %s - %s", payload.EventType, payload.EventID)

    switch payload.EventType {
    case "account.deletion.completed":
        accountId := payload.Data["accountId"].(string)
        platform := payload.Data["platform"].(string)
        log.Printf("Account %s deleted from %s", accountId, platform)
    case "erasure.request.completed":
        requestId := payload.Data["requestId"].(string)
        log.Printf("Erasure request %s completed", requestId)
    }

    w.WriteHeader(http.StatusOK)
    w.Write([]byte("OK"))
}

func main() {
    http.HandleFunc("/webhooks/erasure", handleWebhook)
    log.Println("Webhook server listening on :8080")
    log.Fatal(http.ListenAndServe(":8080", nil))
}
```

---

## Security

## Security

### 8.1 Security Best Practices

| Practice | Description |
|----------|-------------|
| **TLS 1.3** | All API calls must use TLS 1.3+ |
| **Token Expiry** | Access tokens expire after 1 hour |
| **Refresh Tokens** | Valid for 30 days, rotate on use |
| **Webhook Signatures** | HMAC-SHA256 verification required |
| **Idempotency** | Use idempotency keys for POST requests |
| **IP Whitelisting** | Optional IP restrictions for sensitive operations |

### 8.2 Data Encryption

| Data Type | Encryption | Key Length |
|-----------|------------|------------|
| In Transit | TLS 1.3 | 256-bit |
| At Rest | AES-256-GCM | 256-bit |
| Death Certificates | End-to-end encryption | 4096-bit RSA |
| Verification Proofs | Digital signatures | 256-bit ECDSA |

### 8.3 Compliance

| Regulation | Compliance | Notes |
|------------|------------|-------|
| GDPR | Full compliance | Right to erasure support |
| CCPA | Full compliance | Data deletion rights |
| SOC 2 Type II | Certified | Annual audit |
| ISO 27001 | Certified | Information security |
| HIPAA | Compliant | For health data deletion |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-12-18 | Initial release |

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.
