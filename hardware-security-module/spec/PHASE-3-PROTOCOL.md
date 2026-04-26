Phase 2: Hybrid mode (transition)
         Classical + PQC
              ↓
Phase 3: PQC only (future)
         Kyber768 / Dilithium3
```

### 2.3.3 Performance Considerations

| Algorithm | Key Size | Signature Size | Sign Time | Verify Time |
|-----------|----------|----------------|-----------|-------------|
| RSA-2048 | 2048 bits | 256 bytes | 1ms | 0.1ms |
| ECDSA-P256 | 256 bits | 64 bytes | 0.5ms | 1ms |
| Dilithium2 | 2528 bytes | 2420 bytes | 0.3ms | 0.15ms |
| Dilithium3 | 4000 bytes | 3293 bytes | 0.5ms | 0.25ms |

---

## 2.4 Advanced Key Management

### 2.4.1 Key Rotation Automation

```python
class KeyRotationPolicy:
    def __init__(self, hsm_client):
        self.hsm = hsm_client

    def rotate_key(self, old_key_id, rotation_config):
        # Generate new key
        new_key = self.hsm.generate_key({
            'type': rotation_config['keyType'],
            'label': f"{rotation_config['baseLabel']}-{datetime.now().isoformat()}",
            'attributes': rotation_config['attributes']
        })

        # Transition period (dual operation)
        self.enable_dual_mode(old_key_id, new_key['keyId'])

        # Wait for transition period
        time.sleep(rotation_config['transitionPeriod'])

        # Retire old key
        self.hsm.update_key(old_key_id, {
            'CKA_SIGN': False,
            'CKA_ENCRYPT': False,
            'status': 'RETIRED'
        })

        return new_key
```

### 2.4.2 Key Lifecycle Automation

```yaml
keyLifecyclePolicy:
  - keyType: RSA-2048
    usage: signing
    rotation:
      frequency: 90 days
      autoRotate: true
      transitionPeriod: 7 days

  - keyType: AES-256
    usage: encryption
    rotation:
      frequency: 180 days
      autoRotate: true
      backupBeforeRotation: true

  - keyType: ECDSA-P256
    usage: authentication
    expiration:
      maxAge: 365 days
      warningPeriod: 30 days
      autoRenew: false
```

---

# Phase 3: Enterprise Integration

## 3.1 Multi-Tenancy Architecture

### 3.1.1 Tenant Isolation

```
┌─────────────────────────────────────────────┐
│              HSM Hardware Layer              │
├─────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │ Tenant A │  │ Tenant B │  │ Tenant C │  │
│  │  Keys    │  │  Keys    │  │  Keys    │  │
│  │ Isolated │  │ Isolated │  │ Isolated │  │
│  └──────────┘  └──────────┘  └──────────┘  │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │    Tenant Separation Enforcement       │ │
│  │  - Namespace isolation                 │ │
│  │  - Access control verification         │ │
│  │  - Audit logging per tenant            │ │
│  └────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### 3.1.2 Tenant Management API

```http
POST /api/v1/tenants
Authorization: Bearer {admin-token}

{
  "tenantId": "acme-corp",
  "name": "ACME Corporation",
  "quotas": {
    "maxKeys": 1000,
    "maxOperationsPerSecond": 500
  },
  "features": [
    "key-generation",
    "signing",
    "encryption",
    "blockchain-integration"
  ]
}

Response:
{
  "tenantId": "acme-corp",
  "apiKey": "tenant-api-key-xyz123",
  "encryptionRootKey": "key-handle-abc456",
  "createdAt": "2025-12-25T10:00:00Z"
}
```

---

## 3.2 Federated HSM Networks

### 3.2.1 Cross-Organization Key Sharing

```json
{
  "federation": {
    "name": "financial-consortium",
    "members": [
      {
        "orgId": "bank-alpha",
        "hsmEndpoint": "https://hsm.bank-alpha.com",
        "publicKey": "-----BEGIN PUBLIC KEY-----..."
      },
      {
        "orgId": "bank-beta",
        "hsmEndpoint": "https://hsm.bank-beta.com",
        "publicKey": "-----BEGIN PUBLIC KEY-----..."
      }
    ],
    "sharedKeys": [
      {
        "keyId": "interbank-settlement-key",
        "type": "multi-signature",
        "threshold": "2-of-3",
        "participants": ["bank-alpha", "bank-beta", "central-bank"]
      }
    ]
  }
}
```

### 3.2.2 Distributed Key Generation (DKG)

```
Step 1: Each HSM generates a share
  HSM-1: Share-1, Commitment-1
  HSM-2: Share-2, Commitment-2
  HSM-3: Share-3, Commitment-3

Step 2: Share distribution (encrypted)
  HSM-1 → HSM-2: Enc(Share-1-for-2)
  HSM-1 → HSM-3: Enc(Share-1-for-3)
  (similar for HSM-2 and HSM-3)

Step 3: Each HSM combines shares
  HSM-1: Combined-Share-1 = f(Share-1, Share-2-for-1, Share-3-for-1)
  HSM-2: Combined-Share-2 = f(Share-2, Share-1-for-2, Share-3-for-2)
  HSM-3: Combined-Share-3 = f(Share-3, Share-1-for-3, Share-2-for-3)

Result: Distributed key with threshold signing capability
```

---

## 3.3 Certificate Authority Integration

### 3.3.1 CA Key Storage

```json
{
  "caConfiguration": {
    "rootCA": {
      "keyHandle": "root-ca-key",
      "keyType": "RSA-4096",
      "subject": "CN=WIA Root CA,O=WIA,C=US",
      "validityYears": 20,
      "keyUsage": ["keyCertSign", "cRLSign"],
      "extractable": false,
      "backupShares": 5,
      "threshold": 3
    },
    "intermediateCA": {
      "keyHandle": "intermediate-ca-key",
      "keyType": "RSA-2048",
      "subject": "CN=WIA Intermediate CA,O=WIA,C=US",
      "validityYears": 10,
      "pathLength": 0
    }
  }
}
```

### 3.3.2 Certificate Issuance Workflow

```python
def issue_certificate(hsm, csr, ca_key_handle):
    # Parse CSR
    csr_data = parse_csr(csr)

    # Validate CSR
    if not validate_csr(csr_data):
        raise ValueError("Invalid CSR")

    # Build certificate template
    cert_template = {
        'version': 'v3',
        'serialNumber': generate_serial(),
        'subject': csr_data['subject'],
        'publicKey': csr_data['publicKey'],
        'validity': {
            'notBefore': datetime.now(),
            'notAfter': datetime.now() + timedelta(days=365)
        },
        'extensions': {
            'keyUsage': ['digitalSignature', 'keyEncipherment'],
            'extendedKeyUsage': ['serverAuth', 'clientAuth']
        }
    }

    # Sign with HSM-protected CA key
    cert_tbs = encode_tbs_certificate(cert_template)
    signature = hsm.sign({
        'keyHandle': ca_key_handle,
        'data': cert_tbs,
        'algorithm': 'SHA256-RSA-PKCS'
    })

    # Assemble final certificate
    certificate = build_certificate(cert_template, signature)
    return certificate
```

---

## 3.4 DevOps & Automation

### 3.4.1 Infrastructure as Code

#### Terraform Example
```hcl
resource "aws_cloudhsm_v2_cluster" "main" {
  hsm_type = "hsm1.medium"
  subnet_ids = var.subnet_ids

  tags = {
    Name = "wia-hsm-cluster"
    Environment = "production"
  }
}


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
