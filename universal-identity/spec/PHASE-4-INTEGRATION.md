# WIA-CORE-001: Phase 4 - Platform Integration

> **Phase:** 4 of 4  
> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27

---

## Overview

Phase 4 provides tools, SDKs, and connectors for real-world deployment of universal identity systems. This phase enables developers to integrate WIA-CORE-001 into their applications quickly and correctly.

## Language SDKs

### JavaScript/TypeScript

**Installation:**
```bash
npm install @wia/core-001
```

**Usage:**
```typescript
import { UniversalIdentity, VerifiableCredential } from '@wia/core-001';

// Create identity
const identity = await UniversalIdentity.create({
  type: 'email',
  attributes: {
    email: 'user@example.com',
    emailVerified: true
  }
});

// Issue credential
const credential = await VerifiableCredential.issue({
  issuer: 'did:web:example.com',
  subject: identity.did,
  type: 'EmailCredential',
  claims: { email: 'user@example.com', emailVerified: true }
});

// Verify credential
const result = await VerifiableCredential.verify(credential);
if (result.valid) {
  console.log('Credential is valid!');
}
```

### Python

**Installation:**
```bash
pip install wia-core-001
```

**Usage:**
```python
from wia_core_001 import UniversalIdentity, VerifiableCredential

# Create identity
identity = UniversalIdentity.create(
    type='email',
    attributes={'email': 'user@example.com', 'emailVerified': True}
)

# Issue credential
credential = VerifiableCredential.issue(
    issuer='did:web:example.com',
    subject=identity.did,
    type='EmailCredential',
    claims={'email': 'user@example.com', 'emailVerified': True}
)

# Verify credential
result = VerifiableCredential.verify(credential)
print(f'Valid: {result.valid}')
```

### Java

**Maven:**
```xml
<dependency>
    <groupId>org.wia</groupId>
    <artifactId>wia-core-001</artifactId>
    <version>1.0.0</version>
</dependency>
```

**Usage:**
```java
import org.wia.core001.*;

UniversalIdentity identity = UniversalIdentity.builder()
    .type("email")
    .attribute("email", "user@example.com")
    .attribute("emailVerified", true)
    .build();

VerifiableCredential credential = VerifiableCredential.builder()
    .issuer("did:web:example.com")
    .subject(identity.getDid())
    .type("EmailCredential")
    .claim("email", "user@example.com")
    .claim("emailVerified", true)
    .build();

VerificationResult result = credential.verify();
```

## Platform Connectors

### WordPress Plugin

**Installation:**
1. Upload plugin to `/wp-content/plugins/wia-core-001/`
2. Activate through WordPress admin
3. Configure DID provider settings

**Configuration:**
```php
define('WIA_DID_METHOD', 'web');
define('WIA_DID_HOST', 'example.com');
define('WIA_API_ENDPOINT', 'https://api.wia.org/v1');
```

**Usage in Theme:**
```php
<?php
if (wia_is_authenticated()) {
    $identity = wia_get_current_identity();
    echo "Welcome, " . $identity->attributes->name;
    echo "Trust Level: " . $identity->trustLevel;
}
?>
```

### Shopify App

**Installation:**
- Install from Shopify App Store
- Configure OAuth settings
- Enable Universal Identity login

**Checkout Integration:**
```javascript
// Automatically populate checkout with verified credentials
WIA.onCredentialPresentation(async (presentation) => {
  const claims = presentation.getClaims();
  
  Shopify.Checkout.setContactInfo({
    email: claims.email,
    phone: claims.phone
  });
  
  if (claims.shippingAddress) {
    Shopify.Checkout.setShippingAddress(claims.shippingAddress);
  }
});
```

### Salesforce Connector

**Installation:**
- Install managed package from AppExchange
- Configure SAML SSO with WIA identity provider
- Map DID to Salesforce User

**Apex Integration:**
```java
WIAIdentity identity = WIAService.getCurrentIdentity();
if (identity.trustLevel >= 3) {
    // Allow access to sensitive customer data
    Account acc = [SELECT Id, Name FROM Account WHERE OwnerId = :identity.salesforceUserId];
}
```

## Framework Integrations

### React

```tsx
import { useWIAAuth } from '@wia/react';

function App() {
  const { user, login, logout, trustLevel } = useWIAAuth();

  return (
    <div>
      {user ? (
        <>
          <p>Welcome, {user.name}!</p>
          <p>Trust Level: {trustLevel}</p>
          <button onClick={logout}>Logout</button>
        </>
      ) : (
        <button onClick={login}>Login with WIA</button>
      )}
    </div>
  );
}
```

### Express.js

```javascript
const express = require('express');
const { WIAAuthMiddleware } = require('@wia/express');

const app = express();

app.use(WIAAuthMiddleware({
  issuer: 'https://accounts.example.com',
  clientId: 'your-client-id',
  clientSecret: 'your-client-secret',
  requiredTrustLevel: 1
}));

app.get('/profile', (req, res) => {
  res.json({
    did: req.wia.identity.did,
    trustLevel: req.wia.trustLevel,
    attributes: req.wia.identity.attributes
  });
});
```

### Django

```python
# settings.py
AUTHENTICATION_BACKENDS = [
    'wia_core_001.django.WIAAuthBackend',
]

WIA_CONFIG = {
    'ISSUER': 'https://accounts.example.com',
    'CLIENT_ID': 'your-client-id',
    'CLIENT_SECRET': 'your-client-secret',
}

# views.py
from wia_core_001.django import wia_required

@wia_required(trust_level=2)
def sensitive_view(request):
    identity = request.wia_identity
    return render(request, 'profile.html', {'identity': identity})
```

## Migration Tools

### Import from Auth0

```bash
wia-migrate --source auth0 --auth0-domain example.auth0.com --auth0-token TOKEN
```

### Import from Okta

```bash
wia-migrate --source okta --okta-domain example.okta.com --okta-token TOKEN
```

### Import from Legacy Database

```bash
wia-migrate --source csv --file users.csv --mapping mapping.json
```

**Mapping Configuration:**
```json
{
  "emailColumn": "email",
  "nameColumn": "full_name",
  "phoneColumn": "phone_number",
  "verifiedEmails": true,
  "defaultTrustLevel": 1
}
```

## Testing Tools

### Test Identity Generator

```bash
wia-test-gen --count 1000 --trust-level 2 --output test-identities.json
```

### Conformance Test Suite

```bash
wia-test-suite --endpoint https://your-api.example.com --spec-version 1.0
```

**Output:**
```
✓ DID Resolution (15/15 tests passed)
✓ Credential Issuance (23/23 tests passed)
✓ Credential Verification (18/18 tests passed)
✓ OAuth 2.1 Flow (12/12 tests passed)
✓ OIDC Integration (20/20 tests passed)

Overall: 88/88 tests passed (100%)
Status: CONFORMANT
```

## Certification Process

1. **Self-Assessment**
   - Complete checklist of requirements
   - Run automated conformance tests
   - Document implementation decisions

2. **Automated Testing**
   - Submit endpoint for testing
   - Automated test suite validates conformance
   - Performance benchmarks verified

3. **Security Audit**
   - Third-party security assessment
   - Penetration testing
   - Cryptographic implementation review

4. **Privacy Review**
   - GDPR compliance verification
   - Data minimization audit
   - Consent management validation

5. **Certification**
   - Receive official WIA-CORE-001 certification
   - License to use WIA branding
   - Listed in official directory

## Deployment Checklist

- [ ] Choose DID method (did:web recommended for production)
- [ ] Set up DID hosting infrastructure
- [ ] Implement credential issuance flows
- [ ] Configure authentication protocols (OAuth/OIDC)
- [ ] Integrate with existing user database
- [ ] Set up rate limiting and security controls
- [ ] Configure audit logging
- [ ] Test with conformance suite
- [ ] Conduct security audit
- [ ] Train support staff
- [ ] Prepare user documentation
- [ ] Plan gradual rollout strategy
- [ ] Monitor metrics post-launch

---

**Phase 4 Complete.** All phases implemented! 🎉

**弘益人間 (Benefit All Humanity)**

© 2025 SmileStory Inc. / WIA

## P.4 Integration Cross-References

This Phase describes how the data formats (Phase 1), API surface (Phase 2),
and protocol layer (Phase 3) compose with adjacent infrastructure to form a
production deployment.

### P.4.1 Deployment Topologies

| Topology | When to Use | Trade-off |
|----------|------------|-----------|
| Single-region active-passive | Predictable latency, single-region users | Cold standby cost |
| Multi-region active-active | Global users, regional sovereignty | Conflict resolution complexity |
| Edge fan-out | Low latency at the edge, central system of record | Cache coherence |
| Air-gapped enclave | Regulatory / national security domains | Manual reconciliation |

### P.4.2 Dependency Inventory

Every implementation MUST publish a Software Bill of Materials (SBOM) in
SPDX 2.3 or CycloneDX 1.5 format covering: (a) direct runtime dependencies,
(b) transitive dependencies pinned to specific versions, (c) base container
images, (d) cryptographic libraries.

### P.4.3 Operational Readiness Checklist

- [ ] Health check endpoint returns 200 within 1 s p99
- [ ] Metrics exposed in Prometheus or OTLP format
- [ ] Logs are structured JSON with correlation IDs
- [ ] Traces use W3C Trace Context headers end-to-end
- [ ] Backups verified by quarterly restore drill
- [ ] Runbook published and indexed
- [ ] Disaster recovery RTO / RPO documented
- [ ] On-call rotation defined and acknowledged

### P.4.4 Migration Pathways

Adopters migrating from legacy systems should follow the staged pattern:
(1) shadow read, (2) shadow write, (3) primary write with legacy fallback,
(4) primary read, (5) legacy decommission. Each stage runs for at least one
business cycle before the next.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of universal-identity so that conformance claims at any
Phase remain unambiguous.*

