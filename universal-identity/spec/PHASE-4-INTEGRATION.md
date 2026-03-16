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
