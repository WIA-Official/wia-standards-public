# Phase 2: Identity Data Formats & Verification

## WIA-SEC-009 Identity Management - Data Specification

**Version**: 1.0.0
**Date**: 2025-12-25
**Status**: Active
**Standard ID**: WIA-SEC-009-PHASE2-002
**Primary Color**: #8B5CF6 (Purple)

---

## 1. Overview

This phase defines comprehensive data formats for identity attributes, verification algorithms, credential schemas, and attribute mappings for identity management systems.

---

## 2. LDAP/Active Directory Schema

### 2.1 LDAP User Entry

```ldif
dn: uid=alice,ou=users,dc=example,dc=com
objectClass: top
objectClass: person
objectClass: organizationalPerson
objectClass: inetOrgPerson
objectClass: wiaIdentity
uid: alice
cn: Alice Smith
sn: Smith
givenName: Alice
mail: alice@example.com
telephoneNumber: +1-555-0100
userPassword: {SSHA}...hashed...
employeeNumber: 12345
employeeType: full-time
title: Senior Engineer
departmentNumber: Engineering
ou: Engineering
manager: uid=bob,ou=users,dc=example,dc=com
description: Software engineer specializing in identity systems
wiaIdentityDID: did:web:example.com:users:alice
wiaIdentityStatus: active
createTimestamp: 20250115100000Z
modifyTimestamp: 20250120150000Z
```

### 2.2 LDAP Group Entry

```ldif
dn: cn=developers,ou=groups,dc=example,dc=com
objectClass: top
objectClass: groupOfNames
objectClass: wiaGroup
cn: developers
description: Software Development Team
member: uid=alice,ou=users,dc=example,dc=com
member: uid=bob,ou=users,dc=example,dc=com
wiaGroupType: security
wiaGroupScope: global
wiaGroupPermissions: code.read,code.write,deploy.staging
createTimestamp: 20250101100000Z
modifyTimestamp: 20250120150000Z
```

### 2.3 WIA LDAP Object Classes

```ldif
# WIA Identity Object Class
dn: cn=wiaIdentity,cn=schema,cn=config
objectClass: olcSchemaConfig
cn: wiaIdentity
olcAttributeTypes: (
  1.3.6.1.4.1.99999.1.1.1
  NAME 'wiaIdentityDID'
  DESC 'Decentralized Identifier (DID)'
  EQUALITY caseExactMatch
  SYNTAX 1.3.6.1.4.1.1466.115.121.1.15
  SINGLE-VALUE
)
olcAttributeTypes: (
  1.3.6.1.4.1.99999.1.1.2
  NAME 'wiaIdentityStatus'
  DESC 'Identity status'
  EQUALITY caseIgnoreMatch
  SYNTAX 1.3.6.1.4.1.1466.115.121.1.15
  SINGLE-VALUE
)
olcAttributeTypes: (
  1.3.6.1.4.1.99999.1.1.3
  NAME 'wiaVerifiableCredentials'
  DESC 'List of verifiable credentials'
  EQUALITY caseExactMatch
  SYNTAX 1.3.6.1.4.1.1466.115.121.1.15
)
olcObjectClasses: (
  1.3.6.1.4.1.99999.1.2.1
  NAME 'wiaIdentity'
  DESC 'WIA Identity Management Extension'
  SUP top
  AUXILIARY
  MAY (wiaIdentityDID $ wiaIdentityStatus $ wiaVerifiableCredentials)
)
```

---

## 3. Active Directory Schema Extension

### 3.1 AD User Attributes

```powershell
# PowerShell script to extend AD schema
Import-Module ActiveDirectory

# Define WIA custom attributes
New-ADObject -Name "wia-DID" -Type attributeSchema -Path "CN=Schema,CN=Configuration,DC=example,DC=com" -OtherAttributes @{
    lDAPDisplayName = "wiaDID"
    attributeId = "1.3.6.1.4.1.99999.1.1.1"
    attributeSyntax = "2.5.5.12"
    oMSyntax = 64
    isSingleValued = $true
    searchFlags = 1
    description = "Decentralized Identifier (DID)"
}

New-ADObject -Name "wia-IdentityStatus" -Type attributeSchema -Path "CN=Schema,CN=Configuration,DC=example,DC=com" -OtherAttributes @{
    lDAPDisplayName = "wiaIdentityStatus"
    attributeId = "1.3.6.1.4.1.99999.1.1.2"
    attributeSyntax = "2.5.5.12"
    oMSyntax = 64
    isSingleValued = $true
    searchFlags = 1
    description = "Identity status (active/inactive/suspended)"
}

# Extend User class
Get-ADObject "CN=User,CN=Schema,CN=Configuration,DC=example,DC=com" |
    Set-ADObject -Add @{
        mayContain = @("wiaDID", "wiaIdentityStatus")
    }
```

### 3.2 AD User Example

```json
{
  "distinguishedName": "CN=Alice Smith,OU=Users,DC=example,DC=com",
  "sAMAccountName": "alice",
  "userPrincipalName": "alice@example.com",
  "displayName": "Alice Smith",
  "givenName": "Alice",
  "sn": "Smith",
  "mail": "alice@example.com",
  "telephoneNumber": "+1-555-0100",
  "department": "Engineering",
  "title": "Senior Engineer",
  "manager": "CN=Bob Johnson,OU=Users,DC=example,DC=com",
  "memberOf": [
    "CN=Developers,OU=Groups,DC=example,DC=com",
    "CN=Engineering,OU=Groups,DC=example,DC=com"
  ],
  "wiaDID": "did:web:example.com:users:alice",
  "wiaIdentityStatus": "active",
  "accountEnabled": true,
  "whenCreated": "2025-01-15T10:00:00Z",
  "whenChanged": "2025-01-20T15:00:00Z"
}
```

---

## 4. SCIM 2.0 User Schema

### 4.1 SCIM User Resource

```json
{
  "schemas": [
    "urn:ietf:params:scim:schemas:core:2.0:User",
    "urn:ietf:params:scim:schemas:extension:enterprise:2.0:User",
    "urn:wia:params:scim:schemas:extension:identity:1.0:User"
  ],
  "id": "2819c223-7f76-453a-919d-413861904646",
  "externalId": "alice@example.com",
  "meta": {
    "resourceType": "User",
    "created": "2025-01-15T10:00:00Z",
    "lastModified": "2025-01-20T15:00:00Z",
    "location": "https://example.com/scim/v2/Users/2819c223-7f76-453a-919d-413861904646",
    "version": "W/\"a330bc54f0671c9\""
  },
  "userName": "alice",
  "name": {
    "formatted": "Ms. Alice Smith",
    "familyName": "Smith",
    "givenName": "Alice",
    "middleName": "Marie",
    "honorificPrefix": "Ms.",
    "honorificSuffix": ""
  },
  "displayName": "Alice Smith",
  "nickName": "Ali",
  "profileUrl": "https://example.com/users/alice",
  "title": "Senior Engineer",
  "userType": "Employee",
  "preferredLanguage": "en-US",
  "locale": "en-US",
  "timezone": "America/Los_Angeles",
  "active": true,
  "emails": [
    {
      "value": "alice@example.com",
      "type": "work",
      "primary": true
    }
  ],
  "phoneNumbers": [
    {
      "value": "+1-555-0100",
      "type": "work",
      "primary": true
    }
  ],
  "addresses": [
    {
      "type": "work",
      "formatted": "100 Universal City Plaza\nHollywood, CA 91608 USA",
      "streetAddress": "100 Universal City Plaza",
      "locality": "Hollywood",
      "region": "CA",
      "postalCode": "91608",
      "country": "USA",
      "primary": true
    }
  ],
  "photos": [
    {
      "value": "https://example.com/photos/alice.jpg",
      "type": "photo"
    }
  ],
  "groups": [
    {
      "value": "e9e30dba-f08f-4109-8486-d5c6a331660a",
      "display": "Developers",
      "$ref": "https://example.com/scim/v2/Groups/e9e30dba-f08f-4109-8486-d5c6a331660a"
    }
  ],
  "urn:ietf:params:scim:schemas:extension:enterprise:2.0:User": {
    "employeeNumber": "12345",
    "costCenter": "4130",
    "organization": "Example Corp",
    "division": "Engineering",
    "department": "Platform Engineering",
    "manager": {
      "value": "26118915-6090-4610-87e4-49d8ca9f808d",
      "displayName": "Bob Johnson",
      "$ref": "https://example.com/scim/v2/Users/26118915-6090-4610-87e4-49d8ca9f808d"
    }
  },
  "urn:wia:params:scim:schemas:extension:identity:1.0:User": {
    "did": "did:web:example.com:users:alice",
    "identityStatus": "active",
    "verifiableCredentials": [
      "https://example.com/credentials/3732"
    ],
    "biometricEnrolled": true,
    "mfaEnabled": true,
    "mfaMethods": ["totp", "webauthn"],
    "roles": ["engineer", "developer"],
    "permissions": ["code.read", "code.write", "deploy.staging"]
  }
}
```

### 4.2 SCIM Group Resource

```json
{
  "schemas": ["urn:ietf:params:scim:schemas:core:2.0:Group"],
  "id": "e9e30dba-f08f-4109-8486-d5c6a331660a",
  "displayName": "Developers",
  "externalId": "developers",
  "meta": {
    "resourceType": "Group",
    "created": "2025-01-01T10:00:00Z",
    "lastModified": "2025-01-20T15:00:00Z",
    "location": "https://example.com/scim/v2/Groups/e9e30dba-f08f-4109-8486-d5c6a331660a"
  },
  "members": [
    {
      "value": "2819c223-7f76-453a-919d-413861904646",
      "display": "Alice Smith",
      "$ref": "https://example.com/scim/v2/Users/2819c223-7f76-453a-919d-413861904646",
      "type": "User"
    }
  ]
}
```

---

## 5. Identity Verification Algorithms

### 5.1 Attribute Matching Algorithm

```javascript
/**
 * Calculate identity attribute matching score
 * @param {Object} claimed - Claimed identity attributes
 * @param {Object} verified - Verified identity attributes
 * @returns {Object} Matching score and details
 */
function calculateAttributeMatch(claimed, verified) {
  const weights = {
    givenName: 0.15,
    familyName: 0.15,
    dateOfBirth: 0.20,
    nationality: 0.10,
    documentNumber: 0.20,
    biometricMatch: 0.20
  };

  let totalScore = 0;
  let matches = {};

  for (const [attribute, weight] of Object.entries(weights)) {
    if (!claimed[attribute] || !verified[attribute]) {
      matches[attribute] = { score: 0, reason: 'missing' };
      continue;
    }

    let score = 0;
    if (attribute === 'biometricMatch') {
      // Biometric matching score (0-1)
      score = verified[attribute];
    } else if (attribute === 'givenName' || attribute === 'familyName') {
      // Fuzzy string matching for names
      score = fuzzyMatch(
        claimed[attribute].toLowerCase(),
        verified[attribute].toLowerCase()
      );
    } else {
      // Exact match for other attributes
      score = claimed[attribute] === verified[attribute] ? 1.0 : 0.0;
    }

    matches[attribute] = { score, weight };
    totalScore += score * weight;
  }

  return {
    overallScore: totalScore,
    threshold: 0.75,
    verified: totalScore >= 0.75,
    matches,
    timestamp: new Date().toISOString()
  };
}

/**
 * Fuzzy string matching using Levenshtein distance
 */
function fuzzyMatch(str1, str2) {
  const distance = levenshteinDistance(str1, str2);
  const maxLength = Math.max(str1.length, str2.length);
  return 1.0 - (distance / maxLength);
}

function levenshteinDistance(str1, str2) {
  const matrix = [];

  for (let i = 0; i <= str2.length; i++) {
    matrix[i] = [i];
  }

  for (let j = 0; j <= str1.length; j++) {
    matrix[0][j] = j;
  }

  for (let i = 1; i <= str2.length; i++) {
    for (let j = 1; j <= str1.length; j++) {
      if (str2.charAt(i - 1) === str1.charAt(j - 1)) {
        matrix[i][j] = matrix[i - 1][j - 1];
      } else {
        matrix[i][j] = Math.min(
          matrix[i - 1][j - 1] + 1,
          matrix[i][j - 1] + 1,
          matrix[i - 1][j] + 1
        );
      }
    }
  }

  return matrix[str2.length][str1.length];
}
```

### 5.2 Identity Confidence Score

```javascript
/**
 * Calculate identity confidence score based on multiple factors
 */
function calculateConfidenceScore(identity) {
  const factors = {
    // Document verification
    documentVerified: identity.documentVerified ? 0.30 : 0.00,

    // Biometric verification
    biometricVerified: identity.biometricVerified ? 0.25 : 0.00,

    // Email verification
    emailVerified: identity.emailVerified ? 0.10 : 0.00,

    // Phone verification
    phoneVerified: identity.phoneVerified ? 0.10 : 0.00,

    // Address verification
    addressVerified: identity.addressVerified ? 0.10 : 0.00,

    // Third-party attestation
    attestationCount: Math.min(identity.attestations.length * 0.05, 0.15)
  };

  const totalScore = Object.values(factors).reduce((a, b) => a + b, 0);

  let level;
  if (totalScore >= 0.80) level = 'high';
  else if (totalScore >= 0.50) level = 'medium';
  else level = 'low';

  return {
    score: totalScore,
    level,
    factors,
    timestamp: new Date().toISOString()
  };
}
```

### 5.3 Risk Assessment Algorithm

```javascript
/**
 * Assess authentication risk based on context
 */
function assessAuthenticationRisk(context) {
  let riskScore = 0;
  const factors = {};

  // Device fingerprint
  if (!context.deviceTrusted) {
    riskScore += 0.20;
    factors.deviceTrusted = { score: 0.20, reason: 'unknown device' };
  }

  // Location anomaly
  if (context.locationAnomaly) {
    riskScore += 0.25;
    factors.locationAnomaly = { score: 0.25, reason: 'unusual location' };
  }

  // Time anomaly
  if (context.timeAnomaly) {
    riskScore += 0.15;
    factors.timeAnomaly = { score: 0.15, reason: 'unusual time' };
  }

  // Failed attempts
  if (context.failedAttempts > 3) {
    const attemptScore = Math.min(context.failedAttempts * 0.10, 0.30);
    riskScore += attemptScore;
    factors.failedAttempts = { score: attemptScore, count: context.failedAttempts };
  }

  // IP reputation
  if (context.ipReputation < 50) {
    riskScore += 0.15;
    factors.ipReputation = { score: 0.15, value: context.ipReputation };
  }

  let level;
  if (riskScore >= 0.70) level = 'high';
  else if (riskScore >= 0.40) level = 'medium';
  else level = 'low';

  return {
    riskScore,
    level,
    factors,
    requireMFA: riskScore >= 0.40,
    requireAdditionalVerification: riskScore >= 0.70,
    timestamp: new Date().toISOString()
  };
}
```

---

## 6. Attribute Mapping Schema

### 6.1 LDAP to SCIM Mapping

```json
{
  "attributeMappings": [
    { "ldap": "uid", "scim": "userName" },
    { "ldap": "givenName", "scim": "name.givenName" },
    { "ldap": "sn", "scim": "name.familyName" },
    { "ldap": "cn", "scim": "name.formatted" },
    { "ldap": "mail", "scim": "emails[type eq \"work\"].value" },
    { "ldap": "telephoneNumber", "scim": "phoneNumbers[type eq \"work\"].value" },
    { "ldap": "title", "scim": "title" },
    { "ldap": "departmentNumber", "scim": "urn:ietf:params:scim:schemas:extension:enterprise:2.0:User:department" },
    { "ldap": "employeeNumber", "scim": "urn:ietf:params:scim:schemas:extension:enterprise:2.0:User:employeeNumber" },
    { "ldap": "manager", "scim": "urn:ietf:params:scim:schemas:extension:enterprise:2.0:User:manager.value" },
    { "ldap": "wiaIdentityDID", "scim": "urn:wia:params:scim:schemas:extension:identity:1.0:User:did" }
  ],
  "transformations": {
    "mail": {
      "function": "toLowerCase"
    },
    "telephoneNumber": {
      "function": "formatE164"
    },
    "manager": {
      "function": "extractDNToUserId"
    }
  }
}
```

### 6.2 AD to OAuth Claims Mapping

```json
{
  "claimMappings": [
    { "ad": "sAMAccountName", "oauth": "preferred_username" },
    { "ad": "userPrincipalName", "oauth": "email" },
    { "ad": "displayName", "oauth": "name" },
    { "ad": "givenName", "oauth": "given_name" },
    { "ad": "sn", "oauth": "family_name" },
    { "ad": "mail", "oauth": "email" },
    { "ad": "telephoneNumber", "oauth": "phone_number" },
    { "ad": "department", "oauth": "department" },
    { "ad": "title", "oauth": "title" },
    { "ad": "memberOf", "oauth": "groups" },
    { "ad": "wiaDID", "oauth": "did" }
  ],
  "conditions": {
    "groups": {
      "transform": "extractCNFromDN",
      "filter": "CN=*,OU=Groups,DC=example,DC=com"
    }
  }
}
```

---

## 7. Digital Identity Certificate

### 7.1 Certificate Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DigitalIdentityCertificate",
  "type": "object",
  "required": ["certificateId", "subject", "issuer", "validFrom", "validUntil"],
  "properties": {
    "certificateId": {
      "type": "string",
      "format": "uuid"
    },
    "subject": {
      "type": "object",
      "required": ["did", "name"],
      "properties": {
        "did": {
          "type": "string",
          "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$"
        },
        "name": {
          "type": "object",
          "properties": {
            "givenName": { "type": "string" },
            "familyName": { "type": "string" },
            "formatted": { "type": "string" }
          }
        },
        "dateOfBirth": {
          "type": "string",
          "format": "date"
        },
        "nationality": {
          "type": "string",
          "pattern": "^[A-Z]{2}$"
        }
      }
    },
    "issuer": {
      "type": "object",
      "required": ["did", "name"],
      "properties": {
        "did": {
          "type": "string",
          "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$"
        },
        "name": { "type": "string" },
        "country": { "type": "string" },
        "registrationNumber": { "type": "string" }
      }
    },
    "certificateType": {
      "type": "string",
      "enum": ["identity", "employee", "student", "citizen", "resident"]
    },
    "assuranceLevel": {
      "type": "string",
      "enum": ["low", "substantial", "high"],
      "description": "eIDAS assurance level"
    },
    "validFrom": {
      "type": "string",
      "format": "date-time"
    },
    "validUntil": {
      "type": "string",
      "format": "date-time"
    },
    "attributes": {
      "type": "object",
      "additionalProperties": true
    },
    "verificationMethods": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": ["document", "biometric", "video", "in-person"]
      }
    },
    "signature": {
      "type": "object",
      "required": ["algorithm", "value"],
      "properties": {
        "algorithm": {
          "type": "string",
          "enum": ["RS256", "ES256", "EdDSA"]
        },
        "value": {
          "type": "string",
          "description": "Base64-encoded signature"
        },
        "publicKey": {
          "type": "string",
          "description": "Issuer's public key"
        }
      }
    },
    "qrCode": {
      "type": "string",
      "description": "Base64-encoded QR code image"
    }
  }
}
```

---

## 8. Revocation & Status

### 8.1 Credential Status List

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "StatusList2021",
  "type": "object",
  "required": ["@context", "id", "type", "statusPurpose", "encodedList"],
  "properties": {
    "@context": {
      "type": "array",
      "items": { "type": "string" },
      "default": ["https://w3id.org/vc/status-list/2021/v1"]
    },
    "id": {
      "type": "string",
      "format": "uri"
    },
    "type": {
      "const": "StatusList2021Credential"
    },
    "statusPurpose": {
      "type": "string",
      "enum": ["revocation", "suspension"]
    },
    "encodedList": {
      "type": "string",
      "description": "Base64-encoded compressed bitstring"
    }
  }
}
```

---

**Next Phase**: [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md) - Protocols & Workflows

© 2025 WIA (World Certification Industry Association)
**弘益人間 (홍익인간)** - Benefit All Humanity
