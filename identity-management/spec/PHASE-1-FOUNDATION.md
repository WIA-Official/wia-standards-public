# Phase 1: Identity Management Foundation Specification

## WIA-SEC-009 Identity Management Standard

**Version**: 1.0.0
**Date**: 2025-12-25
**Status**: Active
**Standard ID**: WIA-SEC-009-PHASE1-001
**Primary Color**: #8B5CF6 (Purple)

---

## 1. Overview

### 1.1 Purpose

WIA-SEC-009 is a comprehensive standard for digital identity management and access control systems. This standard enables interoperability between identity providers, credential issuers, verifiers, and relying parties across decentralized and centralized identity ecosystems.

**Core Objectives**:
- Standardize Decentralized Identifier (DID) formats and methods
- Define Verifiable Credential (VC) schemas and protocols
- Enable Self-Sovereign Identity (SSI) implementations
- Integrate with traditional IAM systems (LDAP, Active Directory)
- Support modern authentication protocols (OAuth 2.0, OpenID Connect, SAML)
- Facilitate SCIM-based user provisioning and lifecycle management
- Ensure privacy-preserving identity verification
- Enable cross-domain identity federation

### 1.2 Scope

This standard covers:

| Domain | Description |
|--------|-------------|
| **DID & SSI** | Decentralized identifiers, DID documents, key management |
| **Verifiable Credentials** | W3C VC data model, credential schemas, proof formats |
| **IAM Integration** | LDAP, Active Directory, Azure AD, Okta, Auth0 |
| **Authentication** | OAuth 2.0, OpenID Connect, SAML 2.0, WebAuthn |
| **Provisioning** | SCIM 2.0, user lifecycle management, attribute mapping |
| **Access Control** | RBAC, ABAC, policy enforcement, attribute-based access |
| **Identity Verification** | KYC, identity proofing, attestation, reputation systems |
| **Privacy & Compliance** | GDPR, CCPA, data minimization, zero-knowledge proofs |

### 1.3 Philosophy

**弘益人間 (홍익인간)** - Benefit All Humanity

Identity management should empower individuals with control over their digital identities while enabling secure, privacy-preserving access to services and resources.

---

## 2. Decentralized Identifier (DID) Schema

### 2.1 DID Format

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DecentralizedIdentifier",
  "type": "object",
  "required": ["did", "method", "identifier"],
  "properties": {
    "did": {
      "type": "string",
      "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$",
      "description": "Complete DID string",
      "examples": [
        "did:web:example.com:users:alice",
        "did:key:z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK",
        "did:ethr:0x3b0BC51Ab9De1e5B7B6E34E5b960285805C41736",
        "did:ion:EiClkZMDxPKqC9c-umQfTkR8vvZ9JPhl_xLDI9Nfk38w5w"
      ]
    },
    "method": {
      "type": "string",
      "enum": ["web", "key", "ethr", "ion", "peer", "sov", "btcr"],
      "description": "DID method identifier"
    },
    "identifier": {
      "type": "string",
      "description": "Method-specific identifier"
    },
    "controller": {
      "type": "string",
      "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$",
      "description": "DID of the entity that controls this identifier"
    },
    "created": {
      "type": "string",
      "format": "date-time",
      "description": "DID creation timestamp (ISO 8601)"
    },
    "updated": {
      "type": "string",
      "format": "date-time",
      "description": "Last update timestamp (ISO 8601)"
    },
    "deactivated": {
      "type": "boolean",
      "description": "Whether the DID is deactivated"
    }
  }
}
```

### 2.2 DID Document

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "DIDDocument",
  "type": "object",
  "required": ["@context", "id", "verificationMethod"],
  "properties": {
    "@context": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "default": [
        "https://www.w3.org/ns/did/v1",
        "https://w3id.org/security/suites/ed25519-2020/v1",
        "https://w3id.org/security/suites/x25519-2020/v1"
      ]
    },
    "id": {
      "type": "string",
      "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$",
      "description": "The DID subject"
    },
    "controller": {
      "oneOf": [
        {"type": "string"},
        {"type": "array", "items": {"type": "string"}}
      ],
      "description": "Entity authorized to make changes to the DID document"
    },
    "verificationMethod": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["id", "type", "controller"],
        "properties": {
          "id": {
            "type": "string",
            "description": "Verification method identifier"
          },
          "type": {
            "type": "string",
            "enum": [
              "Ed25519VerificationKey2020",
              "EcdsaSecp256k1VerificationKey2019",
              "RsaVerificationKey2018",
              "X25519KeyAgreementKey2020"
            ]
          },
          "controller": {
            "type": "string",
            "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$"
          },
          "publicKeyMultibase": {
            "type": "string",
            "description": "Multibase-encoded public key"
          },
          "publicKeyJwk": {
            "type": "object",
            "description": "JWK representation of public key"
          }
        }
      }
    },
    "authentication": {
      "type": "array",
      "items": {
        "oneOf": [
          {"type": "string"},
          {"type": "object"}
        ]
      },
      "description": "Authentication methods"
    },
    "assertionMethod": {
      "type": "array",
      "items": {
        "oneOf": [
          {"type": "string"},
          {"type": "object"}
        ]
      },
      "description": "Methods for making assertions"
    },
    "keyAgreement": {
      "type": "array",
      "items": {
        "oneOf": [
          {"type": "string"},
          {"type": "object"}
        ]
      },
      "description": "Key agreement methods for encryption"
    },
    "capabilityInvocation": {
      "type": "array",
      "items": {
        "oneOf": [
          {"type": "string"},
          {"type": "object"}
        ]
      },
      "description": "Methods for invoking cryptographic capabilities"
    },
    "capabilityDelegation": {
      "type": "array",
      "items": {
        "oneOf": [
          {"type": "string"},
          {"type": "object"}
        ]
      },
      "description": "Methods for delegating capabilities"
    },
    "service": {
      "type": "array",
      "items": {
        "type": "object",
        "required": ["id", "type", "serviceEndpoint"],
        "properties": {
          "id": {
            "type": "string",
            "description": "Service identifier"
          },
          "type": {
            "type": "string",
            "description": "Service type (e.g., CredentialRegistry, MessagingService)"
          },
          "serviceEndpoint": {
            "oneOf": [
              {"type": "string", "format": "uri"},
              {"type": "object"}
            ],
            "description": "Service endpoint URL or object"
          }
        }
      }
    }
  }
}
```

---

## 3. Verifiable Credential Schema

### 3.1 Basic Credential

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "VerifiableCredential",
  "type": "object",
  "required": ["@context", "type", "issuer", "issuanceDate", "credentialSubject"],
  "properties": {
    "@context": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "default": [
        "https://www.w3.org/2018/credentials/v1"
      ]
    },
    "id": {
      "type": "string",
      "format": "uri",
      "description": "Unique credential identifier"
    },
    "type": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "contains": {
        "const": "VerifiableCredential"
      },
      "examples": [
        ["VerifiableCredential", "IdentityCredential"],
        ["VerifiableCredential", "EmployeeCredential"]
      ]
    },
    "issuer": {
      "oneOf": [
        {
          "type": "string",
          "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$"
        },
        {
          "type": "object",
          "required": ["id"],
          "properties": {
            "id": {
              "type": "string",
              "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$"
            },
            "name": {"type": "string"}
          }
        }
      ],
      "description": "Credential issuer DID"
    },
    "issuanceDate": {
      "type": "string",
      "format": "date-time",
      "description": "When the credential was issued (ISO 8601)"
    },
    "expirationDate": {
      "type": "string",
      "format": "date-time",
      "description": "When the credential expires (ISO 8601)"
    },
    "credentialSubject": {
      "type": "object",
      "required": ["id"],
      "properties": {
        "id": {
          "type": "string",
          "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$",
          "description": "DID of the credential subject"
        }
      },
      "additionalProperties": true,
      "description": "Claims about the credential subject"
    },
    "credentialStatus": {
      "type": "object",
      "required": ["id", "type"],
      "properties": {
        "id": {
          "type": "string",
          "format": "uri",
          "description": "Status list URL"
        },
        "type": {
          "type": "string",
          "enum": ["RevocationList2020", "StatusList2021"],
          "description": "Status mechanism type"
        }
      },
      "description": "Revocation status information"
    },
    "proof": {
      "type": "object",
      "required": ["type", "created", "proofPurpose", "verificationMethod", "proofValue"],
      "properties": {
        "type": {
          "type": "string",
          "enum": [
            "Ed25519Signature2020",
            "EcdsaSecp256k1Signature2019",
            "RsaSignature2018",
            "BbsBlsSignature2020"
          ]
        },
        "created": {
          "type": "string",
          "format": "date-time"
        },
        "proofPurpose": {
          "type": "string",
          "enum": ["assertionMethod", "authentication"]
        },
        "verificationMethod": {
          "type": "string",
          "description": "Reference to verification method in DID document"
        },
        "proofValue": {
          "type": "string",
          "description": "Base64-encoded signature"
        }
      }
    }
  }
}
```

### 3.2 Identity Credential

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "IdentityCredential",
  "allOf": [
    {
      "$ref": "#/definitions/VerifiableCredential"
    }
  ],
  "properties": {
    "credentialSubject": {
      "type": "object",
      "required": ["id", "givenName", "familyName", "dateOfBirth"],
      "properties": {
        "id": {
          "type": "string",
          "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$"
        },
        "givenName": {
          "type": "string",
          "description": "First name"
        },
        "familyName": {
          "type": "string",
          "description": "Last name"
        },
        "dateOfBirth": {
          "type": "string",
          "format": "date",
          "description": "Date of birth (YYYY-MM-DD)"
        },
        "nationality": {
          "type": "string",
          "pattern": "^[A-Z]{2}$",
          "description": "ISO 3166-1 alpha-2 country code"
        },
        "email": {
          "type": "string",
          "format": "email"
        },
        "phoneNumber": {
          "type": "string",
          "pattern": "^\\+[1-9]\\d{1,14}$",
          "description": "E.164 format phone number"
        },
        "address": {
          "type": "object",
          "properties": {
            "streetAddress": {"type": "string"},
            "locality": {"type": "string"},
            "region": {"type": "string"},
            "postalCode": {"type": "string"},
            "country": {
              "type": "string",
              "pattern": "^[A-Z]{2}$"
            }
          }
        },
        "identityDocument": {
          "type": "object",
          "properties": {
            "type": {
              "type": "string",
              "enum": ["passport", "nationalId", "driversLicense"]
            },
            "number": {"type": "string"},
            "issuingCountry": {
              "type": "string",
              "pattern": "^[A-Z]{2}$"
            },
            "issuedDate": {
              "type": "string",
              "format": "date"
            },
            "expiryDate": {
              "type": "string",
              "format": "date"
            }
          }
        }
      }
    }
  }
}
```

---

## 4. Identity Attribute Schema

### 4.1 User Profile

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "UserProfile",
  "type": "object",
  "required": ["userId", "username", "attributes", "created", "updated"],
  "properties": {
    "userId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique user identifier"
    },
    "username": {
      "type": "string",
      "pattern": "^[a-zA-Z0-9._-]{3,50}$",
      "description": "Username (3-50 characters)"
    },
    "did": {
      "type": "string",
      "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$",
      "description": "Optional DID for the user"
    },
    "attributes": {
      "type": "object",
      "properties": {
        "email": {
          "type": "string",
          "format": "email"
        },
        "emailVerified": {
          "type": "boolean",
          "default": false
        },
        "phoneNumber": {
          "type": "string",
          "pattern": "^\\+[1-9]\\d{1,14}$"
        },
        "phoneNumberVerified": {
          "type": "boolean",
          "default": false
        },
        "givenName": {"type": "string"},
        "familyName": {"type": "string"},
        "middleName": {"type": "string"},
        "displayName": {"type": "string"},
        "preferredUsername": {"type": "string"},
        "profile": {
          "type": "string",
          "format": "uri"
        },
        "picture": {
          "type": "string",
          "format": "uri"
        },
        "website": {
          "type": "string",
          "format": "uri"
        },
        "gender": {
          "type": "string",
          "enum": ["male", "female", "other", "prefer-not-to-say"]
        },
        "birthdate": {
          "type": "string",
          "format": "date"
        },
        "zoneinfo": {
          "type": "string",
          "description": "IANA Time Zone identifier"
        },
        "locale": {
          "type": "string",
          "pattern": "^[a-z]{2}-[A-Z]{2}$",
          "description": "BCP47 language tag"
        },
        "address": {
          "$ref": "#/definitions/Address"
        }
      }
    },
    "roles": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "User roles for RBAC"
    },
    "groups": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Group memberships"
    },
    "permissions": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Direct permissions"
    },
    "customAttributes": {
      "type": "object",
      "additionalProperties": true,
      "description": "Organization-specific attributes"
    },
    "status": {
      "type": "string",
      "enum": ["active", "inactive", "suspended", "deleted"],
      "default": "active"
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "updated": {
      "type": "string",
      "format": "date-time"
    },
    "lastLogin": {
      "type": "string",
      "format": "date-time"
    }
  },
  "definitions": {
    "Address": {
      "type": "object",
      "properties": {
        "formatted": {"type": "string"},
        "streetAddress": {"type": "string"},
        "locality": {"type": "string"},
        "region": {"type": "string"},
        "postalCode": {"type": "string"},
        "country": {"type": "string"}
      }
    }
  }
}
```

---

## 5. Authentication Context

### 5.1 Authentication Request

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "AuthenticationRequest",
  "type": "object",
  "required": ["requestId", "client", "requestedClaims", "timestamp"],
  "properties": {
    "requestId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique request identifier"
    },
    "client": {
      "type": "object",
      "required": ["clientId", "redirectUri"],
      "properties": {
        "clientId": {
          "type": "string",
          "description": "OAuth 2.0 client identifier"
        },
        "redirectUri": {
          "type": "string",
          "format": "uri",
          "description": "Authorized redirect URI"
        },
        "responseType": {
          "type": "string",
          "enum": ["code", "token", "id_token"],
          "default": "code"
        },
        "scope": {
          "type": "string",
          "description": "Requested scopes (space-separated)"
        },
        "state": {
          "type": "string",
          "description": "Client state parameter"
        },
        "nonce": {
          "type": "string",
          "description": "Nonce for replay protection"
        }
      }
    },
    "requestedClaims": {
      "type": "array",
      "items": {
        "type": "string"
      },
      "description": "Requested user claims"
    },
    "authenticationMethods": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": [
          "password",
          "otp",
          "totp",
          "sms",
          "email",
          "biometric",
          "hardware-token",
          "webauthn",
          "did-auth"
        ]
      },
      "description": "Allowed authentication methods"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "expiresAt": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

---

## 6. Access Token Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "AccessToken",
  "type": "object",
  "required": ["sub", "iss", "aud", "exp", "iat"],
  "properties": {
    "sub": {
      "type": "string",
      "description": "Subject (user identifier or DID)"
    },
    "iss": {
      "type": "string",
      "format": "uri",
      "description": "Issuer (identity provider)"
    },
    "aud": {
      "oneOf": [
        {"type": "string"},
        {"type": "array", "items": {"type": "string"}}
      ],
      "description": "Audience (resource server)"
    },
    "exp": {
      "type": "integer",
      "description": "Expiration time (Unix timestamp)"
    },
    "iat": {
      "type": "integer",
      "description": "Issued at (Unix timestamp)"
    },
    "nbf": {
      "type": "integer",
      "description": "Not before (Unix timestamp)"
    },
    "scope": {
      "type": "string",
      "description": "Granted scopes (space-separated)"
    },
    "client_id": {
      "type": "string",
      "description": "OAuth client identifier"
    },
    "username": {
      "type": "string",
      "description": "Username"
    },
    "email": {
      "type": "string",
      "format": "email"
    },
    "roles": {
      "type": "array",
      "items": {
        "type": "string"
      }
    },
    "permissions": {
      "type": "array",
      "items": {
        "type": "string"
      }
    }
  }
}
```

---

## 7. Key Management

### 7.1 Key Pair

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "KeyPair",
  "type": "object",
  "required": ["keyId", "type", "publicKey", "created"],
  "properties": {
    "keyId": {
      "type": "string",
      "description": "Unique key identifier"
    },
    "type": {
      "type": "string",
      "enum": [
        "Ed25519",
        "Secp256k1",
        "RSA",
        "P-256",
        "P-384"
      ]
    },
    "publicKey": {
      "type": "string",
      "description": "Base64-encoded public key"
    },
    "publicKeyJwk": {
      "type": "object",
      "description": "JWK representation of public key"
    },
    "controller": {
      "type": "string",
      "pattern": "^did:[a-z0-9]+:[a-zA-Z0-9._%-]+$",
      "description": "DID that controls this key"
    },
    "purpose": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": [
          "authentication",
          "assertionMethod",
          "keyAgreement",
          "capabilityInvocation",
          "capabilityDelegation"
        ]
      }
    },
    "created": {
      "type": "string",
      "format": "date-time"
    },
    "revoked": {
      "type": "boolean",
      "default": false
    },
    "revokedAt": {
      "type": "string",
      "format": "date-time"
    }
  }
}
```

---

## 8. Examples

### 8.1 Complete DID Document Example

```json
{
  "@context": [
    "https://www.w3.org/ns/did/v1",
    "https://w3id.org/security/suites/ed25519-2020/v1"
  ],
  "id": "did:web:example.com:users:alice",
  "controller": "did:web:example.com:users:alice",
  "verificationMethod": [
    {
      "id": "did:web:example.com:users:alice#key-1",
      "type": "Ed25519VerificationKey2020",
      "controller": "did:web:example.com:users:alice",
      "publicKeyMultibase": "z6MkhaXgBZDvotDkL5257faiztiGiC2QtKLGpbnnEGta2doK"
    }
  ],
  "authentication": [
    "did:web:example.com:users:alice#key-1"
  ],
  "assertionMethod": [
    "did:web:example.com:users:alice#key-1"
  ],
  "service": [
    {
      "id": "did:web:example.com:users:alice#messaging",
      "type": "MessagingService",
      "serviceEndpoint": "https://example.com/messages/alice"
    }
  ]
}
```

### 8.2 Identity Credential Example

```json
{
  "@context": [
    "https://www.w3.org/2018/credentials/v1",
    "https://wia.global/contexts/identity/v1"
  ],
  "id": "https://example.com/credentials/3732",
  "type": ["VerifiableCredential", "IdentityCredential"],
  "issuer": {
    "id": "did:web:gov.example:identity",
    "name": "Example Government Identity Authority"
  },
  "issuanceDate": "2025-01-15T10:00:00Z",
  "expirationDate": "2030-01-15T10:00:00Z",
  "credentialSubject": {
    "id": "did:web:example.com:users:alice",
    "givenName": "Alice",
    "familyName": "Smith",
    "dateOfBirth": "1990-05-15",
    "nationality": "US",
    "email": "alice@example.com",
    "identityDocument": {
      "type": "passport",
      "number": "X12345678",
      "issuingCountry": "US",
      "issuedDate": "2020-01-15",
      "expiryDate": "2030-01-15"
    }
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T10:00:00Z",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:web:gov.example:identity#key-1",
    "proofValue": "z3FXQjecWufY46...signature...2EjFZrhTkRBKDV"
  }
}
```

---

## 9. Status Codes & Error Handling

| Code | Status | Description |
|------|--------|-------------|
| 200 | Success | Operation completed successfully |
| 201 | Created | Resource created successfully |
| 400 | Bad Request | Invalid request format or parameters |
| 401 | Unauthorized | Authentication required |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource not found |
| 409 | Conflict | Resource already exists |
| 422 | Unprocessable | Validation failed |
| 500 | Server Error | Internal server error |

---

## 10. Compliance & Standards

- **W3C DID Core**: https://www.w3.org/TR/did-core/
- **W3C Verifiable Credentials**: https://www.w3.org/TR/vc-data-model/
- **OAuth 2.0**: RFC 6749
- **OpenID Connect**: OpenID Connect Core 1.0
- **SAML 2.0**: OASIS Standard
- **SCIM 2.0**: RFC 7643, RFC 7644
- **WebAuthn**: W3C Web Authentication
- **GDPR**: EU General Data Protection Regulation
- **CCPA**: California Consumer Privacy Act

---

**Next Phase**: [PHASE-2-DATA.md](PHASE-2-DATA.md) - Data Formats & Schemas

© 2025 WIA (World Certification Industry Association)
**弘益人間 (홍익인간)** - Benefit All Humanity
