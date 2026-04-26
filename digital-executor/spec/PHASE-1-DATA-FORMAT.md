# WIA-DIGITAL_EXECUTOR PHASE 1 — Data Format Specification

**Standard:** WIA-DIGITAL_EXECUTOR
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA Digital Executor Standard - Phase 1: Data Format Specification

**Version:** 1.0.0
**Status:** Draft
**Date:** 2025-12-18
**Primary Color:** #64748B (Slate - Digital Death Series)
**Category:** Digital Estate Management

---

## Table of Contents

1. [Introduction](#introduction)
2. [Core Data Structures](#core-data-structures)
3. [Executor Profile Schema](#executor-profile-schema)
4. [Task Management Schema](#task-management-schema)
5. [Access Control Schema](#access-control-schema)
6. [Legal Authority Schema](#legal-authority-schema)
7. [Progress Tracking Schema](#progress-tracking-schema)
8. [Communication Schema](#communication-schema)
9. [Audit Trail Schema](#audit-trail-schema)
10. [Data Validation Rules](#data-validation-rules)
11. [Code Examples](#code-examples)

---

## 1. Introduction

The WIA Digital Executor Standard defines the data formats, structures, and schemas required for implementing digital estate executor systems. This specification ensures interoperability, security, and compliance across different platforms managing digital assets after death.

### 1.1 Purpose

The digital executor role is critical in modern estate management, handling:
- Access to deceased's digital accounts and assets
- Distribution of digital property to beneficiaries
- Compliance with legal requirements and platform policies
- Communication with stakeholders
- Documentation and audit trails

### 1.2 Scope

This Phase 1 specification covers:
- Data structure definitions for all executor-related entities
- JSON schema specifications
- Validation rules and constraints
- Data relationship models
- Security and privacy requirements

### 1.3 Key Principles

| Principle | Description | Implementation |
|-----------|-------------|----------------|
| **Security First** | All executor data must be encrypted and access-controlled | AES-256, role-based access |
| **Legal Compliance** | Support for jurisdiction-specific requirements | Configurable legal frameworks |
| **Transparency** | Complete audit trails for all actions | Immutable logging |
| **Succession Planning** | Support for backup executors and delegation | Hierarchical executor model |
| **Interoperability** | Work across platforms and services | Standard JSON schemas |

---

## 2. Core Data Structures

### 2.1 Base Entity Model

All digital executor entities inherit from this base structure:

```json
{
  "entityId": "uuid-v4",
  "entityType": "executor|task|access|legal|audit",
  "version": "1.0.0",
  "createdAt": "ISO-8601 timestamp",
  "updatedAt": "ISO-8601 timestamp",
  "createdBy": "executor-id",
  "metadata": {
    "jurisdiction": "ISO-3166-1 country code",
    "language": "ISO-639-1 language code",
    "timezone": "IANA timezone",
    "tags": ["string"]
  },
  "status": "active|inactive|suspended|completed",
  "checksum": "SHA-256 hash"
}
```

### 2.2 Relationship Model

```json
{
  "relationships": {
    "executor": {
      "hasMany": ["tasks", "accessGrants", "auditLogs"],
      "belongsTo": ["estate", "decedent"],
      "references": ["beneficiaries", "platforms"]
    },
    "task": {
      "belongsTo": ["executor", "estate"],
      "hasMany": ["subtasks", "documents", "auditLogs"]
    },
    "accessGrant": {
      "belongsTo": ["executor", "platform"],
      "hasOne": ["legalAuthority"]
    }
  }
}
```

### 2.3 Enumeration Types

| Enumeration | Values | Usage |
|-------------|--------|-------|
| **ExecutorRole** | primary, backup, co-executor, specialist, legal-advisor | Defines executor hierarchy |
| **TaskPriority** | critical, high, medium, low | Task prioritization |
| **AccessLevel** | full, read-only, limited, emergency | Permission levels |
| **LegalStatus** | pending, verified, approved, rejected, expired | Legal document state |
| **ComplianceLevel** | required, recommended, optional | Compliance requirements |

---

## 3. Executor Profile Schema

### 3.1 Complete Executor Profile

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Digital Executor Profile",
  "type": "object",
  "required": [
    "executorId",
    "personalInfo",
    "role",
    "permissions",
    "contactInfo",
    "legalAuthority"
  ],
  "properties": {
    "executorId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique identifier for the executor"
    },
    "personalInfo": {
      "type": "object",
      "required": ["firstName", "lastName", "dateOfBirth"],
      "properties": {
        "firstName": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100
        },
        "lastName": {
          "type": "string",
          "minLength": 1,
          "maxLength": 100
        },
        "middleName": {
          "type": "string",
          "maxLength": 100
        },
        "dateOfBirth": {
          "type": "string",
          "format": "date"
        },
        "citizenship": {
          "type": "array",
          "items": {
            "type": "string",
            "pattern": "^[A-Z]{2}$"
          }
        },
        "governmentIds": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "type": {
                "type": "string",
                "enum": ["passport", "nationalId", "driversLicense", "taxId"]
              },
              "number": {
                "type": "string"
              },
              "issuingCountry": {
                "type": "string",
                "pattern": "^[A-Z]{2}$"
              },
              "expiryDate": {
                "type": "string",
                "format": "date"
              },
              "verified": {
                "type": "boolean"
              }
            }
          }
        }
      }
    },
    "role": {
      "type": "object",
      "required": ["type", "appointmentDate", "status"],
      "properties": {
        "type": {
          "type": "string",
          "enum": ["primary", "backup", "co-executor", "specialist", "legal-advisor"]
        },
        "appointmentDate": {
          "type": "string",
          "format": "date-time"
        },
        "effectiveDate": {
          "type": "string",
          "format": "date-time"
        },
        "expiryDate": {
          "type": "string",
          "format": "date-time"
        },
        "status": {
          "type": "string",
          "enum": ["designated", "active", "inactive", "suspended", "revoked"]
        },
        "priority": {
          "type": "integer",
          "minimum": 1,
          "maximum": 100,
          "description": "Lower numbers = higher priority"
        },
        "specialization": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["social-media", "financial", "intellectual-property", "gaming", "cryptocurrency", "general"]
          }
        }
      }
    },
    "permissions": {
      "type": "object",
      "properties": {
        "accessLevel": {
          "type": "string",
          "enum": ["full", "read-only", "limited", "emergency"]
        },
        "allowedActions": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": [
              "view-assets",
              "download-data",
              "delete-accounts",
              "transfer-assets",
              "communicate-beneficiaries",
              "update-settings",
              "appoint-successor",
              "access-credentials",
              "modify-permissions",
              "generate-reports"
            ]
          }
        },
        "restrictions": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "type": {
                "type": "string",
                "enum": ["time-based", "approval-required", "platform-specific", "asset-type"]
              },
              "condition": {
                "type": "string"
              },
              "expiresAt": {
                "type": "string",
                "format": "date-time"
              }
            }
          }
        },
        "delegationAllowed": {
          "type": "boolean"
        },
        "requiresApproval": {
          "type": "array",
          "items": {
            "type": "string"
          }
        }
      }
    },
    "contactInfo": {
      "type": "object",
      "required": ["email", "phone"],
      "properties": {
        "email": {
          "type": "array",
          "minItems": 1,
          "items": {
            "type": "object",
            "properties": {
              "address": {
                "type": "string",
                "format": "email"
              },
              "type": {
                "type": "string",
                "enum": ["primary", "secondary", "emergency"]
              },
              "verified": {
                "type": "boolean"
              }
            }
          }
        },
        "phone": {
          "type": "array",
          "minItems": 1,
          "items": {
            "type": "object",
            "properties": {
              "number": {
                "type": "string",
                "pattern": "^\\+[1-9]\\d{1,14}$"
              },
              "type": {
                "type": "string",
                "enum": ["mobile", "home", "work"]
              },
              "verified": {
                "type": "boolean"
              }
            }
          }
        },
        "address": {
          "type": "object",
          "properties": {
            "street": {
              "type": "string"
            },
            "city": {
              "type": "string"
            },
            "state": {
              "type": "string"
            },
            "postalCode": {
              "type": "string"
            },
            "country": {
              "type": "string",
              "pattern": "^[A-Z]{2}$"
            }
          }
        },
        "emergencyContact": {
          "type": "object",
          "properties": {
            "name": {
              "type": "string"
            },
            "relationship": {
              "type": "string"
            },
            "phone": {
              "type": "string"
            },
            "email": {
              "type": "string",
              "format": "email"
            }
          }
        }
      }
    },
    "legalAuthority": {
      "type": "object",
      "required": ["documents", "jurisdiction"],
      "properties": {
        "documents": {
          "type": "array",
          "minItems": 1,
          "items": {
            "type": "object",
            "properties": {
              "type": {
                "type": "string",
                "enum": ["will", "power-of-attorney", "court-order", "trust-document", "letter-of-appointment"]
              },
              "documentId": {
                "type": "string"
              },
              "issuedBy": {
                "type": "string"
              },
              "issuedDate": {
                "type": "string",
                "format": "date"
              },
              "effectiveDate": {
                "type": "string",
                "format": "date"
              },
              "expiryDate": {
                "type": "string",
                "format": "date"
              },
              "verified": {
                "type": "boolean"
              },
              "verifiedBy": {
                "type": "string"
              },
              "verificationDate": {
                "type": "string",
                "format": "date-time"
              },
              "documentUrl": {
                "type": "string",
                "format": "uri"
              },
              "checksum": {
                "type": "string"
              }
            }
          }
        },
        "jurisdiction": {
          "type": "string",
          "pattern": "^[A-Z]{2}$"
        },
        "legalRepresentation": {
          "type": "object",
          "properties": {
            "hasAttorney": {
              "type": "boolean"
            },
            "attorneyName": {
              "type": "string"
            },
            "barNumber": {
              "type": "string"
            },
            "contactInfo": {
              "type": "object"
            }
          }
        }
      }
    },
    "succession": {
      "type": "object",
      "properties": {
        "backupExecutors": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "executorId": {
                "type": "string",
                "format": "uuid"
              },
              "priority": {
                "type": "integer",
                "minimum": 1
              },
              "activationConditions": {
                "type": "array",
                "items": {
                  "type": "string",
                  "enum": ["primary-unavailable", "primary-deceased", "primary-incapacitated", "time-based", "task-specific"]
                }
              }
            }
          }
        },
        "delegatedAuthorities": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "delegatedTo": {
                "type": "string",
                "format": "uuid"
              },
              "scope": {
                "type": "array",
                "items": {
                  "type": "string"
                }
              },
              "validFrom": {
                "type": "string",
                "format": "date-time"
              },
              "validUntil": {
                "type": "string",
                "format": "date-time"
              }
            }
          }
        }
      }
    },
    "training": {
      "type": "object",
      "properties": {
        "certifications": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "name": {
                "type": "string"
              },
              "issuedBy": {
                "type": "string"
              },
              "issuedDate": {
                "type": "string",
                "format": "date"
              },
              "expiryDate": {
                "type": "string",
                "format": "date"
              },
              "certificateUrl": {
                "type": "string",
                "format": "uri"
              }
            }
          }
        },
        "completedTraining": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "module": {
                "type": "string"
              },
              "completedDate": {
                "type": "string",
                "format": "date-time"
              },
              "score": {
                "type": "number",
                "minimum": 0,
                "maximum": 100
              }
            }
          }
        }
      }
    },
    "preferences": {
      "type": "object",
      "properties": {
        "language": {
          "type": "string",
          "pattern": "^[a-z]{2}$"
        },
        "timezone": {
          "type": "string"
        },
        "notificationPreferences": {
          "type": "object",
          "properties": {
            "email": {
              "type": "boolean"
            },
            "sms": {
              "type": "boolean"
            },
            "push": {
              "type": "boolean"
            },
            "frequency": {
              "type": "string",
              "enum": ["immediate", "daily-digest", "weekly-summary"]
            }
          }
        },
        "reportingPreferences": {
          "type": "object",
          "properties": {
            "frequency": {
              "type": "string",
              "enum": ["weekly", "monthly", "quarterly"]
            },
            "format": {
              "type": "string",
              "enum": ["pdf", "html", "json", "csv"]
            },
            "recipients": {
              "type": "array",
              "items": {
                "type": "string",
                "format": "email"
              }
            }
          }
        }
      }
    },
    "security": {
      "type": "object",
      "required": ["mfaEnabled", "lastSecurityAudit"],
      "properties": {
        "mfaEnabled": {
          "type": "boolean"
        },
        "mfaMethods": {
          "type": "array",
          "items": {
            "type": "string",
            "enum": ["totp", "sms", "email", "hardware-key", "biometric"]
          }
        },
        "lastSecurityAudit": {
          "type": "string",
          "format": "date-time"
        },
        "securityQuestions": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "question": {
                "type": "string"
              },
              "answerHash": {
                "type": "string"
              }
            }
          }
        },
        "accessRestrictions": {
          "type": "object",
          "properties": {
            "allowedIpRanges": {
              "type": "array",
              "items": {
                "type": "string"
              }
            },
            "allowedCountries": {
              "type": "array",
              "items": {
                "type": "string",
                "pattern": "^[A-Z]{2}$"
              }
            },
            "timeRestrictions": {
              "type": "object",
              "properties": {
                "allowedHours": {
                  "type": "string"
                },
                "timezone": {
                  "type": "string"
                }
              }
            }
          }
        }
      }
    }
  }
}
```

### 3.2 Executor Role Comparison

| Role Type | Access Level | Decision Authority | Appointment Method | Typical Use Case |
|-----------|--------------|-------------------|-------------------|------------------|
| **Primary Executor** | Full | Independent | Will or court | Main estate manager |
| **Co-Executor** | Full | Joint decisions | Will or agreement | Shared responsibility |
| **Backup Executor** | Limited until activated | None until activated | Succession plan | Contingency planning |
| **Specialist Executor** | Domain-specific | Limited to specialty | Appointment letter | Technical expertise |
| **Legal Advisor** | Read-only | Advisory only | Professional engagement | Legal compliance |

---
