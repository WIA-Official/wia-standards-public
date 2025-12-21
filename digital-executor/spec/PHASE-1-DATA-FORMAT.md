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

## 4. Task Management Schema

### 4.1 Task Definition Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Executor Task",
  "type": "object",
  "required": ["taskId", "title", "category", "priority", "status", "assignedTo"],
  "properties": {
    "taskId": {
      "type": "string",
      "format": "uuid"
    },
    "title": {
      "type": "string",
      "minLength": 1,
      "maxLength": 200
    },
    "description": {
      "type": "string",
      "maxLength": 5000
    },
    "category": {
      "type": "string",
      "enum": [
        "account-access",
        "data-download",
        "account-closure",
        "asset-transfer",
        "notification",
        "legal-compliance",
        "documentation",
        "beneficiary-communication",
        "platform-liaison",
        "audit"
      ]
    },
    "priority": {
      "type": "string",
      "enum": ["critical", "high", "medium", "low"]
    },
    "status": {
      "type": "string",
      "enum": ["pending", "in-progress", "blocked", "completed", "cancelled"]
    },
    "assignedTo": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "platform": {
      "type": "string",
      "description": "Platform or service this task relates to"
    },
    "accountId": {
      "type": "string",
      "description": "Specific account identifier"
    },
    "deadlines": {
      "type": "object",
      "properties": {
        "dueDate": {
          "type": "string",
          "format": "date-time"
        },
        "legalDeadline": {
          "type": "string",
          "format": "date-time"
        },
        "platformDeadline": {
          "type": "string",
          "format": "date-time"
        }
      }
    },
    "dependencies": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "taskId": {
            "type": "string",
            "format": "uuid"
          },
          "type": {
            "type": "string",
            "enum": ["blocks", "blocked-by", "related-to"]
          }
        }
      }
    },
    "checklist": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "item": {
            "type": "string"
          },
          "completed": {
            "type": "boolean"
          },
          "completedAt": {
            "type": "string",
            "format": "date-time"
          },
          "completedBy": {
            "type": "string",
            "format": "uuid"
          },
          "notes": {
            "type": "string"
          }
        }
      }
    },
    "attachments": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "fileId": {
            "type": "string"
          },
          "fileName": {
            "type": "string"
          },
          "fileType": {
            "type": "string"
          },
          "fileSize": {
            "type": "integer"
          },
          "uploadedAt": {
            "type": "string",
            "format": "date-time"
          },
          "uploadedBy": {
            "type": "string",
            "format": "uuid"
          },
          "checksum": {
            "type": "string"
          }
        }
      }
    },
    "approvals": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "approverId": {
            "type": "string",
            "format": "uuid"
          },
          "approverRole": {
            "type": "string"
          },
          "status": {
            "type": "string",
            "enum": ["pending", "approved", "rejected"]
          },
          "timestamp": {
            "type": "string",
            "format": "date-time"
          },
          "comments": {
            "type": "string"
          }
        }
      }
    },
    "estimates": {
      "type": "object",
      "properties": {
        "timeEstimate": {
          "type": "string",
          "description": "ISO 8601 duration"
        },
        "actualTime": {
          "type": "string",
          "description": "ISO 8601 duration"
        },
        "complexity": {
          "type": "string",
          "enum": ["simple", "moderate", "complex", "very-complex"]
        }
      }
    },
    "notifications": {
      "type": "object",
      "properties": {
        "notifyOnStart": {
          "type": "array",
          "items": {
            "type": "string",
            "format": "uuid"
          }
        },
        "notifyOnComplete": {
          "type": "array",
          "items": {
            "type": "string",
            "format": "uuid"
          }
        },
        "notifyOnDeadline": {
          "type": "array",
          "items": {
            "type": "string",
            "format": "uuid"
          }
        },
        "reminderSchedule": {
          "type": "array",
          "items": {
            "type": "string",
            "description": "ISO 8601 duration before deadline"
          }
        }
      }
    },
    "compliance": {
      "type": "object",
      "properties": {
        "requiredDocuments": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "regulatoryRequirements": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "auditTrailRequired": {
          "type": "boolean"
        }
      }
    }
  }
}
```

### 4.2 Task Templates

| Template Type | Priority | Typical Duration | Required Documents | Checklist Items |
|---------------|----------|------------------|-------------------|-----------------|
| **Account Access** | High | 3-7 days | Death certificate, executor letter | Verify identity, submit request, receive credentials, document access |
| **Data Download** | Medium | 1-3 days | Access credentials | Request archive, verify completeness, store securely, confirm receipt |
| **Account Closure** | Medium | 7-14 days | Legal authority, beneficiary consent | Backup data, notify contacts, close account, confirm deletion |
| **Asset Transfer** | High | 14-30 days | Transfer authorization, beneficiary ID | Identify assets, value assets, get approvals, execute transfer, confirm receipt |
| **Legal Filing** | Critical | Varies | Court documents, compliance proof | Research requirements, prepare docs, file with authority, track status |

---

## 5. Access Control Schema

### 5.1 Access Grant Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Platform Access Grant",
  "type": "object",
  "required": ["grantId", "executorId", "platformId", "accessLevel", "grantedAt"],
  "properties": {
    "grantId": {
      "type": "string",
      "format": "uuid"
    },
    "executorId": {
      "type": "string",
      "format": "uuid"
    },
    "platformId": {
      "type": "string"
    },
    "platformName": {
      "type": "string"
    },
    "accountId": {
      "type": "string",
      "description": "Decedent's account ID on platform"
    },
    "accessLevel": {
      "type": "string",
      "enum": ["full", "read-only", "limited", "emergency"]
    },
    "permissions": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": [
          "view-profile",
          "view-content",
          "download-data",
          "delete-content",
          "post-memorial",
          "close-account",
          "transfer-assets",
          "view-messages",
          "send-messages",
          "manage-settings"
        ]
      }
    },
    "credentials": {
      "type": "object",
      "properties": {
        "method": {
          "type": "string",
          "enum": ["password", "oauth", "api-key", "certificate", "legacy-contact"]
        },
        "credentialId": {
          "type": "string",
          "description": "Reference to encrypted credential storage"
        },
        "lastRotated": {
          "type": "string",
          "format": "date-time"
        },
        "expiresAt": {
          "type": "string",
          "format": "date-time"
        }
      }
    },
    "grantedAt": {
      "type": "string",
      "format": "date-time"
    },
    "grantedBy": {
      "type": "string",
      "description": "Platform or authority granting access"
    },
    "validFrom": {
      "type": "string",
      "format": "date-time"
    },
    "validUntil": {
      "type": "string",
      "format": "date-time"
    },
    "revocationConditions": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "condition": {
            "type": "string"
          },
          "autoRevoke": {
            "type": "boolean"
          }
        }
      }
    },
    "restrictions": {
      "type": "object",
      "properties": {
        "ipWhitelist": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "geofence": {
          "type": "array",
          "items": {
            "type": "string",
            "pattern": "^[A-Z]{2}$"
          }
        },
        "timeWindows": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "start": {
                "type": "string",
                "format": "time"
              },
              "end": {
                "type": "string",
                "format": "time"
              },
              "timezone": {
                "type": "string"
              }
            }
          }
        },
        "maxSessions": {
          "type": "integer",
          "minimum": 1
        },
        "sessionDuration": {
          "type": "string",
          "description": "ISO 8601 duration"
        }
      }
    },
    "monitoring": {
      "type": "object",
      "properties": {
        "logAllActions": {
          "type": "boolean"
        },
        "alertOnSuspicious": {
          "type": "boolean"
        },
        "requireReasonForAccess": {
          "type": "boolean"
        },
        "notifyBeneficiaries": {
          "type": "boolean"
        }
      }
    },
    "supportingDocuments": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid",
        "description": "Reference to legal authority documents"
      }
    }
  }
}
```

### 5.2 Credential Transfer Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Credential Transfer",
  "type": "object",
  "required": ["transferId", "fromExecutor", "toExecutor", "credentials", "reason"],
  "properties": {
    "transferId": {
      "type": "string",
      "format": "uuid"
    },
    "fromExecutor": {
      "type": "string",
      "format": "uuid"
    },
    "toExecutor": {
      "type": "string",
      "format": "uuid"
    },
    "credentials": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "credentialId": {
            "type": "string"
          },
          "platform": {
            "type": "string"
          },
          "accessLevel": {
            "type": "string"
          }
        }
      }
    },
    "reason": {
      "type": "string",
      "enum": ["executor-change", "delegation", "succession", "emergency", "specialization"]
    },
    "initiatedAt": {
      "type": "string",
      "format": "date-time"
    },
    "approvals": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "approvedBy": {
            "type": "string",
            "format": "uuid"
          },
          "role": {
            "type": "string"
          },
          "approvedAt": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "completedAt": {
      "type": "string",
      "format": "date-time"
    },
    "securityMeasures": {
      "type": "object",
      "properties": {
        "encryptionMethod": {
          "type": "string"
        },
        "verificationRequired": {
          "type": "boolean"
        },
        "notifyStakeholders": {
          "type": "boolean"
        }
      }
    }
  }
}
```

### 5.3 Access Control Matrix

| Executor Role | View Assets | Download Data | Delete Content | Transfer Assets | Close Accounts | Appoint Successor |
|---------------|-------------|---------------|----------------|-----------------|----------------|-------------------|
| **Primary** | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| **Co-Executor** | ✓ | ✓ | With approval | With approval | With approval | ✗ |
| **Backup** | Limited | ✗ | ✗ | ✗ | ✗ | ✗ |
| **Specialist** | Domain only | Domain only | With approval | Domain only | ✗ | ✗ |
| **Legal Advisor** | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |

---

## 6. Legal Authority Schema

### 6.1 Legal Document Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Legal Authority Document",
  "type": "object",
  "required": ["documentId", "type", "jurisdiction", "issuedDate", "status"],
  "properties": {
    "documentId": {
      "type": "string",
      "format": "uuid"
    },
    "type": {
      "type": "string",
      "enum": [
        "will",
        "codicil",
        "power-of-attorney",
        "court-order",
        "letters-testamentary",
        "letters-of-administration",
        "trust-document",
        "executor-appointment",
        "beneficiary-designation",
        "platform-authorization"
      ]
    },
    "jurisdiction": {
      "type": "object",
      "required": ["country"],
      "properties": {
        "country": {
          "type": "string",
          "pattern": "^[A-Z]{2}$"
        },
        "state": {
          "type": "string"
        },
        "county": {
          "type": "string"
        },
        "court": {
          "type": "string"
        }
      }
    },
    "issuedBy": {
      "type": "object",
      "properties": {
        "entity": {
          "type": "string"
        },
        "authorityType": {
          "type": "string",
          "enum": ["court", "notary", "attorney", "government-agency", "decedent"]
        },
        "officialName": {
          "type": "string"
        },
        "licenseNumber": {
          "type": "string"
        },
        "contactInfo": {
          "type": "object"
        }
      }
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
    "status": {
      "type": "string",
      "enum": ["draft", "pending-review", "verified", "active", "expired", "revoked", "superseded"]
    },
    "verification": {
      "type": "object",
      "required": ["verified", "verifiedAt"],
      "properties": {
        "verified": {
          "type": "boolean"
        },
        "verifiedBy": {
          "type": "string"
        },
        "verifiedAt": {
          "type": "string",
          "format": "date-time"
        },
        "verificationMethod": {
          "type": "string",
          "enum": ["manual-review", "automated", "third-party", "notarized", "court-certified"]
        },
        "verificationNotes": {
          "type": "string"
        },
        "revalidationRequired": {
          "type": "boolean"
        },
        "revalidationDate": {
          "type": "string",
          "format": "date"
        }
      }
    },
    "content": {
      "type": "object",
      "properties": {
        "summary": {
          "type": "string"
        },
        "executorProvisions": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "provision": {
                "type": "string"
              },
              "executorId": {
                "type": "string",
                "format": "uuid"
              },
              "scope": {
                "type": "string"
              }
            }
          }
        },
        "restrictions": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "specialInstructions": {
          "type": "array",
          "items": {
            "type": "string"
          }
        }
      }
    },
    "storage": {
      "type": "object",
      "properties": {
        "originalLocation": {
          "type": "string"
        },
        "digitalCopyUrl": {
          "type": "string",
          "format": "uri"
        },
        "backupLocations": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "checksum": {
          "type": "string"
        },
        "encrypted": {
          "type": "boolean"
        },
        "encryptionMethod": {
          "type": "string"
        }
      }
    },
    "attestations": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "witnessName": {
            "type": "string"
          },
          "witnessSignature": {
            "type": "string"
          },
          "attestationDate": {
            "type": "string",
            "format": "date"
          },
          "witnessType": {
            "type": "string",
            "enum": ["witness", "notary", "court-official"]
          }
        }
      }
    },
    "amendments": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "amendmentId": {
            "type": "string"
          },
          "amendmentDate": {
            "type": "string",
            "format": "date"
          },
          "description": {
            "type": "string"
          },
          "documentReference": {
            "type": "string"
          }
        }
      }
    }
  }
}
```

---

## 7. Progress Tracking Schema

### 7.1 Progress Report Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Executor Progress Report",
  "type": "object",
  "required": ["reportId", "executorId", "reportPeriod", "generatedAt"],
  "properties": {
    "reportId": {
      "type": "string",
      "format": "uuid"
    },
    "executorId": {
      "type": "string",
      "format": "uuid"
    },
    "reportPeriod": {
      "type": "object",
      "required": ["startDate", "endDate"],
      "properties": {
        "startDate": {
          "type": "string",
          "format": "date"
        },
        "endDate": {
          "type": "string",
          "format": "date"
        }
      }
    },
    "generatedAt": {
      "type": "string",
      "format": "date-time"
    },
    "summary": {
      "type": "object",
      "properties": {
        "totalTasks": {
          "type": "integer"
        },
        "completedTasks": {
          "type": "integer"
        },
        "inProgressTasks": {
          "type": "integer"
        },
        "blockedTasks": {
          "type": "integer"
        },
        "platformsAccessed": {
          "type": "integer"
        },
        "dataDownloaded": {
          "type": "string",
          "description": "Size in bytes"
        },
        "accountsClosed": {
          "type": "integer"
        },
        "assetsTransferred": {
          "type": "integer"
        }
      }
    },
    "taskBreakdown": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "category": {
            "type": "string"
          },
          "total": {
            "type": "integer"
          },
          "completed": {
            "type": "integer"
          },
          "completionRate": {
            "type": "number",
            "minimum": 0,
            "maximum": 100
          }
        }
      }
    },
    "platformProgress": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "platform": {
            "type": "string"
          },
          "status": {
            "type": "string",
            "enum": ["not-started", "in-progress", "completed", "blocked"]
          },
          "tasksCompleted": {
            "type": "integer"
          },
          "totalTasks": {
            "type": "integer"
          },
          "lastActivity": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "milestones": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "milestone": {
            "type": "string"
          },
          "targetDate": {
            "type": "string",
            "format": "date"
          },
          "actualDate": {
            "type": "string",
            "format": "date"
          },
          "status": {
            "type": "string",
            "enum": ["pending", "on-track", "at-risk", "delayed", "completed"]
          }
        }
      }
    },
    "issues": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "issueId": {
            "type": "string"
          },
          "severity": {
            "type": "string",
            "enum": ["low", "medium", "high", "critical"]
          },
          "description": {
            "type": "string"
          },
          "impact": {
            "type": "string"
          },
          "resolution": {
            "type": "string"
          },
          "status": {
            "type": "string",
            "enum": ["open", "in-progress", "resolved"]
          }
        }
      }
    },
    "beneficiaryUpdates": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "beneficiaryId": {
            "type": "string"
          },
          "updateSent": {
            "type": "string",
            "format": "date-time"
          },
          "method": {
            "type": "string",
            "enum": ["email", "portal", "mail", "phone"]
          },
          "acknowledged": {
            "type": "boolean"
          }
        }
      }
    },
    "complianceStatus": {
      "type": "object",
      "properties": {
        "legalRequirementsMet": {
          "type": "integer"
        },
        "totalLegalRequirements": {
          "type": "integer"
        },
        "outstandingFilings": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "filing": {
                "type": "string"
              },
              "deadline": {
                "type": "string",
                "format": "date"
              },
              "status": {
                "type": "string"
              }
            }
          }
        }
      }
    },
    "timeTracking": {
      "type": "object",
      "properties": {
        "totalHours": {
          "type": "number"
        },
        "hoursByCategory": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "category": {
                "type": "string"
              },
              "hours": {
                "type": "number"
              }
            }
          }
        }
      }
    },
    "nextSteps": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "action": {
            "type": "string"
          },
          "targetDate": {
            "type": "string",
            "format": "date"
          },
          "assignedTo": {
            "type": "string"
          }
        }
      }
    }
  }
}
```

---

## 8. Communication Schema

### 8.1 Beneficiary Communication Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Beneficiary Communication",
  "type": "object",
  "required": ["communicationId", "fromExecutor", "toBeneficiaries", "subject", "sentAt"],
  "properties": {
    "communicationId": {
      "type": "string",
      "format": "uuid"
    },
    "fromExecutor": {
      "type": "string",
      "format": "uuid"
    },
    "toBeneficiaries": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "ccExecutors": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "subject": {
      "type": "string"
    },
    "messageType": {
      "type": "string",
      "enum": ["initial-notification", "progress-update", "asset-distribution", "account-closure", "action-required", "completion-notice"]
    },
    "content": {
      "type": "object",
      "properties": {
        "body": {
          "type": "string"
        },
        "format": {
          "type": "string",
          "enum": ["plain-text", "html", "markdown"]
        },
        "language": {
          "type": "string",
          "pattern": "^[a-z]{2}$"
        }
      }
    },
    "attachments": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "fileId": {
            "type": "string"
          },
          "fileName": {
            "type": "string"
          },
          "fileType": {
            "type": "string"
          },
          "description": {
            "type": "string"
          }
        }
      }
    },
    "sentAt": {
      "type": "string",
      "format": "date-time"
    },
    "deliveryMethod": {
      "type": "string",
      "enum": ["email", "portal", "postal-mail", "sms", "secure-message"]
    },
    "deliveryStatus": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "beneficiaryId": {
            "type": "string",
            "format": "uuid"
          },
          "status": {
            "type": "string",
            "enum": ["sent", "delivered", "read", "acknowledged", "bounced", "failed"]
          },
          "timestamp": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "requiresResponse": {
      "type": "boolean"
    },
    "responseDeadline": {
      "type": "string",
      "format": "date-time"
    },
    "responses": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "beneficiaryId": {
            "type": "string",
            "format": "uuid"
          },
          "response": {
            "type": "string"
          },
          "respondedAt": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "template": {
      "type": "object",
      "properties": {
        "templateId": {
          "type": "string"
        },
        "variables": {
          "type": "object"
        }
      }
    },
    "compliance": {
      "type": "object",
      "properties": {
        "retentionPeriod": {
          "type": "string",
          "description": "ISO 8601 duration"
        },
        "privacyLevel": {
          "type": "string",
          "enum": ["public", "confidential", "restricted"]
        },
        "archiveRequired": {
          "type": "boolean"
        }
      }
    }
  }
}
```

---

## 9. Audit Trail Schema

### 9.1 Audit Log Entry Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Executor Audit Log Entry",
  "type": "object",
  "required": ["logId", "executorId", "action", "timestamp"],
  "properties": {
    "logId": {
      "type": "string",
      "format": "uuid"
    },
    "executorId": {
      "type": "string",
      "format": "uuid"
    },
    "action": {
      "type": "string",
      "enum": [
        "login",
        "logout",
        "view-asset",
        "download-data",
        "delete-content",
        "transfer-asset",
        "close-account",
        "update-task",
        "send-communication",
        "update-permissions",
        "access-credentials",
        "upload-document",
        "verify-legal-authority",
        "generate-report",
        "delegate-authority"
      ]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time"
    },
    "platform": {
      "type": "string"
    },
    "accountId": {
      "type": "string"
    },
    "details": {
      "type": "object",
      "properties": {
        "description": {
          "type": "string"
        },
        "affectedResources": {
          "type": "array",
          "items": {
            "type": "string"
          }
        },
        "previousState": {
          "type": "object"
        },
        "newState": {
          "type": "object"
        }
      }
    },
    "context": {
      "type": "object",
      "properties": {
        "ipAddress": {
          "type": "string"
        },
        "userAgent": {
          "type": "string"
        },
        "location": {
          "type": "object",
          "properties": {
            "country": {
              "type": "string"
            },
            "city": {
              "type": "string"
            },
            "coordinates": {
              "type": "object",
              "properties": {
                "latitude": {
                  "type": "number"
                },
                "longitude": {
                  "type": "number"
                }
              }
            }
          }
        },
        "sessionId": {
          "type": "string"
        },
        "requestId": {
          "type": "string"
        }
      }
    },
    "result": {
      "type": "string",
      "enum": ["success", "failure", "partial", "blocked"]
    },
    "errorDetails": {
      "type": "object",
      "properties": {
        "errorCode": {
          "type": "string"
        },
        "errorMessage": {
          "type": "string"
        },
        "stackTrace": {
          "type": "string"
        }
      }
    },
    "securityFlags": {
      "type": "array",
      "items": {
        "type": "string",
        "enum": ["suspicious-location", "unusual-time", "rapid-actions", "high-risk-action", "unauthorized-attempt"]
      }
    },
    "approvals": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "approverExecutorId": {
            "type": "string"
          },
          "approvedAt": {
            "type": "string",
            "format": "date-time"
          }
        }
      }
    },
    "relatedLogs": {
      "type": "array",
      "items": {
        "type": "string",
        "format": "uuid"
      }
    },
    "immutabilityProof": {
      "type": "object",
      "properties": {
        "hash": {
          "type": "string"
        },
        "previousHash": {
          "type": "string"
        },
        "signature": {
          "type": "string"
        }
      }
    }
  }
}
```

---

## 10. Data Validation Rules

### 10.1 Validation Requirements

| Data Type | Validation Rules | Error Handling | Example |
|-----------|-----------------|----------------|---------|
| **Executor ID** | UUID v4 format, exists in database | Return 404 if not found | `550e8400-e29b-41d4-a716-446655440000` |
| **Email Address** | RFC 5322 compliant, verified | Reject unverified emails | `executor@example.com` |
| **Phone Number** | E.164 format, verified | Require verification code | `+14155552671` |
| **Date Fields** | ISO 8601, not in future (except deadlines) | Reject invalid dates | `2025-12-18T10:30:00Z` |
| **Legal Documents** | Valid checksum, verified status | Require re-verification | SHA-256 hash |
| **Access Permissions** | Valid role + permission combination | Deny invalid combinations | Primary executor + full access |

### 10.2 Business Logic Validation

```javascript
// Validation Rules Examples
const validationRules = {
  executorAppointment: {
    mustHaveLegalAuthority: true,
    mustBeAdult: true, // Age >= 18
    cannotBeBeneficiaryOnly: false, // Can be both
    requiresBackupExecutor: true,
    minimumExecutors: 1,
    maximumCoExecutors: 5
  },

  taskAssignment: {
    mustMatchExecutorSpecialization: false, // Recommended but not required
    cannotExceedPermissions: true,
    requiresApprovalForCritical: true,
    mustHaveDeadline: true,
    cannotBePastDue: false // Can assign past due tasks
  },

  accessGrant: {
    requiresLegalDocumentation: true,
    mustHaveExpiryDate: true,
    cannotExceedExecutorPermissions: true,
    requiresPlatformApproval: true,
    mustLogAllAccess: true
  },

  dataTransfer: {
    requiresEncryption: true,
    mustVerifyRecipient: true,
    requiresAuditTrail: true,
    mustNotifyStakeholders: true,
    cannotTransferToUnverifiedExecutor: true
  }
};
```

### 10.3 Data Integrity Checks

```javascript
// Data Integrity Verification
class DataIntegrityValidator {
  static validateExecutorProfile(profile) {
    const checks = {
      checksumValid: this.verifyChecksum(profile),
      versionsConsistent: this.checkVersionConsistency(profile),
      relationshipsValid: this.validateRelationships(profile),
      permissionsCoherent: this.checkPermissionLogic(profile),
      datesLogical: this.validateDateSequence(profile)
    };

    return {
      isValid: Object.values(checks).every(v => v === true),
      checks: checks,
      errors: this.generateErrorReport(checks)
    };
  }

  static verifyChecksum(data) {
    const computed = this.computeSHA256(JSON.stringify(data));
    return computed === data.checksum;
  }

  static validateDateSequence(profile) {
    const dates = {
      appointed: profile.role.appointmentDate,
      effective: profile.role.effectiveDate,
      expiry: profile.role.expiryDate
    };

    // Appointed <= Effective < Expiry
    return dates.appointed <= dates.effective &&
           dates.effective < dates.expiry;
  }

  static checkPermissionLogic(profile) {
    const role = profile.role.type;
    const accessLevel = profile.permissions.accessLevel;

    // Primary executor must have full access
    if (role === 'primary' && accessLevel !== 'full') {
      return false;
    }

    // Backup executor cannot have full access until activated
    if (role === 'backup' && accessLevel === 'full') {
      return false;
    }

    return true;
  }
}
```

---

## 11. Code Examples

### 11.1 Creating an Executor Profile

```javascript
// Example: Creating a new executor profile
async function createExecutorProfile(executorData) {
  const profile = {
    executorId: generateUUID(),
    personalInfo: {
      firstName: executorData.firstName,
      lastName: executorData.lastName,
      dateOfBirth: executorData.dateOfBirth,
      citizenship: [executorData.country],
      governmentIds: [{
        type: "passport",
        number: executorData.passportNumber,
        issuingCountry: executorData.country,
        expiryDate: executorData.passportExpiry,
        verified: false
      }]
    },
    role: {
      type: executorData.roleType || "primary",
      appointmentDate: new Date().toISOString(),
      effectiveDate: executorData.effectiveDate || new Date().toISOString(),
      status: "designated",
      priority: executorData.priority || 1,
      specialization: executorData.specializations || ["general"]
    },
    permissions: {
      accessLevel: executorData.roleType === "primary" ? "full" : "limited",
      allowedActions: getDefaultActionsForRole(executorData.roleType),
      restrictions: [],
      delegationAllowed: executorData.roleType === "primary",
      requiresApproval: executorData.roleType !== "primary" ?
        ["delete-accounts", "transfer-assets"] : []
    },
    contactInfo: {
      email: [{
        address: executorData.email,
        type: "primary",
        verified: false
      }],
      phone: [{
        number: executorData.phone,
        type: "mobile",
        verified: false
      }],
      address: executorData.address
    },
    legalAuthority: {
      documents: [],
      jurisdiction: executorData.jurisdiction,
      legalRepresentation: {
        hasAttorney: false
      }
    },
    succession: {
      backupExecutors: [],
      delegatedAuthorities: []
    },
    security: {
      mfaEnabled: false,
      mfaMethods: [],
      lastSecurityAudit: new Date().toISOString()
    },
    preferences: {
      language: executorData.language || "en",
      timezone: executorData.timezone || "UTC",
      notificationPreferences: {
        email: true,
        sms: false,
        push: false,
        frequency: "daily-digest"
      }
    }
  };

  // Validate profile
  const validation = DataIntegrityValidator.validateExecutorProfile(profile);
  if (!validation.isValid) {
    throw new Error(`Invalid executor profile: ${validation.errors.join(', ')}`);
  }

  // Compute checksum
  profile.checksum = computeSHA256(JSON.stringify(profile));

  // Save to database
  await saveExecutorProfile(profile);

  // Send verification emails
  await sendVerificationEmail(profile.contactInfo.email[0].address);

  return profile;
}

function getDefaultActionsForRole(roleType) {
  const actionsByRole = {
    primary: [
      "view-assets", "download-data", "delete-accounts",
      "transfer-assets", "communicate-beneficiaries",
      "update-settings", "appoint-successor", "access-credentials",
      "modify-permissions", "generate-reports"
    ],
    "co-executor": [
      "view-assets", "download-data", "communicate-beneficiaries",
      "update-settings", "generate-reports"
    ],
    backup: ["view-assets"],
    specialist: ["view-assets", "download-data", "generate-reports"],
    "legal-advisor": ["view-assets", "generate-reports"]
  };

  return actionsByRole[roleType] || [];
}
```

### 11.2 Managing Task Assignments

```javascript
// Example: Creating and assigning tasks
async function createExecutorTask(taskData) {
  const task = {
    taskId: generateUUID(),
    title: taskData.title,
    description: taskData.description,
    category: taskData.category,
    priority: determinePriority(taskData.category, taskData.deadline),
    status: "pending",
    assignedTo: [taskData.executorId],
    platform: taskData.platform,
    accountId: taskData.accountId,
    deadlines: {
      dueDate: taskData.dueDate,
      legalDeadline: taskData.legalDeadline,
      platformDeadline: taskData.platformDeadline
    },
    dependencies: [],
    checklist: generateChecklist(taskData.category),
    attachments: [],
    approvals: requiresApproval(taskData.category) ?
      [{ approverId: getPrimaryExecutor(), status: "pending" }] : [],
    estimates: {
      timeEstimate: estimateTaskDuration(taskData.category),
      complexity: assessComplexity(taskData)
    },
    notifications: {
      notifyOnStart: [taskData.executorId],
      notifyOnComplete: await getAllStakeholders(),
      notifyOnDeadline: [taskData.executorId],
      reminderSchedule: ["P1D", "P3D", "P7D"] // 1, 3, 7 days before
    },
    compliance: {
      requiredDocuments: getRequiredDocuments(taskData.category),
      regulatoryRequirements: getRegulatoryRequirements(taskData.jurisdiction),
      auditTrailRequired: true
    }
  };

  // Create audit log
  await createAuditLog({
    executorId: taskData.executorId,
    action: "update-task",
    details: { description: `Created task: ${task.title}` },
    result: "success"
  });

  // Save task
  await saveTask(task);

  // Send notifications
  await notifyExecutor(taskData.executorId, "new-task-assigned", task);

  return task;
}

function generateChecklist(category) {
  const checklists = {
    "account-access": [
      { item: "Gather legal documentation", completed: false },
      { item: "Submit access request to platform", completed: false },
      { item: "Verify identity", completed: false },
      { item: "Receive credentials", completed: false },
      { item: "Document access in audit log", completed: false }
    ],
    "data-download": [
      { item: "Request data archive", completed: false },
      { item: "Verify archive completeness", completed: false },
      { item: "Store in secure location", completed: false },
      { item: "Verify checksums", completed: false },
      { item: "Confirm beneficiary notification", completed: false }
    ],
    "account-closure": [
      { item: "Backup all data", completed: false },
      { item: "Notify relevant contacts", completed: false },
      { item: "Remove personal information", completed: false },
      { item: "Submit closure request", completed: false },
      { item: "Confirm account deletion", completed: false },
      { item: "Document closure", completed: false }
    ]
  };

  return checklists[category] || [];
}

function determinePriority(category, deadline) {
  const now = new Date();
  const dueDate = new Date(deadline);
  const daysUntilDue = (dueDate - now) / (1000 * 60 * 60 * 24);

  const criticalCategories = ["legal-compliance", "asset-transfer"];

  if (criticalCategories.includes(category)) {
    return "critical";
  }

  if (daysUntilDue < 7) {
    return "high";
  } else if (daysUntilDue < 30) {
    return "medium";
  }

  return "low";
}
```

### 11.3 Granting Platform Access

```javascript
// Example: Granting executor access to a platform
async function grantPlatformAccess(grantRequest) {
  // Verify executor authority
  const executor = await getExecutor(grantRequest.executorId);
  const legalAuth = await verifyLegalAuthority(executor.legalAuthority);

  if (!legalAuth.verified) {
    throw new Error("Legal authority not verified");
  }

  // Create access grant
  const grant = {
    grantId: generateUUID(),
    executorId: grantRequest.executorId,
    platformId: grantRequest.platformId,
    platformName: grantRequest.platformName,
    accountId: grantRequest.accountId,
    accessLevel: determineAccessLevel(executor.role.type),
    permissions: getPermissionsForAccessLevel(
      executor.role.type,
      grantRequest.platformId
    ),
    credentials: {
      method: grantRequest.credentialMethod,
      credentialId: await storeEncryptedCredentials(grantRequest.credentials),
      lastRotated: new Date().toISOString(),
      expiresAt: calculateExpiryDate(90) // 90 days
    },
    grantedAt: new Date().toISOString(),
    grantedBy: grantRequest.platformId,
    validFrom: new Date().toISOString(),
    validUntil: calculateExpiryDate(365), // 1 year
    revocationConditions: [
      { condition: "executor-role-revoked", autoRevoke: true },
      { condition: "legal-authority-expired", autoRevoke: true },
      { condition: "suspicious-activity", autoRevoke: false }
    ],
    restrictions: {
      ipWhitelist: grantRequest.ipWhitelist || [],
      geofence: [executor.legalAuthority.jurisdiction],
      maxSessions: 3,
      sessionDuration: "PT4H" // 4 hours
    },
    monitoring: {
      logAllActions: true,
      alertOnSuspicious: true,
      requireReasonForAccess: true,
      notifyBeneficiaries: true
    },
    supportingDocuments: [legalAuth.documentId]
  };

  // Save grant
  await saveAccessGrant(grant);

  // Create audit log
  await createAuditLog({
    executorId: grantRequest.executorId,
    action: "access-credentials",
    platform: grantRequest.platformId,
    accountId: grantRequest.accountId,
    details: {
      description: `Granted ${grant.accessLevel} access to ${grant.platformName}`,
      affectedResources: [grant.grantId]
    },
    result: "success"
  });

  // Notify stakeholders
  await notifyBeneficiaries({
    type: "executor-access-granted",
    platform: grant.platformName,
    executor: executor.personalInfo.firstName + " " + executor.personalInfo.lastName,
    accessLevel: grant.accessLevel
  });

  return grant;
}

function determineAccessLevel(roleType) {
  const accessLevels = {
    primary: "full",
    "co-executor": "full",
    backup: "read-only",
    specialist: "limited",
    "legal-advisor": "read-only"
  };

  return accessLevels[roleType] || "limited";
}

async function storeEncryptedCredentials(credentials) {
  const encrypted = await encryptAES256(JSON.stringify(credentials));
  const credentialId = generateUUID();

  await saveToSecureVault({
    id: credentialId,
    data: encrypted,
    encryptedAt: new Date().toISOString()
  });

  return credentialId;
}
```

### 11.4 Tracking Progress

```javascript
// Example: Generating progress report
async function generateProgressReport(executorId, period) {
  const executor = await getExecutor(executorId);
  const tasks = await getTasksInPeriod(executorId, period.startDate, period.endDate);
  const platforms = await getPlatformAccessByExecutor(executorId);

  const report = {
    reportId: generateUUID(),
    executorId: executorId,
    reportPeriod: period,
    generatedAt: new Date().toISOString(),
    summary: {
      totalTasks: tasks.length,
      completedTasks: tasks.filter(t => t.status === "completed").length,
      inProgressTasks: tasks.filter(t => t.status === "in-progress").length,
      blockedTasks: tasks.filter(t => t.status === "blocked").length,
      platformsAccessed: platforms.length,
      dataDownloaded: await calculateTotalDataDownloaded(executorId, period),
      accountsClosed: await countClosedAccounts(executorId, period),
      assetsTransferred: await countTransferredAssets(executorId, period)
    },
    taskBreakdown: calculateTaskBreakdown(tasks),
    platformProgress: await calculatePlatformProgress(platforms, tasks),
    milestones: await checkMilestones(executorId),
    issues: await getOpenIssues(executorId),
    beneficiaryUpdates: await getBeneficiaryUpdates(executorId, period),
    complianceStatus: await checkComplianceStatus(executorId),
    timeTracking: await calculateTimeTracking(executorId, period),
    nextSteps: await generateNextSteps(tasks)
  };

  // Save report
  await saveReport(report);

  // Send to stakeholders
  await distributeReport(report, executor.preferences.reportingPreferences);

  return report;
}

function calculateTaskBreakdown(tasks) {
  const categories = [...new Set(tasks.map(t => t.category))];

  return categories.map(category => {
    const categoryTasks = tasks.filter(t => t.category === category);
    const completed = categoryTasks.filter(t => t.status === "completed").length;

    return {
      category: category,
      total: categoryTasks.length,
      completed: completed,
      completionRate: (completed / categoryTasks.length) * 100
    };
  });
}

async function calculatePlatformProgress(platforms, tasks) {
  return Promise.all(platforms.map(async platform => {
    const platformTasks = tasks.filter(t => t.platform === platform.platformName);
    const completed = platformTasks.filter(t => t.status === "completed");
    const lastActivity = await getLastPlatformActivity(platform.platformId);

    let status = "not-started";
    if (completed.length === platformTasks.length) {
      status = "completed";
    } else if (completed.length > 0) {
      status = "in-progress";
    } else if (platformTasks.some(t => t.status === "blocked")) {
      status = "blocked";
    }

    return {
      platform: platform.platformName,
      status: status,
      tasksCompleted: completed.length,
      totalTasks: platformTasks.length,
      lastActivity: lastActivity
    };
  }));
}
```

### 11.5 Managing Succession

```javascript
// Example: Setting up backup executor succession
async function setupExecutorSuccession(primaryExecutorId, backupData) {
  const primaryExecutor = await getExecutor(primaryExecutorId);

  // Verify primary executor has authority to appoint backup
  if (!primaryExecutor.permissions.allowedActions.includes("appoint-successor")) {
    throw new Error("Primary executor does not have succession appointment authority");
  }

  // Create backup executor profile
  const backupExecutor = await createExecutorProfile({
    ...backupData,
    roleType: "backup"
  });

  // Set up succession plan
  const succession = {
    backupExecutors: [
      ...(primaryExecutor.succession.backupExecutors || []),
      {
        executorId: backupExecutor.executorId,
        priority: backupData.priority || 1,
        activationConditions: [
          "primary-unavailable",
          "primary-deceased",
          "primary-incapacitated"
        ]
      }
    ],
    delegatedAuthorities: []
  };

  // Update primary executor profile
  await updateExecutor(primaryExecutorId, {
    succession: succession
  });

  // Create notification for backup executor
  await sendCommunication({
    fromExecutor: primaryExecutorId,
    toBeneficiaries: [backupExecutor.executorId],
    subject: "Digital Executor Backup Appointment",
    messageType: "initial-notification",
    content: {
      body: `You have been designated as a backup digital executor.
             You will be notified if your services are required.
             Please review the attached documentation.`,
      format: "html",
      language: backupExecutor.preferences.language
    },
    deliveryMethod: "email",
    requiresResponse: true,
    responseDeadline: calculateFutureDate(14) // 14 days to respond
  });

  // Create audit log
  await createAuditLog({
    executorId: primaryExecutorId,
    action: "appoint-successor",
    details: {
      description: `Appointed backup executor: ${backupExecutor.executorId}`,
      affectedResources: [backupExecutor.executorId]
    },
    result: "success"
  });

  return {
    primaryExecutor: primaryExecutor,
    backupExecutor: backupExecutor,
    succession: succession
  };
}

// Example: Activating backup executor
async function activateBackupExecutor(backupExecutorId, activationReason) {
  const backupExecutor = await getExecutor(backupExecutorId);

  // Verify backup executor exists and is designated
  if (backupExecutor.role.type !== "backup") {
    throw new Error("Executor is not a backup executor");
  }

  // Update role to active
  await updateExecutor(backupExecutorId, {
    role: {
      ...backupExecutor.role,
      type: "primary",
      status: "active",
      effectiveDate: new Date().toISOString()
    },
    permissions: {
      ...backupExecutor.permissions,
      accessLevel: "full",
      allowedActions: getDefaultActionsForRole("primary")
    }
  });

  // Transfer access grants
  const primaryExecutor = await findPrimaryExecutor();
  if (primaryExecutor) {
    await transferAllAccessGrants(primaryExecutor.executorId, backupExecutorId);
  }

  // Notify all stakeholders
  await notifyStakeholders({
    type: "executor-succession",
    formerExecutor: primaryExecutor?.executorId,
    newExecutor: backupExecutorId,
    reason: activationReason
  });

  // Create audit log
  await createAuditLog({
    executorId: backupExecutorId,
    action: "delegate-authority",
    details: {
      description: `Backup executor activated due to: ${activationReason}`,
      affectedResources: [backupExecutorId]
    },
    result: "success"
  });

  return await getExecutor(backupExecutorId);
}
```

### 11.6 Audit Trail Creation

```javascript
// Example: Creating comprehensive audit logs
async function createAuditLog(logData) {
  const log = {
    logId: generateUUID(),
    executorId: logData.executorId,
    action: logData.action,
    timestamp: new Date().toISOString(),
    platform: logData.platform,
    accountId: logData.accountId,
    details: {
      description: logData.details?.description,
      affectedResources: logData.details?.affectedResources || [],
      previousState: logData.details?.previousState,
      newState: logData.details?.newState
    },
    context: await captureRequestContext(),
    result: logData.result,
    errorDetails: logData.errorDetails,
    securityFlags: analyzeSecurityFlags(logData),
    approvals: logData.approvals || [],
    relatedLogs: logData.relatedLogs || []
  };

  // Generate immutability proof
  const previousLog = await getLatestAuditLog();
  log.immutabilityProof = {
    hash: computeSHA256(JSON.stringify(log)),
    previousHash: previousLog?.immutabilityProof?.hash || "GENESIS",
    signature: await signLog(log)
  };

  // Save to immutable storage
  await saveAuditLog(log);

  // Check for security alerts
  if (log.securityFlags && log.securityFlags.length > 0) {
    await triggerSecurityAlert(log);
  }

  return log;
}

async function captureRequestContext() {
  return {
    ipAddress: getCurrentIPAddress(),
    userAgent: getCurrentUserAgent(),
    location: await geolocateIP(getCurrentIPAddress()),
    sessionId: getCurrentSessionId(),
    requestId: generateUUID()
  };
}

function analyzeSecurityFlags(logData) {
  const flags = [];

  // Check for unusual location
  const executor = getExecutor(logData.executorId);
  const currentLocation = getCurrentLocation();
  if (!isLocationAuthorized(executor, currentLocation)) {
    flags.push("suspicious-location");
  }

  // Check for unusual time
  const currentHour = new Date().getHours();
  if (currentHour < 6 || currentHour > 22) {
    flags.push("unusual-time");
  }

  // Check for rapid actions
  const recentLogs = getRecentLogs(logData.executorId, 5); // Last 5 minutes
  if (recentLogs.length > 10) {
    flags.push("rapid-actions");
  }

  // Check for high-risk actions
  const highRiskActions = ["delete-accounts", "transfer-assets", "modify-permissions"];
  if (highRiskActions.includes(logData.action)) {
    flags.push("high-risk-action");
  }

  return flags;
}
```

---

## Conclusion

This Phase 1 specification provides comprehensive data format definitions for the WIA Digital Executor Standard. All implementations must adhere to these schemas to ensure interoperability and compliance.

**Next Steps:**
- Proceed to Phase 2 for API interface specifications
- Review legal requirements for your jurisdiction
- Implement validation and security measures
- Establish audit trail mechanisms

---

**弘益人間 (홍익인간)** - Benefit All Humanity
© 2025 WIA
MIT License
