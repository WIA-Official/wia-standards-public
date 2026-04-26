# WIA-DIGITAL_EXECUTOR PHASE 2 — API Interface Specification

**Standard:** WIA-DIGITAL_EXECUTOR
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

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
